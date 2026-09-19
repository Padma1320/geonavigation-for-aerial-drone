import os
import numpy as np
import rasterio
import matplotlib.pyplot as plt

from sklearn.preprocessing import StandardScaler
from sklearn.cluster import KMeans


# ============================================================
# PATHS
# ============================================================

BASE_DIR = os.path.expanduser("~/geonav_ai/geospatial")

RAW_DIR = os.path.join(BASE_DIR, "data", "raw")
PROCESSED_DIR = os.path.join(BASE_DIR, "data", "processed")
OUTPUT_DIR = os.path.join(BASE_DIR, "outputs")

os.makedirs(PROCESSED_DIR, exist_ok=True)
os.makedirs(OUTPUT_DIR, exist_ok=True)

B02_PATH = os.path.join(RAW_DIR, "B02.tif")
B03_PATH = os.path.join(RAW_DIR, "B03.tif")
B04_PATH = os.path.join(RAW_DIR, "B04.tif")
B08_PATH = os.path.join(RAW_DIR, "B08.tif")

NDVI_PATH = os.path.join(PROCESSED_DIR, "NDVI.tif")
NDWI_PATH = os.path.join(PROCESSED_DIR, "NDWI.tif")


# ============================================================
# READ RASTER
# ============================================================

def read_raster(path):

    with rasterio.open(path) as src:

        data = src.read(1).astype(np.float32)

        profile = src.profile.copy()

        transform = src.transform

        crs = src.crs

    return data, profile, transform, crs


blue, profile, transform, crs = read_raster(B02_PATH)
green, _, _, _ = read_raster(B03_PATH)
red, _, _, _ = read_raster(B04_PATH)
nir, _, _, _ = read_raster(B08_PATH)

ndvi, _, _, _ = read_raster(NDVI_PATH)
ndwi, _, _, _ = read_raster(NDWI_PATH)


# ============================================================
# CLEAN NODATA
# ============================================================

ndvi[ndvi <= -9990] = np.nan
ndwi[ndwi <= -9990] = np.nan

valid_mask = (
    np.isfinite(ndvi)
    & np.isfinite(ndwi)
    & (blue > 0)
    & (green > 0)
    & (red > 0)
    & (nir > 0)
)

height, width = red.shape

print("\n==========================================")
print("GeoNav Land-Cover K-Means Baseline")
print("==========================================")

print("Raster shape:", red.shape)
print("CRS:", crs)
print("Valid pixels:", np.sum(valid_mask))


# ============================================================
# REFLECTANCE NORMALIZATION
# ============================================================

# Sentinel-2 Level-2A bands are stored as scaled integers.
# Dividing by 10000 gives approximately scaled reflectance.

blue_ref = blue / 10000.0
green_ref = green / 10000.0
red_ref = red / 10000.0
nir_ref = nir / 10000.0


# ============================================================
# BRIGHTNESS FEATURE
# ============================================================

brightness = (
    blue_ref
    + green_ref
    + red_ref
) / 3.0


# ============================================================
# BUILD FEATURE MATRIX
# ============================================================

features = np.column_stack(
    (
        blue_ref[valid_mask],
        green_ref[valid_mask],
        red_ref[valid_mask],
        nir_ref[valid_mask],
        ndvi[valid_mask],
        ndwi[valid_mask],
        brightness[valid_mask],
    )
)

print("Feature matrix:", features.shape)

print("\nFeatures:")
print("1. Blue")
print("2. Green")
print("3. Red")
print("4. NIR")
print("5. NDVI")
print("6. NDWI")
print("7. Brightness")


# ============================================================
# STANDARDIZE FEATURES
# ============================================================

scaler = StandardScaler()

features_scaled = scaler.fit_transform(features)


# ============================================================
# K-MEANS
# ============================================================

NUM_CLUSTERS = 5

print("\nRunning K-Means with", NUM_CLUSTERS, "clusters...")

kmeans = KMeans(
    n_clusters=NUM_CLUSTERS,
    random_state=42,
    n_init=20,
)

cluster_labels = kmeans.fit_predict(features_scaled)

print("K-Means COMPLETE")


# ============================================================
# CREATE FULL CLUSTER MAP
# ============================================================

cluster_map = np.full(
    (height, width),
    -1,
    dtype=np.int16
)

cluster_map[valid_mask] = cluster_labels


# ============================================================
# CLUSTER STATISTICS
# ============================================================

cluster_stats = []

print("\n==========================================")
print("CLUSTER SPECTRAL STATISTICS")
print("==========================================")

for cluster_id in range(NUM_CLUSTERS):

    mask = cluster_labels == cluster_id

    count = np.sum(mask)

    mean_blue = np.mean(
        blue_ref[valid_mask][mask]
    )

    mean_green = np.mean(
        green_ref[valid_mask][mask]
    )

    mean_red = np.mean(
        red_ref[valid_mask][mask]
    )

    mean_nir = np.mean(
        nir_ref[valid_mask][mask]
    )

    mean_ndvi = np.mean(
        ndvi[valid_mask][mask]
    )

    mean_ndwi = np.mean(
        ndwi[valid_mask][mask]
    )

    mean_brightness = np.mean(
        brightness[valid_mask][mask]
    )

    cluster_stats.append(
        {
            "id": cluster_id,
            "count": int(count),
            "blue": mean_blue,
            "green": mean_green,
            "red": mean_red,
            "nir": mean_nir,
            "ndvi": mean_ndvi,
            "ndwi": mean_ndwi,
            "brightness": mean_brightness,
        }
    )

    print(f"\nCluster {cluster_id}")
    print("--------------------------")
    print("Pixels:", count)
    print("Blue:", round(float(mean_blue), 3))
    print("Green:", round(float(mean_green), 3))
    print("Red:", round(float(mean_red), 3))
    print("NIR:", round(float(mean_nir), 3))
    print("NDVI:", round(float(mean_ndvi), 3))
    print("NDWI:", round(float(mean_ndwi), 3))
    print(
        "Brightness:",
        round(float(mean_brightness), 3)
    )


# ============================================================
# AUTOMATIC SPECTRAL INTERPRETATION
# ============================================================

# IMPORTANT:
# These are spectral interpretations, not ground-truth labels.
#
# K-Means does not know semantic classes.
# We assign approximate meanings based on cluster means.

semantic_names = {}

remaining_clusters = set(range(NUM_CLUSTERS))


# ------------------------------------------------------------
# WATER
# ------------------------------------------------------------

water_cluster = max(
    remaining_clusters,
    key=lambda i: cluster_stats[i]["ndwi"]
)

semantic_names[water_cluster] = "Likely Water"

remaining_clusters.remove(water_cluster)


# ------------------------------------------------------------
# VEGETATION
# ------------------------------------------------------------

vegetation_cluster = max(
    remaining_clusters,
    key=lambda i: cluster_stats[i]["ndvi"]
)

semantic_names[vegetation_cluster] = "Likely Vegetation"

remaining_clusters.remove(vegetation_cluster)


# ------------------------------------------------------------
# BRIGHT / OPEN / IMPERVIOUS SURFACE
# ------------------------------------------------------------

bright_cluster = max(
    remaining_clusters,
    key=lambda i: cluster_stats[i]["brightness"]
)

semantic_names[bright_cluster] = "Bright Built/Open Surface"

remaining_clusters.remove(bright_cluster)


# ------------------------------------------------------------
# REMAINING TWO CLUSTERS
# ------------------------------------------------------------

remaining_list = list(remaining_clusters)

remaining_list.sort(
    key=lambda i: cluster_stats[i]["brightness"]
)

if len(remaining_list) == 2:

    semantic_names[
        remaining_list[0]
    ] = "Dark Built/Mixed Surface"

    semantic_names[
        remaining_list[1]
    ] = "Mixed Urban/Open Surface"


# ============================================================
# PRINT INTERPRETATION
# ============================================================

print("\n==========================================")
print("AUTOMATIC CLUSTER INTERPRETATION")
print("==========================================")

for cluster_id in range(NUM_CLUSTERS):

    print(
        f"Cluster {cluster_id}: "
        f"{semantic_names.get(cluster_id, 'Unknown')}"
    )


# ============================================================
# CREATE SEMANTIC MAP
# ============================================================

semantic_map = np.full(
    (height, width),
    -1,
    dtype=np.int16
)

# Semantic IDs
#
# 0 = likely water
# 1 = likely vegetation
# 2 = bright built/open
# 3 = dark built/mixed
# 4 = mixed urban/open

semantic_id_lookup = {}

for cluster_id, name in semantic_names.items():

    if name == "Likely Water":
        semantic_id_lookup[cluster_id] = 0

    elif name == "Likely Vegetation":
        semantic_id_lookup[cluster_id] = 1

    elif name == "Bright Built/Open Surface":
        semantic_id_lookup[cluster_id] = 2

    elif name == "Dark Built/Mixed Surface":
        semantic_id_lookup[cluster_id] = 3

    elif name == "Mixed Urban/Open Surface":
        semantic_id_lookup[cluster_id] = 4


for cluster_id, semantic_id in semantic_id_lookup.items():

    semantic_map[
        cluster_map == cluster_id
    ] = semantic_id


# ============================================================
# SAVE RAW K-MEANS CLUSTER MAP
# ============================================================

cluster_output = os.path.join(
    PROCESSED_DIR,
    "landcover_clusters.tif"
)

cluster_profile = profile.copy()

cluster_profile.update(
    dtype=rasterio.int16,
    count=1,
    nodata=-1
)

with rasterio.open(
    cluster_output,
    "w",
    **cluster_profile
) as dst:

    dst.write(
        cluster_map.astype(np.int16),
        1
    )


# ============================================================
# SAVE SEMANTIC INTERPRETATION
# ============================================================

semantic_output = os.path.join(
    PROCESSED_DIR,
    "landcover_baseline.tif"
)

with rasterio.open(
    semantic_output,
    "w",
    **cluster_profile
) as dst:

    dst.write(
        semantic_map.astype(np.int16),
        1
    )


# ============================================================
# VISUALIZE RAW CLUSTERS
# ============================================================

cluster_png = os.path.join(
    OUTPUT_DIR,
    "landcover_clusters.png"
)

plt.figure(figsize=(9, 9))

image = plt.imshow(
    np.ma.masked_where(
        cluster_map < 0,
        cluster_map
    ),
    cmap="tab10"
)

plt.colorbar(
    image,
    label="K-Means Cluster"
)

plt.title(
    "GeoNav Sentinel-2 K-Means Land-Cover Clusters"
)

plt.axis("off")

plt.tight_layout()

plt.savefig(
    cluster_png,
    dpi=200,
    bbox_inches="tight"
)

plt.close()


# ============================================================
# VISUALIZE SEMANTIC BASELINE
# ============================================================

semantic_png = os.path.join(
    OUTPUT_DIR,
    "landcover_baseline.png"
)

plt.figure(figsize=(9, 9))

image = plt.imshow(
    np.ma.masked_where(
        semantic_map < 0,
        semantic_map
    ),
    cmap="tab10",
    vmin=0,
    vmax=4
)

cbar = plt.colorbar(
    image,
    ticks=[0, 1, 2, 3, 4]
)

cbar.ax.set_yticklabels(
    [
        "Likely Water",
        "Likely Vegetation",
        "Bright Built/Open",
        "Dark Built/Mixed",
        "Mixed Urban/Open",
    ]
)

plt.title(
    "GeoNav Spectral Land-Cover Baseline"
)

plt.axis("off")

plt.tight_layout()

plt.savefig(
    semantic_png,
    dpi=200,
    bbox_inches="tight"
)

plt.close()


# ============================================================
# FINISH
# ============================================================

print("\n==========================================")
print("FILES SAVED")
print("==========================================")

print(cluster_output)
print(semantic_output)
print(cluster_png)
print(semantic_png)

print(
    "\nIMPORTANT: these are spectral baseline labels, "
    "not ground-truth semantic segmentation."
)

print(
    "\nGeoNav land-cover K-Means baseline COMPLETE"
)
