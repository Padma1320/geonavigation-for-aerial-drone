import os
import numpy as np
import rasterio
import matplotlib.pyplot as plt

# --------------------------------------------------
# PATHS
# --------------------------------------------------

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

# --------------------------------------------------
# READ BANDS
# --------------------------------------------------

def read_band(path):
    with rasterio.open(path) as src:
        data = src.read(1).astype(np.float32)
        profile = src.profile.copy()
        transform = src.transform
        crs = src.crs

    return data, profile, transform, crs


blue, profile, transform, crs = read_band(B02_PATH)
green, _, _, _ = read_band(B03_PATH)
red, _, _, _ = read_band(B04_PATH)
nir, _, _, _ = read_band(B08_PATH)

print("Bands loaded")
print("Shape:", red.shape)
print("CRS:", crs)
print("Transform:", transform)

# --------------------------------------------------
# REMOVE INVALID VALUES
# --------------------------------------------------

valid_mask = (
    (blue > 0) &
    (green > 0) &
    (red > 0) &
    (nir > 0)
)

# Sentinel-2 L2A values are scaled reflectance DNs.
# Ratios like NDVI/NDWI can be computed directly because
# the common scale factor cancels out.

# --------------------------------------------------
# NDVI
# NDVI = (NIR - RED) / (NIR + RED)
# --------------------------------------------------

ndvi = np.full(red.shape, np.nan, dtype=np.float32)

denominator = nir + red

valid_ndvi = valid_mask & (denominator != 0)

ndvi[valid_ndvi] = (
    nir[valid_ndvi] - red[valid_ndvi]
) / denominator[valid_ndvi]

# --------------------------------------------------
# NDWI
# McFeeters NDWI = (GREEN - NIR) / (GREEN + NIR)
# --------------------------------------------------

ndwi = np.full(green.shape, np.nan, dtype=np.float32)

denominator = green + nir

valid_ndwi = valid_mask & (denominator != 0)

ndwi[valid_ndwi] = (
    green[valid_ndwi] - nir[valid_ndwi]
) / denominator[valid_ndwi]

# --------------------------------------------------
# SAVE NDVI AND NDWI AS GEOTIFF
# --------------------------------------------------

index_profile = profile.copy()

index_profile.update(
    dtype=rasterio.float32,
    count=1,
    nodata=-9999.0
)

ndvi_output = os.path.join(
    PROCESSED_DIR,
    "NDVI.tif"
)

ndwi_output = os.path.join(
    PROCESSED_DIR,
    "NDWI.tif"
)

with rasterio.open(
    ndvi_output,
    "w",
    **index_profile
) as dst:

    output = np.where(
        np.isnan(ndvi),
        -9999.0,
        ndvi
    ).astype(np.float32)

    dst.write(output, 1)


with rasterio.open(
    ndwi_output,
    "w",
    **index_profile
) as dst:

    output = np.where(
        np.isnan(ndwi),
        -9999.0,
        ndwi
    ).astype(np.float32)

    dst.write(output, 1)

print("\nSaved:")
print(ndvi_output)
print(ndwi_output)

# --------------------------------------------------
# RGB VISUALIZATION
# --------------------------------------------------

def percentile_stretch(channel, lower=2, upper=98):

    valid = channel[channel > 0]

    low = np.percentile(valid, lower)
    high = np.percentile(valid, upper)

    stretched = (
        channel - low
    ) / (
        high - low
    )

    return np.clip(
        stretched,
        0,
        1
    )


r_display = percentile_stretch(red)
g_display = percentile_stretch(green)
b_display = percentile_stretch(blue)

rgb = np.dstack(
    (
        r_display,
        g_display,
        b_display
    )
)

# --------------------------------------------------
# SAVE FIGURES
# --------------------------------------------------

rgb_png = os.path.join(
    OUTPUT_DIR,
    "sentinel_rgb.png"
)

ndvi_png = os.path.join(
    OUTPUT_DIR,
    "ndvi.png"
)

ndwi_png = os.path.join(
    OUTPUT_DIR,
    "ndwi.png"
)

plt.figure(figsize=(8, 8))
plt.imshow(rgb)
plt.title("Sentinel-2 True Colour RGB")
plt.axis("off")
plt.tight_layout()
plt.savefig(
    rgb_png,
    dpi=200,
    bbox_inches="tight"
)
plt.close()


plt.figure(figsize=(8, 8))
image = plt.imshow(
    ndvi,
    vmin=-1,
    vmax=1,
    cmap="RdYlGn"
)
plt.colorbar(
    image,
    label="NDVI"
)
plt.title("Normalized Difference Vegetation Index")
plt.axis("off")
plt.tight_layout()
plt.savefig(
    ndvi_png,
    dpi=200,
    bbox_inches="tight"
)
plt.close()


plt.figure(figsize=(8, 8))
image = plt.imshow(
    ndwi,
    vmin=-1,
    vmax=1,
    cmap="BrBG"
)
plt.colorbar(
    image,
    label="NDWI"
)
plt.title("Normalized Difference Water Index")
plt.axis("off")
plt.tight_layout()
plt.savefig(
    ndwi_png,
    dpi=200,
    bbox_inches="tight"
)
plt.close()

# --------------------------------------------------
# BASIC STATISTICS
# --------------------------------------------------

valid_ndvi_values = ndvi[np.isfinite(ndvi)]
valid_ndwi_values = ndwi[np.isfinite(ndwi)]

print("\nNDVI statistics")
print("----------------")
print("Min:", np.min(valid_ndvi_values))
print("Max:", np.max(valid_ndvi_values))
print("Mean:", np.mean(valid_ndvi_values))

print("\nNDWI statistics")
print("----------------")
print("Min:", np.min(valid_ndwi_values))
print("Max:", np.max(valid_ndwi_values))
print("Mean:", np.mean(valid_ndwi_values))

print("\nImages saved:")
print(rgb_png)
print(ndvi_png)
print(ndwi_png)

print("\nGeoNav Sentinel processing COMPLETE")
