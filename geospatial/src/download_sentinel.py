from pystac_client import Client
import planetary_computer
import rasterio
from rasterio.windows import from_bounds
from rasterio.enums import Resampling
import numpy as np
import os

# --------------------------------------------------
# CONFIG
# --------------------------------------------------

# Small test region around Helsinki
# bbox = [min_lon, min_lat, max_lon, max_lat]
BBOX = [24.90, 60.15, 24.96, 60.19]

START_DATE = "2026-06-01"
END_DATE = "2026-08-31"

MAX_CLOUD = 20

OUTPUT_DIR = os.path.expanduser(
    "~/geonav_ai/geospatial/data/raw"
)

os.makedirs(OUTPUT_DIR, exist_ok=True)

# --------------------------------------------------
# CONNECT TO MICROSOFT PLANETARY COMPUTER STAC
# --------------------------------------------------

catalog = Client.open(
    "https://planetarycomputer.microsoft.com/api/stac/v1"
)

search = catalog.search(
    collections=["sentinel-2-l2a"],
    bbox=BBOX,
    datetime=f"{START_DATE}/{END_DATE}",
    query={
        "eo:cloud_cover": {
            "lt": MAX_CLOUD
        }
    }
)

items = list(search.items())

if len(items) == 0:
    raise RuntimeError("No Sentinel-2 images found.")

items.sort(
    key=lambda x: x.properties.get(
        "eo:cloud_cover",
        100
    )
)

item = items[0]

print("\nSelected Sentinel-2 scene")
print("--------------------------")
print("ID:", item.id)
print(
    "Cloud cover:",
    item.properties.get("eo:cloud_cover")
)
print(
    "Datetime:",
    item.properties.get("datetime")
)

# --------------------------------------------------
# BANDS WE WANT
# --------------------------------------------------

bands = {
    "B02": "B02",
    "B03": "B03",
    "B04": "B04",
    "B08": "B08",
}

saved_files = {}

# --------------------------------------------------
# DOWNLOAD ONLY OUR SMALL BBOX
# --------------------------------------------------

for band_name, asset_key in bands.items():

    asset = item.assets[asset_key]

    signed_asset = planetary_computer.sign(
        asset.href
    )

    print(
        f"\nReading {band_name}..."
    )

    with rasterio.open(signed_asset) as src:

        print("CRS:", src.crs)
        print("Full resolution:", src.width, "x", src.height)
        print("Pixel size:", src.res)
        print("Bounds:", src.bounds)

        # Convert lat/lon bounding box
        # into the raster CRS
        from rasterio.warp import transform_bounds

        raster_bbox = transform_bounds(
            "EPSG:4326",
            src.crs,
            *BBOX
        )

        window = from_bounds(
            *raster_bbox,
            transform=src.transform
        )

        data = src.read(
            1,
            window=window,
            boundless=True,
            resampling=Resampling.nearest
        )

        transform = src.window_transform(
            window
        )

        profile = src.profile.copy()

        profile.update(
            {
                "height": data.shape[0],
                "width": data.shape[1],
                "transform": transform,
                "count": 1,
            }
        )

        output_path = os.path.join(
            OUTPUT_DIR,
            f"{band_name}.tif"
        )

        with rasterio.open(
            output_path,
            "w",
            **profile
        ) as dst:
            dst.write(data, 1)

        saved_files[band_name] = output_path

        print(
            f"Saved: {output_path}"
        )
        print(
            f"Subset shape: {data.shape}"
        )
        print(
            f"Min/Max DN: {data.min()} / {data.max()}"
        )

print("\n--------------------------------")
print("Sentinel-2 download COMPLETE")
print("--------------------------------")

for name, path in saved_files.items():
    print(name, "->", path)
