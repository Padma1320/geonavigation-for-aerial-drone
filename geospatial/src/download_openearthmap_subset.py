from pathlib import Path
from remotezip import RemoteZip

URL = "https://zenodo.org/records/7223446/files/OpenEarthMap.zip?download=1"

BASE = Path.home() / "geonav_ai" / "geospatial"
OUT_BASE = BASE / "data" / "openearthmap_subset"

IMAGE_DIR = OUT_BASE / "images"
LABEL_DIR = OUT_BASE / "labels"

IMAGE_DIR.mkdir(parents=True, exist_ok=True)
LABEL_DIR.mkdir(parents=True, exist_ok=True)

SAMPLES = [
    "kagera_51.tif",
    "kagera_23.tif",
    "kagera_59.tif",
    "kagera_12.tif",
    "kagera_69.tif",
]

print("=" * 70)
print("GeoNav-AI | Download OpenEarthMap diverse subset")
print("=" * 70)

with RemoteZip(URL) as rz:
    names = set(rz.namelist())

    for filename in SAMPLES:

        image_name = (
            f"OpenEarthMap_wo_xBD/kagera/images/{filename}"
        )

        label_name = (
            f"OpenEarthMap_wo_xBD/kagera/labels/{filename}"
        )

        if image_name not in names:
            print(f"\nSKIP: image missing: {filename}")
            continue

        if label_name not in names:
            print(f"\nSKIP: label missing: {filename}")
            continue

        print(f"\nDownloading {filename}")

        image_bytes = rz.read(image_name)
        label_bytes = rz.read(label_name)

        image_out = IMAGE_DIR / filename
        label_out = LABEL_DIR / filename

        image_out.write_bytes(image_bytes)
        label_out.write_bytes(label_bytes)

        print(
            f"  image: {len(image_bytes) / 1024 / 1024:.2f} MB"
        )

        print(
            f"  label: {len(label_bytes) / 1024 / 1024:.2f} MB"
        )

print("\nSubset saved to:")
print(OUT_BASE)

print("\nDone.")
