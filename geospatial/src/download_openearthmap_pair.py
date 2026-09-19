import os
from pathlib import Path
from remotezip import RemoteZip

URL = "https://zenodo.org/records/7223446/files/OpenEarthMap.zip?download=1"

BASE = Path.home() / "geonav_ai" / "geospatial"
OUT_IMAGE = BASE / "data" / "openearthmap_sample" / "images" / "sample.tif"
OUT_LABEL = BASE / "data" / "openearthmap_sample" / "labels" / "sample.tif"

OUT_IMAGE.parent.mkdir(parents=True, exist_ok=True)
OUT_LABEL.parent.mkdir(parents=True, exist_ok=True)

print("=" * 60)
print("GeoNav-AI | Download one OpenEarthMap image-mask pair")
print("=" * 60)

with RemoteZip(URL) as rz:
    names = rz.namelist()

    image_files = [
        n for n in names
        if "/images/" in n.lower()
        and n.lower().endswith((".tif", ".tiff"))
    ]

    name_set = set(names)

    pair = None

    for image_name in image_files:
        label_name = image_name.replace("/images/", "/labels/")

        if label_name in name_set:
            pair = (image_name, label_name)
            break

    if pair is None:
        raise RuntimeError("No matching image-label pair found.")

    image_name, label_name = pair

    print("\nSelected image:")
    print(image_name)

    print("\nMatching label:")
    print(label_name)

    print("\nDownloading image...")
    image_bytes = rz.read(image_name)

    print("Downloading label...")
    label_bytes = rz.read(label_name)

with open(OUT_IMAGE, "wb") as f:
    f.write(image_bytes)

with open(OUT_LABEL, "wb") as f:
    f.write(label_bytes)

print("\nSaved image:")
print(OUT_IMAGE)

print("\nSaved label:")
print(OUT_LABEL)

print("\nImage size:", OUT_IMAGE.stat().st_size, "bytes")
print("Label size:", OUT_LABEL.stat().st_size, "bytes")

print("\nDone.")
