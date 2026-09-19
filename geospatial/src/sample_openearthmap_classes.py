from pathlib import Path
from remotezip import RemoteZip
import rasterio
import numpy as np
import tempfile
import os

URL = "https://zenodo.org/records/7223446/files/OpenEarthMap.zip?download=1"

CLASS_NAMES = {
    0: "background",
    1: "bareland",
    2: "rangeland",
    3: "developed_space",
    4: "road",
    5: "tree",
    6: "water",
    7: "agriculture_land",
    8: "building",
}

TARGET_CLASSES = {1, 2, 3, 4, 5, 6, 7, 8}

MAX_CHECK = 80
MAX_SELECTED = 12

print("=" * 70)
print("GeoNav-AI | OpenEarthMap class-diversity sampler")
print("=" * 70)

selected = []
covered = set()

with RemoteZip(URL) as rz:
    names = rz.namelist()
    names_set = set(names)

    label_files = [
        n for n in names
        if "/labels/" in n.lower()
        and n.lower().endswith(".tif")
    ]

    checked = 0

    for label_name in label_files:
        if checked >= MAX_CHECK:
            break

        image_name = label_name.replace("/labels/", "/images/")

        if image_name not in names_set:
            continue

        checked += 1

        label_bytes = rz.read(label_name)

        with tempfile.NamedTemporaryFile(suffix=".tif", delete=False) as tmp:
            tmp.write(label_bytes)
            tmp_path = tmp.name

        try:
            with rasterio.open(tmp_path) as src:
                label = src.read(1)

            classes = set(int(x) for x in np.unique(label))
        finally:
            os.unlink(tmp_path)

        useful_classes = classes & TARGET_CLASSES
        new_classes = useful_classes - covered

        if new_classes or len(selected) < 3:
            selected.append(
                {
                    "image": image_name,
                    "label": label_name,
                    "classes": sorted(useful_classes),
                }
            )

            covered |= useful_classes

            print("\nSelected:")
            print(image_name)
            print(
                "Classes:",
                [
                    CLASS_NAMES[c]
                    for c in sorted(useful_classes)
                ]
            )

            print(
                "Total coverage:",
                [
                    CLASS_NAMES[c]
                    for c in sorted(covered)
                ]
            )

        if covered == TARGET_CLASSES:
            break

        if len(selected) >= MAX_SELECTED:
            break

print("\n" + "=" * 70)
print("SUMMARY")
print("=" * 70)

print("Files checked:", checked)
print("Samples selected:", len(selected))

print("\nClasses covered:")

for c in sorted(covered):
    print(f"{c}: {CLASS_NAMES[c]}")

missing = TARGET_CLASSES - covered

if missing:
    print("\nStill missing:")
    for c in sorted(missing):
        print(f"{c}: {CLASS_NAMES[c]}")
else:
    print("\nAll 8 land-cover classes covered.")

print("\nSelected image-label pairs:")

for i, item in enumerate(selected, start=1):
    print(f"\n{i}.")
    print("Image:", item["image"])
    print("Label:", item["label"])
    print(
        "Classes:",
        [CLASS_NAMES[c] for c in item["classes"]]
    )
