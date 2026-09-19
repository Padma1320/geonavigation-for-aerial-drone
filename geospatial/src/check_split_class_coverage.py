from collections import defaultdict
from pathlib import PurePosixPath
from remotezip import RemoteZip

import numpy as np
import rasterio
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


SPLITS = {
    "train": [
        "aachen",
        "accra",
        "daressalaam",
        "kagera",
        "kyoto",
        "vegas",
    ],

    "val": [
        "melbourne",
        "zanzibar",
        "sechura",
    ],

    "test": [
        "rotterdam",
        "ulaanbaatar",
    ],
}


MASKS_PER_REGION = 5


def get_region(path):
    parts = PurePosixPath(path).parts

    if "labels" not in parts:
        return None

    index = parts.index("labels")

    if index == 0:
        return None

    return parts[index - 1]


def read_remote_mask(rz, path):
    data = rz.read(path)

    with tempfile.NamedTemporaryFile(
        suffix=".tif",
        delete=False
    ) as tmp:
        tmp.write(data)
        tmp_path = tmp.name

    try:
        with rasterio.open(tmp_path) as src:
            mask = src.read(1)
    finally:
        os.unlink(tmp_path)

    return mask


print("=" * 72)
print("GeoNav-AI | Revised geographic split class coverage")
print("=" * 72)


with RemoteZip(URL) as rz:

    names = rz.namelist()

    labels_by_region = defaultdict(list)

    for name in names:

        if "/labels/" not in name.lower():
            continue

        if not name.lower().endswith(".tif"):
            continue

        region = get_region(name)

        if region is not None:
            labels_by_region[region].append(name)


    for split_name, regions in SPLITS.items():

        print()
        print("=" * 72)
        print(split_name.upper())
        print("=" * 72)

        split_classes = set()

        for region in regions:

            files = sorted(labels_by_region.get(region, []))

            if not files:
                print(f"\n{region}: NO LABELS FOUND")
                continue

            indices = np.linspace(
                0,
                len(files) - 1,
                min(MASKS_PER_REGION, len(files)),
                dtype=int,
            )

            region_classes = set()

            for idx in indices:

                mask = read_remote_mask(
                    rz,
                    files[idx]
                )

                classes = set(
                    int(x)
                    for x in np.unique(mask)
                )

                region_classes |= classes
                split_classes |= classes

            useful = sorted(
                c
                for c in region_classes
                if c != 0 and c in CLASS_NAMES
            )

            print(
                f"\n{region:18s}: "
                + ", ".join(
                    CLASS_NAMES[c]
                    for c in useful
                )
            )


        print()
        print("-" * 72)
        print(f"{split_name.upper()} TOTAL COVERAGE")
        print("-" * 72)

        useful_split = sorted(
            c
            for c in split_classes
            if c != 0 and c in CLASS_NAMES
        )

        for c in useful_split:
            print(
                f"{c}: {CLASS_NAMES[c]}"
            )

        missing = (
            set(range(1, 9))
            - set(useful_split)
        )

        if missing:

            print("\nMissing classes:")

            for c in sorted(missing):
                print(
                    f"{c}: {CLASS_NAMES[c]}"
                )

        else:

            print(
                "\nAll 8 land-cover classes detected."
            )


print()
print("=" * 72)
print("Coverage inspection complete.")
print("=" * 72)
