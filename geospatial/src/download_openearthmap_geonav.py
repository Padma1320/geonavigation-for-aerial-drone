from collections import defaultdict
from pathlib import Path, PurePosixPath
from remotezip import RemoteZip

import numpy as np


URL = "https://zenodo.org/records/7223446/files/OpenEarthMap.zip?download=1"

BASE = Path.home() / "geonav_ai" / "geospatial"
OUT_ROOT = BASE / "data" / "openearthmap_geonav"


# Number of image-label pairs to download from each region
SPLITS = {
    "train": {
        "aachen": 20,
        "accra": 20,
        "daressalaam": 20,
        "kagera": 20,
        "kyoto": 20,
        "vegas": 20,
    },

    "val": {
        "melbourne": 20,
        "zanzibar": 20,
        "sechura": 5,
    },

    "test": {
        "rotterdam": 20,
        "ulaanbaatar": 20,
    },
}


def get_region(path):
    parts = PurePosixPath(path).parts

    if "images" not in parts:
        return None

    idx = parts.index("images")

    if idx == 0:
        return None

    return parts[idx - 1]


def select_spread(files, count):
    """
    Select files spread across the full region list instead of
    simply taking the first N files.
    """

    files = sorted(files)

    if count >= len(files):
        return files

    indices = np.linspace(
        0,
        len(files) - 1,
        count,
        dtype=int,
    )

    # np.linspace with integer conversion can theoretically duplicate
    # values, so preserve order while removing duplicates.
    selected = []

    seen = set()

    for idx in indices:

        filename = files[int(idx)]

        if filename not in seen:
            selected.append(filename)
            seen.add(filename)

    return selected


print("=" * 76)
print("GeoNav-AI | OpenEarthMap supervised dataset downloader")
print("=" * 76)

print("\nDestination:")
print(OUT_ROOT)

print("\nOpening remote OpenEarthMap archive...")

with RemoteZip(URL) as rz:

    names = rz.namelist()
    names_set = set(names)

    images_by_region = defaultdict(list)

    # ---------------------------------------------------------------
    # Build inventory of valid image/label pairs
    # ---------------------------------------------------------------

    for name in names:

        if "/images/" not in name.lower():
            continue

        if not name.lower().endswith(".tif"):
            continue

        label_name = name.replace(
            "/images/",
            "/labels/",
        )

        if label_name not in names_set:
            continue

        region = get_region(name)

        if region is not None:
            images_by_region[region].append(name)

    total_expected = sum(
        count
        for split in SPLITS.values()
        for count in split.values()
    )

    print(f"\nTarget image-label pairs: {total_expected}")

    downloaded_pairs = 0
    skipped_pairs = 0
    failed_pairs = 0

    # ---------------------------------------------------------------
    # Download
    # ---------------------------------------------------------------

    for split_name, regions in SPLITS.items():

        print()
        print("=" * 76)
        print(split_name.upper())
        print("=" * 76)

        image_dir = OUT_ROOT / split_name / "images"
        label_dir = OUT_ROOT / split_name / "labels"

        image_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        label_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        for region, requested_count in regions.items():

            available = images_by_region.get(
                region,
                [],
            )

            if not available:

                print(
                    f"\nERROR: no valid pairs found for {region}"
                )

                continue

            selected = select_spread(
                available,
                requested_count,
            )

            print()
            print(
                f"{region}: "
                f"{len(selected)} selected "
                f"from {len(available)} available"
            )

            for image_name in selected:

                filename = Path(image_name).name

                label_name = image_name.replace(
                    "/images/",
                    "/labels/",
                )

                # Prefix region so filenames can never collide
                output_name = (
                    f"{region}__{filename}"
                )

                image_out = (
                    image_dir / output_name
                )

                label_out = (
                    label_dir / output_name
                )

                # ---------------------------------------------------
                # Resume support
                # ---------------------------------------------------

                if (
                    image_out.exists()
                    and label_out.exists()
                    and image_out.stat().st_size > 0
                    and label_out.stat().st_size > 0
                ):

                    skipped_pairs += 1

                    print(
                        f"  SKIP existing: {output_name}"
                    )

                    continue

                print(
                    f"  Downloading: {output_name}"
                )

                try:

                    image_bytes = rz.read(
                        image_name
                    )

                    label_bytes = rz.read(
                        label_name
                    )

                    # Write only after both downloads succeed
                    image_out.write_bytes(
                        image_bytes
                    )

                    label_out.write_bytes(
                        label_bytes
                    )

                    downloaded_pairs += 1

                    print(
                        "    OK | "
                        f"image "
                        f"{len(image_bytes) / 1024 / 1024:.2f} MB | "
                        f"label "
                        f"{len(label_bytes) / 1024 / 1024:.2f} MB"
                    )

                except KeyboardInterrupt:

                    print()
                    print("=" * 76)
                    print("Download interrupted by user.")
                    print(
                        "Run this script again to resume."
                    )
                    print("=" * 76)

                    raise

                except Exception as exc:

                    failed_pairs += 1

                    print(
                        f"    FAILED: {exc}"
                    )


print()
print("=" * 76)
print("DOWNLOAD SUMMARY")
print("=" * 76)

print(
    f"New pairs downloaded : {downloaded_pairs}"
)

print(
    f"Existing pairs skipped: {skipped_pairs}"
)

print(
    f"Failed pairs          : {failed_pairs}"
)

print()
print("Dataset location:")
print(OUT_ROOT)

print()
print("Expected structure:")

print("""
openearthmap_geonav/
├── train/
│   ├── images/
│   └── labels/
├── val/
│   ├── images/
│   └── labels/
└── test/
    ├── images/
    └── labels/
""")

print("=" * 76)
