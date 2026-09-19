from pathlib import Path
from collections import Counter, defaultdict

import numpy as np
import rasterio


BASE = Path.home() / "geonav_ai" / "geospatial"
DATA_ROOT = BASE / "data" / "openearthmap_geonav"

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

SPLITS = ["train", "val", "test"]


def get_region(filename):
    """
    Example:
    aachen__aachen_12.tif -> aachen
    """
    return filename.split("__", 1)[0]


print("=" * 78)
print("GeoNav-AI | OpenEarthMap dataset validation")
print("=" * 78)

all_ok = True

for split in SPLITS:

    print()
    print("=" * 78)
    print(split.upper())
    print("=" * 78)

    image_dir = DATA_ROOT / split / "images"
    label_dir = DATA_ROOT / split / "labels"

    image_files = sorted(image_dir.glob("*.tif"))
    label_files = sorted(label_dir.glob("*.tif"))

    image_names = {p.name for p in image_files}
    label_names = {p.name for p in label_files}

    missing_labels = image_names - label_names
    missing_images = label_names - image_names

    print(f"Images : {len(image_files)}")
    print(f"Labels : {len(label_files)}")

    if missing_labels:
        print("\nERROR: images missing labels:")
        for name in sorted(missing_labels):
            print(" ", name)
        all_ok = False

    if missing_images:
        print("\nERROR: labels missing images:")
        for name in sorted(missing_images):
            print(" ", name)
        all_ok = False

    matched = sorted(image_names & label_names)

    pixel_counts = Counter()
    region_counts = Counter()
    shape_counts = Counter()

    corrupt = []
    shape_mismatch = []
    invalid_classes = defaultdict(set)

    for i, name in enumerate(matched, start=1):

        image_path = image_dir / name
        label_path = label_dir / name

        region = get_region(name)
        region_counts[region] += 1

        try:
            with rasterio.open(image_path) as src:
                image = src.read()

            with rasterio.open(label_path) as src:
                label = src.read(1)

        except Exception as exc:
            corrupt.append((name, str(exc)))
            all_ok = False
            continue

        if image.ndim != 3:
            print(
                f"WARNING: unexpected image dimensions "
                f"{image.shape} in {name}"
            )

        image_h = image.shape[1]
        image_w = image.shape[2]

        label_h, label_w = label.shape

        shape_counts[
            (image_h, image_w)
        ] += 1

        if (
            image_h != label_h
            or image_w != label_w
        ):
            shape_mismatch.append(
                (
                    name,
                    (image_h, image_w),
                    (label_h, label_w),
                )
            )

            all_ok = False
            continue

        classes, counts = np.unique(
            label,
            return_counts=True,
        )

        for class_id, count in zip(
            classes,
            counts,
        ):

            class_id = int(class_id)

            if class_id not in CLASS_NAMES:
                invalid_classes[name].add(
                    class_id
                )
                all_ok = False

            pixel_counts[class_id] += int(count)

        if i % 25 == 0 or i == len(matched):
            print(
                f"Validated "
                f"{i}/{len(matched)} pairs"
            )

    print()
    print("-" * 78)
    print("REGION COUNTS")
    print("-" * 78)

    for region, count in sorted(
        region_counts.items()
    ):
        print(
            f"{region:20s} {count:4d}"
        )

    print()
    print("-" * 78)
    print("IMAGE SHAPES")
    print("-" * 78)

    for shape, count in sorted(
        shape_counts.items()
    ):
        print(
            f"{shape}: {count} images"
        )

    print()
    print("-" * 78)
    print("CLASS PIXEL DISTRIBUTION")
    print("-" * 78)

    total_pixels = sum(
        pixel_counts.values()
    )

    for class_id in sorted(
        pixel_counts
    ):

        count = pixel_counts[class_id]

        pct = (
            100.0 * count / total_pixels
            if total_pixels > 0
            else 0.0
        )

        print(
            f"{class_id}: "
            f"{CLASS_NAMES.get(class_id, 'UNKNOWN'):20s} "
            f"{count:12d} pixels "
            f"{pct:7.3f}%"
        )

    if corrupt:

        print()
        print("CORRUPT FILES:")

        for name, error in corrupt:
            print(
                f"{name}: {error}"
            )

    if shape_mismatch:

        print()
        print("SHAPE MISMATCHES:")

        for (
            name,
            image_shape,
            label_shape,
        ) in shape_mismatch:

            print(
                f"{name}: "
                f"image={image_shape}, "
                f"label={label_shape}"
            )

    if invalid_classes:

        print()
        print("INVALID CLASS IDs:")

        for name, classes in invalid_classes.items():
            print(
                f"{name}: "
                f"{sorted(classes)}"
            )


print()
print("=" * 78)

if all_ok:
    print("DATASET VALIDATION PASSED")
    print("All image-label pairs are structurally valid.")
else:
    print("DATASET VALIDATION FAILED")
    print("Review the errors above before training.")

print("=" * 78)
