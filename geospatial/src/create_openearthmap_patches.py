from pathlib import Path

import cv2
import numpy as np
import rasterio


BASE = Path.home() / "geonav_ai" / "geospatial"

SOURCE_ROOT = (
    BASE
    / "data"
    / "openearthmap_geonav"
)

PATCH_ROOT = (
    BASE
    / "data"
    / "openearthmap_geonav_512"
)

PATCH_SIZE = 512

SPLITS = [
    "train",
    "val",
    "test",
]


def read_image(path):
    with rasterio.open(path) as src:
        image = src.read()

    # CHW -> HWC
    image = np.moveaxis(
        image,
        0,
        -1,
    )

    return image


def read_label(path):
    with rasterio.open(path) as src:
        label = src.read(1)

    return label


def get_crop_positions(
    height,
    width,
):
    """
    Generate deterministic four-corner 512x512 crops.

    For 1024x1024:
        (0,0)
        (0,512)
        (512,0)
        (512,512)

    For other sizes >512, the final crop is
    anchored against the far image boundary.
    """

    if (
        height < PATCH_SIZE
        or width < PATCH_SIZE
    ):
        raise ValueError(
            f"Image is smaller than "
            f"{PATCH_SIZE}x{PATCH_SIZE}: "
            f"{height}x{width}"
        )

    top = 0
    bottom = height - PATCH_SIZE

    left = 0
    right = width - PATCH_SIZE

    positions = [
        (top, left),
        (top, right),
        (bottom, left),
        (bottom, right),
    ]

    # Avoid duplicate positions if dimensions
    # happen to equal the patch size.
    unique_positions = []

    seen = set()

    for position in positions:

        if position not in seen:
            unique_positions.append(
                position
            )
            seen.add(position)

    return unique_positions


print("=" * 78)
print("GeoNav-AI | 512x512 OpenEarthMap patch generator")
print("=" * 78)

print("\nSource:")
print(SOURCE_ROOT)

print("\nDestination:")
print(PATCH_ROOT)


grand_total = 0


for split in SPLITS:

    print()
    print("=" * 78)
    print(split.upper())
    print("=" * 78)

    source_images = (
        SOURCE_ROOT
        / split
        / "images"
    )

    source_labels = (
        SOURCE_ROOT
        / split
        / "labels"
    )

    output_images = (
        PATCH_ROOT
        / split
        / "images"
    )

    output_labels = (
        PATCH_ROOT
        / split
        / "labels"
    )

    output_images.mkdir(
        parents=True,
        exist_ok=True,
    )

    output_labels.mkdir(
        parents=True,
        exist_ok=True,
    )

    image_files = sorted(
        source_images.glob("*.tif")
    )

    split_patch_count = 0

    for i, image_path in enumerate(
        image_files,
        start=1,
    ):

        label_path = (
            source_labels
            / image_path.name
        )

        if not label_path.exists():

            raise FileNotFoundError(
                f"Missing label for "
                f"{image_path.name}"
            )

        image = read_image(
            image_path
        )

        label = read_label(
            label_path
        )

        if image.shape[:2] != label.shape:

            raise ValueError(
                f"Shape mismatch for "
                f"{image_path.name}: "
                f"image={image.shape[:2]} "
                f"label={label.shape}"
            )

        height, width = label.shape

        positions = get_crop_positions(
            height,
            width,
        )

        stem = image_path.stem

        for patch_number, (
            y,
            x,
        ) in enumerate(
            positions,
            start=1,
        ):

            image_patch = image[
                y:y + PATCH_SIZE,
                x:x + PATCH_SIZE,
            ]

            label_patch = label[
                y:y + PATCH_SIZE,
                x:x + PATCH_SIZE,
            ]

            if image_patch.shape[:2] != (
                PATCH_SIZE,
                PATCH_SIZE,
            ):
                raise ValueError(
                    f"Invalid image patch "
                    f"from {image_path.name}"
                )

            if label_patch.shape != (
                PATCH_SIZE,
                PATCH_SIZE,
            ):
                raise ValueError(
                    f"Invalid label patch "
                    f"from {image_path.name}"
                )

            output_name = (
                f"{stem}"
                f"__p{patch_number}"
                f"__y{y}"
                f"_x{x}.png"
            )

            image_output = (
                output_images
                / output_name
            )

            label_output = (
                output_labels
                / output_name
            )

            # Rasterio gives RGB.
            # OpenCV expects BGR when writing.
            if (
                image_patch.ndim == 3
                and image_patch.shape[2] == 3
            ):

                image_to_write = cv2.cvtColor(
                    image_patch,
                    cv2.COLOR_RGB2BGR,
                )

            else:

                raise ValueError(
                    f"Expected 3-band RGB image, "
                    f"got {image_patch.shape} "
                    f"in {image_path.name}"
                )

            success_image = cv2.imwrite(
                str(image_output),
                image_to_write,
            )

            success_label = cv2.imwrite(
                str(label_output),
                label_patch.astype(
                    np.uint8
                ),
            )

            if not success_image:
                raise RuntimeError(
                    f"Could not write "
                    f"{image_output}"
                )

            if not success_label:
                raise RuntimeError(
                    f"Could not write "
                    f"{label_output}"
                )

            split_patch_count += 1

        if (
            i % 20 == 0
            or i == len(image_files)
        ):

            print(
                f"Processed "
                f"{i}/{len(image_files)} "
                f"source images"
            )

    grand_total += split_patch_count

    print()
    print(
        f"{split.upper()} patches: "
        f"{split_patch_count}"
    )


print()
print("=" * 78)
print("PATCH GENERATION COMPLETE")
print("=" * 78)

print(
    f"Total patches: "
    f"{grand_total}"
)

print("\nDataset:")
print(PATCH_ROOT)

print("=" * 78)
