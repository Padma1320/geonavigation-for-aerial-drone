from pathlib import Path

import numpy as np
import rasterio
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap


BASE = Path.home() / "geonav_ai" / "geospatial"
DATA_DIR = BASE / "data" / "openearthmap_subset"
IMAGE_DIR = DATA_DIR / "images"
LABEL_DIR = DATA_DIR / "labels"
OUTPUT = BASE / "outputs" / "openearthmap_subset_grid.png"

SAMPLES = [
    "kagera_51.tif",
    "kagera_23.tif",
    "kagera_59.tif",
    "kagera_12.tif",
    "kagera_69.tif",
]

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

# 9 discrete display classes
cmap = ListedColormap([
    "black",        # 0 background
    "tan",          # 1 bareland
    "khaki",        # 2 rangeland
    "lightgray",    # 3 developed space
    "dimgray",      # 4 road
    "forestgreen",  # 5 tree
    "royalblue",    # 6 water
    "gold",         # 7 agriculture
    "firebrick",    # 8 building
])

fig, axes = plt.subplots(
    nrows=len(SAMPLES),
    ncols=2,
    figsize=(12, 5 * len(SAMPLES)),
)

for row, filename in enumerate(SAMPLES):
    image_path = IMAGE_DIR / filename
    label_path = LABEL_DIR / filename

    with rasterio.open(image_path) as src:
        image = src.read()

    image = np.moveaxis(image, 0, -1)

    with rasterio.open(label_path) as src:
        label = src.read(1)

    classes, counts = np.unique(label, return_counts=True)

    present = [
        CLASS_NAMES[int(c)]
        for c in classes
        if int(c) in CLASS_NAMES
    ]

    print("\n", filename)
    print("Classes:", present)

    for c, count in zip(classes, counts):
        pct = 100.0 * count / label.size

        print(
            f"  {int(c):2d} "
            f"{CLASS_NAMES.get(int(c), 'unknown'):20s} "
            f"{pct:6.2f}%"
        )

    axes[row, 0].imshow(image)
    axes[row, 0].set_title(
        f"{filename} | RGB"
    )
    axes[row, 0].axis("off")

    axes[row, 1].imshow(
        label,
        cmap=cmap,
        vmin=0,
        vmax=8,
        interpolation="nearest",
    )

    axes[row, 1].set_title(
        "Ground Truth\n"
        + ", ".join(present)
    )

    axes[row, 1].axis("off")

plt.tight_layout()

OUTPUT.parent.mkdir(parents=True, exist_ok=True)

plt.savefig(
    OUTPUT,
    dpi=160,
    bbox_inches="tight",
)

plt.close()

print("\nSaved visualization:")
print(OUTPUT)
