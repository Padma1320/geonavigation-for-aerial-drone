import os
import rasterio
import numpy as np
import matplotlib.pyplot as plt

BASE = os.path.expanduser("~/geonav_ai/geospatial")
SAMPLE_DIR = os.path.join(BASE, "data", "openearthmap_sample")

os.makedirs(SAMPLE_DIR, exist_ok=True)

print("GeoNav OpenEarthMap sample checker")
print("----------------------------------")

print("Expected structure:")
print()
print("openearthmap_sample/")
print("├── images/")
print("│   └── sample.tif")
print("└── labels/")
print("    └── sample.tif")
print()

image_path = os.path.join(
    SAMPLE_DIR,
    "images",
    "sample.tif"
)

label_path = os.path.join(
    SAMPLE_DIR,
    "labels",
    "sample.tif"
)

if not os.path.exists(image_path):
    print("Image not found:")
    print(image_path)

if not os.path.exists(label_path):
    print("Label not found:")
    print(label_path)

if not (
    os.path.exists(image_path)
    and os.path.exists(label_path)
):
    print("\nWaiting for one valid image/mask pair.")
    raise SystemExit(0)

with rasterio.open(image_path) as src:
    image = src.read()

with rasterio.open(label_path) as src:
    label = src.read(1)

image = np.moveaxis(image, 0, -1)

print("\nImage shape:", image.shape)
print("Label shape:", label.shape)

classes = np.unique(label)

print("Classes present:", classes)

plt.figure(figsize=(8, 8))
plt.imshow(image)
plt.title("OpenEarthMap RGB image")
plt.axis("off")
plt.tight_layout()

plt.savefig(
    os.path.join(
        BASE,
        "outputs",
        "oem_sample_image.png"
    ),
    dpi=180,
)

plt.close()

plt.figure(figsize=(8, 8))
plt.imshow(
    label,
    cmap="tab10",
    vmin=0,
    vmax=8,
)

plt.colorbar()
plt.title("OpenEarthMap ground-truth mask")
plt.axis("off")
plt.tight_layout()

plt.savefig(
    os.path.join(
        BASE,
        "outputs",
        "oem_sample_mask.png"
    ),
    dpi=180,
)

plt.close()

print("\nSample pair verified successfully.")
