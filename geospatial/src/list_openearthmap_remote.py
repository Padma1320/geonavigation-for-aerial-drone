from remotezip import RemoteZip

URL = "https://zenodo.org/records/7223446/files/OpenEarthMap.zip?download=1"

print("=" * 60)
print("GeoNav-AI | Remote OpenEarthMap archive inspection")
print("=" * 60)

print("\nConnecting to Zenodo archive...")
print("This should read only the ZIP directory, not download 9.1 GB.\n")

with RemoteZip(URL) as rz:
    names = rz.namelist()

print(f"Total files in archive: {len(names)}")

print("\nFirst 100 entries:")
print("-" * 60)

for name in names[:100]:
    print(name)

print("\nSearching for image TIFF files...")
print("-" * 60)

image_files = [
    name for name in names
    if "/images/" in name.lower()
    and name.lower().endswith((".tif", ".tiff"))
]

print(f"Image TIFF files found: {len(image_files)}")

for name in image_files[:30]:
    print(name)

print("\nSearching for label TIFF files...")
print("-" * 60)

label_files = [
    name for name in names
    if "/labels/" in name.lower()
    and name.lower().endswith((".tif", ".tiff"))
]

print(f"Label TIFF files found: {len(label_files)}")

for name in label_files[:30]:
    print(name)
