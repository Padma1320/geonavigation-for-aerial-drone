from collections import defaultdict
from remotezip import RemoteZip

URL = "https://zenodo.org/records/7223446/files/OpenEarthMap.zip?download=1"

print("=" * 70)
print("GeoNav-AI | OpenEarthMap geographic region inventory")
print("=" * 70)

with RemoteZip(URL) as rz:

    names = rz.namelist()
    names_set = set(names)

    regions = defaultdict(list)

    for name in names:

        if "/images/" not in name.lower():
            continue

        if not name.lower().endswith(".tif"):
            continue

        label_name = name.replace("/images/", "/labels/")

        # Only count image/label pairs that actually exist
        if label_name not in names_set:
            continue

        parts = name.split("/")

        try:
            images_index = parts.index("images")
        except ValueError:
            continue

        if images_index == 0:
            continue

        region = parts[images_index - 1]

        regions[region].append(name)


print(f"\nMatched geographic regions: {len(regions)}")
print()

for i, region in enumerate(sorted(regions), start=1):

    count = len(regions[region])

    print(
        f"{i:3d}. "
        f"{region:30s} "
        f"{count:4d} matched pairs"
    )


print("\n" + "=" * 70)

total = sum(len(files) for files in regions.values())

print(f"Total matched image/label pairs: {total}")
print("=" * 70)
