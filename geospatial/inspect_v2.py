from pathlib import Path
import torch

MODEL_PATH = Path(
    "/home/padma/geonav_ai/models/geospatial/"
    "geonav_unet_resnet34_best.pth"
)

print("=== GeoNav V2 Checkpoint ===")
print("Path:", MODEL_PATH)
print("Exists:", MODEL_PATH.exists())

if not MODEL_PATH.exists():
    raise FileNotFoundError(MODEL_PATH)

print(
    "Size:",
    round(MODEL_PATH.stat().st_size / (1024 ** 2), 2),
    "MB"
)

checkpoint = torch.load(
    MODEL_PATH,
    map_location="cpu"
)

print("\nCheckpoint loaded successfully")
print("Object type:", type(checkpoint))

if isinstance(checkpoint, dict):

    print("\nTop-level keys:")
    for key in checkpoint.keys():
        print("  ", key)

    if "model_state_dict" in checkpoint:
        state = checkpoint["model_state_dict"]
        print("\nStructure: checkpoint containing model_state_dict")

    elif "state_dict" in checkpoint:
        state = checkpoint["state_dict"]
        print("\nStructure: checkpoint containing state_dict")

    else:
        state = checkpoint
        print("\nStructure: likely raw state_dict")

    print("Number of entries:", len(state))

    print("\nFirst 10 entries:")
    for key in list(state.keys())[:10]:
        value = state[key]

        if hasattr(value, "shape"):
            print(f"  {key}: {tuple(value.shape)}")
        else:
            print(f"  {key}: {type(value)}")
