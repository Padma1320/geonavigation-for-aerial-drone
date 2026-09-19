#!/usr/bin/env python3

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import torch
import segmentation_models_pytorch as smp


# ============================================================
# CONFIGURATION
# ============================================================

MODEL_PATH = Path(
    "/home/padma/geonav_ai/models/geospatial/"
    "geonav_unet_resnet34_best.pth"
)

OUTPUT_DIR = Path(
    "/home/padma/geonav_ai/results/geospatial_v2"
)

OUTPUT_DIR.mkdir(
    parents=True,
    exist_ok=True
)

RESULT_JSON_PATH = (
    OUTPUT_DIR / "landing_zone_result.json"
)


# ============================================================
# OPENEARTHMAP CLASSES
# ============================================================

CLASS_NAMES = [
    "background",        # 0
    "bareland",          # 1
    "rangeland",         # 2
    "developed_space",   # 3
    "road",              # 4
    "tree",              # 5
    "water",             # 6
    "agriculture_land",  # 7
    "building",          # 8
]

# Preferred candidate landing surfaces
SAFE_CLASSES = {
    1,  # bareland
    2,  # rangeland
}

# Explicit hard hazards
HARD_HAZARDS = {
    4,  # road
    5,  # tree
    6,  # water
    8,  # building
}


# ============================================================
# LOAD V2 MODEL
# ============================================================

print("Loading V2 checkpoint...")

if not MODEL_PATH.exists():
    raise FileNotFoundError(
        f"Model checkpoint not found: {MODEL_PATH}"
    )

checkpoint = torch.load(
    MODEL_PATH,
    map_location="cpu"
)

print(
    "Training epoch:",
    checkpoint.get("epoch")
)

print(
    "Saved val mIoU:",
    checkpoint.get("val_miou")
)

print(
    "Saved model:",
    checkpoint.get("model")
)

print(
    "Encoder weights:",
    checkpoint.get("encoder_weights")
)

if "class_names" in checkpoint:
    print(
        "Checkpoint classes:",
        checkpoint["class_names"]
    )


# Reconstruct exact V2 architecture.
# encoder_weights=None because trained weights
# come from our checkpoint.
model = smp.Unet(
    encoder_name="resnet34",
    encoder_weights=None,
    in_channels=3,
    classes=9,
)

model.load_state_dict(
    checkpoint["model_state_dict"]
)

model.eval()

print("V2 model loaded successfully.")


# ============================================================
# SEMANTIC INFERENCE
# ============================================================

def predict(image_bgr):

    # V2 was trained using 512 x 512 RGB patches.
    image_512 = cv2.resize(
        image_bgr,
        (512, 512),
        interpolation=cv2.INTER_LINEAR
    )

    image_rgb = cv2.cvtColor(
        image_512,
        cv2.COLOR_BGR2RGB
    )

    image_float = (
        image_rgb.astype(np.float32)
        / 255.0
    )

    tensor = torch.from_numpy(
        image_float.transpose(2, 0, 1)
    ).float()

    tensor = tensor.unsqueeze(0)

    with torch.no_grad():

        logits = model(tensor)

        probabilities = torch.softmax(
            logits,
            dim=1
        )

        prediction = torch.argmax(
            probabilities,
            dim=1
        )[0]

    prediction = (
        prediction
        .cpu()
        .numpy()
        .astype(np.uint8)
    )

    probabilities = (
        probabilities[0]
        .cpu()
        .numpy()
    )

    return (
        image_512,
        prediction,
        probabilities
    )


# ============================================================
# LANDING-ZONE SELECTION
# ============================================================

def select_landing_zone(
    prediction,
    probabilities
):

    # Combined confidence that a pixel belongs
    # to one of the preferred landing classes.
    safe_probability = (
        probabilities[1]
        + probabilities[2]
    )

    # Semantic hard-hazard mask.
    hazard_mask = np.isin(
        prediction,
        list(HARD_HAZARDS)
    ).astype(np.uint8)

    # Candidate pixel must:
    # 1. be predicted as bareland/rangeland
    # 2. have sufficient combined safe probability
    candidate_mask = (
        np.isin(
            prediction,
            list(SAFE_CLASSES)
        )
        &
        (safe_probability >= 0.55)
    ).astype(np.uint8)

    # --------------------------------------------------------
    # HAZARD CLEARANCE
    # --------------------------------------------------------

    hazard_kernel = np.ones(
        (21, 21),
        dtype=np.uint8
    )

    expanded_hazards = cv2.dilate(
        hazard_mask,
        hazard_kernel,
        iterations=1
    )

    candidate_mask[
        expanded_hazards > 0
    ] = 0

    # --------------------------------------------------------
    # REMOVE SMALL/NOISY AREAS
    # --------------------------------------------------------

    clean_kernel = np.ones(
        (7, 7),
        dtype=np.uint8
    )

    candidate_mask = cv2.morphologyEx(
        candidate_mask,
        cv2.MORPH_OPEN,
        clean_kernel
    )

    # --------------------------------------------------------
    # CLEARANCE FROM UNSAFE SPACE
    # --------------------------------------------------------

    distance = cv2.distanceTransform(
        candidate_mask,
        cv2.DIST_L2,
        5
    )

    if distance.max() <= 0:

        return (
            None,
            candidate_mask,
            safe_probability
        )

    normalized_distance = (
        distance
        / (distance.max() + 1e-6)
    )

    # --------------------------------------------------------
    # FINAL SAFETY SCORE
    #
    # 65% spatial clearance
    # 35% semantic safe probability
    # --------------------------------------------------------

    score = (
        0.65 * normalized_distance
        + 0.35 * safe_probability
    )

    score[
        candidate_mask == 0
    ] = -1.0

    y, x = np.unravel_index(
        np.argmax(score),
        score.shape
    )

    predicted_class = int(
        prediction[y, x]
    )

    result = {
        "x": int(x),
        "y": int(y),

        "score": float(
            score[y, x]
        ),

        "safe_probability": float(
            safe_probability[y, x]
        ),

        "clearance_pixels": float(
            distance[y, x]
        ),

        "predicted_class":
            predicted_class,

        "class_name":
            CLASS_NAMES[predicted_class],
    }

    return (
        result,
        candidate_mask,
        safe_probability
    )


# ============================================================
# SAVE JSON RESULT
# ============================================================

def save_json_result(
    landing_result,
    image_path
):

    output = {
        "accepted":
            landing_result is not None,

        "image_width": 512,
        "image_height": 512,

        "source_image":
            str(image_path),
    }

    if landing_result is not None:
        output.update(
            landing_result
        )

    with open(
        RESULT_JSON_PATH,
        "w"
    ) as f:

        json.dump(
            output,
            f,
            indent=4
        )

    return output


# ============================================================
# SAVE VISUALIZATIONS
# ============================================================

def save_visualizations(
    image_512,
    prediction,
    landing_result,
    candidate_mask
):

    display = image_512.copy()

    # --------------------------------------------------------
    # SEMANTIC VISUALIZATION
    # --------------------------------------------------------

    semantic_vis = (
        prediction.astype(np.float32)
        / 8.0
        * 255.0
    ).astype(np.uint8)

    semantic_vis = cv2.applyColorMap(
        semantic_vis,
        cv2.COLORMAP_JET
    )

    cv2.imwrite(
        str(
            OUTPUT_DIR
            / "semantic_prediction.png"
        ),
        semantic_vis
    )

    # --------------------------------------------------------
    # LANDING CANDIDATE MASK
    # --------------------------------------------------------

    cv2.imwrite(
        str(
            OUTPUT_DIR
            / "landing_candidates.png"
        ),
        candidate_mask * 255
    )

    # --------------------------------------------------------
    # SELECTED LANDING POINT
    # --------------------------------------------------------

    if landing_result is not None:

        x = landing_result["x"]
        y = landing_result["y"]

        cv2.circle(
            display,
            (x, y),
            12,
            (0, 255, 0),
            3
        )

        cv2.drawMarker(
            display,
            (x, y),
            (0, 0, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=30,
            thickness=2
        )

        text = (
            f"{landing_result['class_name']} "
            f"P="
            f"{landing_result['safe_probability']:.2f}"
        )

        text_x = max(
            5,
            x - 100
        )

        text_y = max(
            25,
            y - 20
        )

        cv2.putText(
            display,
            text,
            (text_x, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2
        )

    cv2.imwrite(
        str(
            OUTPUT_DIR
            / "landing_selection.png"
        ),
        display
    )


# ============================================================
# MAIN
# ============================================================

def main():

    parser = argparse.ArgumentParser(
        description=(
            "GeoNav-AI V2 semantic segmentation "
            "and landing-zone selector"
        )
    )

    parser.add_argument(
        "image",
        help=(
            "Path to an OpenEarthMap-style "
            "RGB aerial image"
        )
    )

    args = parser.parse_args()

    image_path = Path(
        args.image
    ).expanduser()

    if not image_path.exists():

        raise FileNotFoundError(
            f"Image not found: {image_path}"
        )

    image = cv2.imread(
        str(image_path),
        cv2.IMREAD_COLOR
    )

    if image is None:

        raise RuntimeError(
            f"Could not read image: {image_path}"
        )

    print(
        "\nRunning V2 inference..."
    )

    (
        image_512,
        prediction,
        probabilities
    ) = predict(image)

    (
        landing_result,
        candidate_mask,
        safe_probability
    ) = select_landing_zone(
        prediction,
        probabilities
    )

    print(
        "\n=== LANDING-ZONE RESULT ==="
    )

    if landing_result is None:

        print("REJECTED")

        print(
            "No sufficiently safe "
            "landing zone was found."
        )

    else:

        print("ACCEPTED")

        print(
            "x:",
            landing_result["x"]
        )

        print(
            "y:",
            landing_result["y"]
        )

        print(
            "score:",
            landing_result["score"]
        )

        print(
            "safe_probability:",
            landing_result[
                "safe_probability"
            ]
        )

        print(
            "clearance_pixels:",
            landing_result[
                "clearance_pixels"
            ]
        )

        print(
            "predicted_class:",
            landing_result[
                "predicted_class"
            ]
        )

        print(
            "class_name:",
            landing_result[
                "class_name"
            ]
        )

    json_output = save_json_result(
        landing_result,
        image_path
    )

    save_visualizations(
        image_512,
        prediction,
        landing_result,
        candidate_mask
    )

    print(
        "\nLanding result JSON:",
        RESULT_JSON_PATH
    )

    print(
        "Results saved to:",
        OUTPUT_DIR
    )


if __name__ == "__main__":
    main()
