#!/usr/bin/env python3

import json
import subprocess
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped


# ============================================================
# GEONAV-AI CONFIGURATION
# ============================================================

GEONAV_HOME = Path.home() / "geonav_ai"

GEOSPATIAL_DIR = (
    GEONAV_HOME
    / "geospatial"
)

VENV_PYTHON = (
    GEOSPATIAL_DIR
    / ".venv"
    / "bin"
    / "python3"
)

INFERENCE_SCRIPT = (
    GEOSPATIAL_DIR
    / "v2_landing_inference.py"
)

DEFAULT_IMAGE = (
    Path.home()
    / "Downloads"
    / "kagera_1.tif"
)

RESULT_JSON = (
    GEONAV_HOME
    / "results"
    / "geospatial_v2"
    / "landing_zone_result.json"
)

SEMANTIC_IMAGE = (
    GEONAV_HOME
    / "results"
    / "geospatial_v2"
    / "semantic_prediction.png"
)

LANDING_IMAGE = (
    GEONAV_HOME
    / "results"
    / "geospatial_v2"
    / "landing_selection.png"
)


# ============================================================
# SIMULATION ADAPTER
#
# IMPORTANT:
# This is NOT real-world CRS/georeferencing.
#
# It maps the 512x512 AI image into the existing
# GeoNav local A* simulation map.
# ============================================================

LOCAL_MIN = -10.0
LOCAL_MAX = 20.0

GOAL_ALTITUDE = 0.0


class GeoNavGeospatialAI(Node):

    def __init__(self):

        super().__init__(
            "geonav_geospatial_ai"
        )

        # ----------------------------------------------------
        # ROS GOAL PUBLISHER
        # ----------------------------------------------------

        self.goal_publisher = (
            self.create_publisher(
                PoseStamped,
                "/geonav/goal",
                10
            )
        )

        self.get_logger().info(
            "============================================"
        )

        self.get_logger().info(
            "       GEONAV GEOSPATIAL AI"
        )

        self.get_logger().info(
            "============================================"
        )

        # ----------------------------------------------------
        # INPUT IMAGE
        # ----------------------------------------------------

        if len(sys.argv) >= 2:

            self.image_path = Path(
                sys.argv[1]
            ).expanduser()

        else:

            self.image_path = DEFAULT_IMAGE

        self.get_logger().info(
            f"Input aerial image: {self.image_path}"
        )

        # ----------------------------------------------------
        # DELAY PIPELINE SLIGHTLY FOR ROS DISCOVERY
        # ----------------------------------------------------

        self.pipeline_started = False

        self.timer = self.create_timer(
            2.0,
            self.run_ai_pipeline
        )


    # ========================================================
    # COMPLETE GEOSPATIAL AI PIPELINE
    # ========================================================

    def run_ai_pipeline(self):

        if self.pipeline_started:
            return

        self.pipeline_started = True

        self.timer.cancel()

        # ----------------------------------------------------
        # VALIDATE INPUT FILES
        # ----------------------------------------------------

        if not self.image_path.exists():

            self.get_logger().error(
                f"Input image not found: "
                f"{self.image_path}"
            )

            return

        if not VENV_PYTHON.exists():

            self.get_logger().error(
                f"GeoNav Python environment not found: "
                f"{VENV_PYTHON}"
            )

            return

        if not INFERENCE_SCRIPT.exists():

            self.get_logger().error(
                f"Inference script not found: "
                f"{INFERENCE_SCRIPT}"
            )

            return

        # ----------------------------------------------------
        # RUN TRAINED V2 SEGMENTATION MODEL
        # ----------------------------------------------------

        self.get_logger().info("")

        self.get_logger().info(
            "============================================"
        )

        self.get_logger().info(
            "      RUNNING TRAINED SEGMENTATION"
        )

        self.get_logger().info(
            "============================================"
        )

        self.get_logger().info(
            "Model: U-Net + ResNet34 encoder"
        )

        self.get_logger().info(
            f"Input: {self.image_path.name}"
        )

        self.get_logger().info(
            "Running semantic segmentation..."
        )

        command = [
            str(VENV_PYTHON),
            str(INFERENCE_SCRIPT),
            str(self.image_path),
        ]

        try:

            result = subprocess.run(
                command,
                cwd=str(GEOSPATIAL_DIR),
                capture_output=True,
                text=True,
                check=False
            )

        except Exception as exc:

            self.get_logger().error(
                f"Failed to start AI inference: {exc}"
            )

            return

        # ----------------------------------------------------
        # SHOW OUTPUT FROM THE ACTUAL TRAINED MODEL
        # ----------------------------------------------------

        if result.stdout:

            print()

            print(
                "============================================"
            )

            print(
                "          TRAINED AI OUTPUT"
            )

            print(
                "============================================"
            )

            print(
                result.stdout.strip()
            )

            print(
                "============================================"
            )

            print()

        # OpenCV may print harmless GeoTIFF metadata warnings.
        if result.stderr:

            print(
                "AI inference diagnostic output:"
            )

            print(
                result.stderr.strip()
            )

            print()

        if result.returncode != 0:

            self.get_logger().error(
                "V2 semantic segmentation inference failed."
            )

            return

        # ----------------------------------------------------
        # VERIFY THIS RUN PRODUCED OUTPUTS
        # ----------------------------------------------------

        if not RESULT_JSON.exists():

            self.get_logger().error(
                "Inference completed but "
                "landing_zone_result.json was not generated."
            )

            return

        # ----------------------------------------------------
        # VISUALIZE SEMANTIC SEGMENTATION
        # ----------------------------------------------------

        self.get_logger().info("")

        self.get_logger().info(
            "Opening semantic segmentation produced "
            "by this inference run..."
        )

        if SEMANTIC_IMAGE.exists():

            try:

                subprocess.Popen(
                    [
                        "xdg-open",
                        str(SEMANTIC_IMAGE)
                    ],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )

                self.get_logger().info(
                    f"Segmentation visualization: "
                    f"{SEMANTIC_IMAGE}"
                )

            except Exception as exc:

                self.get_logger().warning(
                    f"Could not open segmentation image: "
                    f"{exc}"
                )

        else:

            self.get_logger().warning(
                "semantic_prediction.png "
                "was not generated."
            )

        # ----------------------------------------------------
        # VISUALIZE LANDING-ZONE SELECTION
        # ----------------------------------------------------

        self.get_logger().info(
            "Opening AI-selected landing candidate..."
        )

        if LANDING_IMAGE.exists():

            try:

                subprocess.Popen(
                    [
                        "xdg-open",
                        str(LANDING_IMAGE)
                    ],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )

                self.get_logger().info(
                    f"Landing visualization: "
                    f"{LANDING_IMAGE}"
                )

            except Exception as exc:

                self.get_logger().warning(
                    f"Could not open landing image: "
                    f"{exc}"
                )

        else:

            self.get_logger().warning(
                "landing_selection.png "
                "was not generated."
            )

        # ----------------------------------------------------
        # READ RESULT FROM THIS INFERENCE RUN
        # ----------------------------------------------------

        try:

            with open(
                RESULT_JSON,
                "r"
            ) as file:

                ai_result = json.load(
                    file
                )

        except Exception as exc:

            self.get_logger().error(
                f"Could not read AI result: {exc}"
            )

            return

        # ----------------------------------------------------
        # AI REJECTION
        # ----------------------------------------------------

        if not ai_result.get(
            "accepted",
            False
        ):

            self.get_logger().warning("")

            self.get_logger().warning(
                "============================================"
            )

            self.get_logger().warning(
                "        LANDING CANDIDATE REJECTED"
            )

            self.get_logger().warning(
                "============================================"
            )

            self.get_logger().warning(
                "The trained segmentation + candidate "
                "selector did not find a suitable region."
            )

            self.get_logger().warning(
                "NO /geonav/goal WILL BE PUBLISHED."
            )

            self.get_logger().warning(
                "PX4 mission will therefore NOT start."
            )

            return

        # ----------------------------------------------------
        # EXTRACT AI CANDIDATE
        # ----------------------------------------------------

        width = int(
            ai_result[
                "image_width"
            ]
        )

        height = int(
            ai_result[
                "image_height"
            ]
        )

        x_pixel = float(
            ai_result[
                "x"
            ]
        )

        y_pixel = float(
            ai_result[
                "y"
            ]
        )

        semantic_class = (
            ai_result[
                "class_name"
            ]
        )

        safe_probability = float(
            ai_result[
                "safe_probability"
            ]
        )

        selection_score = float(
            ai_result[
                "score"
            ]
        )

        clearance = float(
            ai_result[
                "clearance_pixels"
            ]
        )

        # ----------------------------------------------------
        # PIXEL -> LOCAL SIMULATION COORDINATES
        #
        # Image:
        # u = horizontal pixel
        # v = vertical pixel
        #
        # Simulation:
        # x = North
        # y = East
        #
        # This is a normalized SIMULATION ADAPTER.
        # It is NOT real-world CRS conversion.
        # ----------------------------------------------------

        u_norm = (
            x_pixel
            / float(
                width - 1
            )
        )

        v_norm = (
            y_pixel
            / float(
                height - 1
            )
        )

        east = (
            LOCAL_MIN
            + u_norm
            * (
                LOCAL_MAX
                - LOCAL_MIN
            )
        )

        north = (
            LOCAL_MAX
            - v_norm
            * (
                LOCAL_MAX
                - LOCAL_MIN
            )
        )

        # ----------------------------------------------------
        # SHOW AI DECISION
        # ----------------------------------------------------

        self.get_logger().info("")

        self.get_logger().info(
            "============================================"
        )

        self.get_logger().info(
            "        AI CANDIDATE ACCEPTED"
        )

        self.get_logger().info(
            "============================================"
        )

        self.get_logger().info(
            f"Semantic class      : "
            f"{semantic_class}"
        )

        self.get_logger().info(
            f"Source pixel        : "
            f"({x_pixel:.0f}, {y_pixel:.0f})"
        )

        self.get_logger().info(
            f"Safe probability    : "
            f"{safe_probability:.3f}"
        )

        self.get_logger().info(
            f"Selection score     : "
            f"{selection_score:.3f}"
        )

        self.get_logger().info(
            f"Clearance           : "
            f"{clearance:.2f} px"
        )

        self.get_logger().info("")

        self.get_logger().info(
            "Simulation coordinate adapter:"
        )

        self.get_logger().info(
            f"Local North (x)     : "
            f"{north:.2f} m"
        )

        self.get_logger().info(
            f"Local East  (y)     : "
            f"{east:.2f} m"
        )

        # ----------------------------------------------------
        # CREATE ROS2 GOAL
        # ----------------------------------------------------

        goal = PoseStamped()

        goal.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        goal.header.frame_id = "map"

        goal.pose.position.x = north
        goal.pose.position.y = east
        goal.pose.position.z = GOAL_ALTITUDE

        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        self.goal = goal

        # ----------------------------------------------------
        # PUBLISH GOAL MULTIPLE TIMES
        #
        # /geonav/goal is VOLATILE.
        #
        # Publishing five times provides a robust handoff
        # to the already-running A* planner.
        # ----------------------------------------------------

        self.publish_count = 0

        self.publish_timer = (
            self.create_timer(
                0.5,
                self.publish_goal
            )
        )

        self.get_logger().info("")

        self.get_logger().info(
            "============================================"
        )

        self.get_logger().info(
            "        AI -> ROS2 MISSION HANDOFF"
        )

        self.get_logger().info(
            "============================================"
        )

        self.get_logger().info(
            "Publishing AI-selected goal to:"
        )

        self.get_logger().info(
            "/geonav/goal"
        )


    # ========================================================
    # ROBUST GOAL PUBLICATION
    # ========================================================

    def publish_goal(self):

        self.goal.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        self.goal_publisher.publish(
            self.goal
        )

        self.publish_count += 1

        self.get_logger().info(
            f"AI goal published "
            f"({self.publish_count}/5)"
        )

        if self.publish_count >= 5:

            self.publish_timer.cancel()

            self.get_logger().info("")

            self.get_logger().info(
                "============================================"
            )

            self.get_logger().info(
                " AI -> ROS2 AUTONOMY HANDOFF COMPLETE"
            )

            self.get_logger().info(
                "============================================"
            )

            self.get_logger().info(
                "Trained U-Net segmentation"
            )

            self.get_logger().info(
                "              |"
            )

            self.get_logger().info(
                "              v"
            )

            self.get_logger().info(
                "Semantic terrain probabilities"
            )

            self.get_logger().info(
                "              |"
            )

            self.get_logger().info(
                "              v"
            )

            self.get_logger().info(
                "Candidate landing-zone selection"
            )

            self.get_logger().info(
                "              |"
            )

            self.get_logger().info(
                "              v"
            )

            self.get_logger().info(
                "/geonav/goal"
            )

            self.get_logger().info(
                "              |"
            )

            self.get_logger().info(
                "              v"
            )

            self.get_logger().info(
                "C++ A* Planner"
            )

            self.get_logger().info(
                "              |"
            )

            self.get_logger().info(
                "              v"
            )

            self.get_logger().info(
                "Mission Control"
            )

            self.get_logger().info(
                "              |"
            )

            self.get_logger().info(
                "              v"
            )

            self.get_logger().info(
                "PX4"
            )


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = (
        GeoNavGeospatialAI()
    )

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":

    main()
