#!/usr/bin/env python3

import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


# ============================================================
# CONFIGURATION
# ============================================================

RESULT_PATH = Path(
    "/home/padma/geonav_ai/results/"
    "geospatial_v2/landing_zone_result.json"
)

# Existing GeoNav A* occupancy-map limits
LOCAL_MIN = -10.0
LOCAL_MAX = 20.0


# ============================================================
# ROS2 NODE
# ============================================================

class LandingGoalPublisher(Node):

    def __init__(self):

        super().__init__(
            "geonav_landing_goal_publisher"
        )

        self.goal_publisher = self.create_publisher(
            PoseStamped,
            "/geonav/goal",
            10
        )

        self.timer = self.create_timer(
            1.0,
            self.publish_goal
        )

        self.goal_published = False

        self.get_logger().info(
            "GeoNav geospatial landing-goal bridge started"
        )

    # ========================================================
    # PIXEL -> LOCAL NORTH/EAST
    # ========================================================

    def pixel_to_local(
        self,
        u,
        v,
        width,
        height
    ):

        u_norm = u / (width - 1.0)
        v_norm = v / (height - 1.0)

        # Image horizontal axis:
        # left -> right
        #
        # Map to East:
        # -10 m -> +20 m
        east = (
            LOCAL_MIN
            + u_norm
            * (LOCAL_MAX - LOCAL_MIN)
        )

        # Image vertical axis points DOWN.
        #
        # North should increase UP,
        # therefore this axis is inverted.
        north = (
            LOCAL_MAX
            - v_norm
            * (LOCAL_MAX - LOCAL_MIN)
        )

        return north, east

    # ========================================================
    # PUBLISH GOAL
    # ========================================================

    def publish_goal(self):

        if self.goal_published:
            return

        # ----------------------------------------------------
        # Check result file
        # ----------------------------------------------------

        if not RESULT_PATH.exists():

            self.get_logger().error(
                f"Landing result does not exist: "
                f"{RESULT_PATH}"
            )

            return

        # ----------------------------------------------------
        # Load AI landing result
        # ----------------------------------------------------

        try:

            with open(
                RESULT_PATH,
                "r"
            ) as file:

                result = json.load(file)

        except Exception as error:

            self.get_logger().error(
                f"Could not read landing result: "
                f"{error}"
            )

            return

        # ----------------------------------------------------
        # Safety rejection
        # ----------------------------------------------------

        if not result.get(
            "accepted",
            False
        ):

            self.get_logger().warning(
                "Geospatial AI rejected the scene. "
                "No UAV goal will be published."
            )

            self.goal_published = True

            return

        # ----------------------------------------------------
        # Extract AI result
        # ----------------------------------------------------

        u = float(
            result["x"]
        )

        v = float(
            result["y"]
        )

        width = float(
            result.get(
                "image_width",
                512
            )
        )

        height = float(
            result.get(
                "image_height",
                512
            )
        )

        class_name = result.get(
            "class_name",
            "unknown"
        )

        safe_probability = float(
            result.get(
                "safe_probability",
                0.0
            )
        )

        score = float(
            result.get(
                "score",
                0.0
            )
        )

        clearance = float(
            result.get(
                "clearance_pixels",
                0.0
            )
        )

        # ----------------------------------------------------
        # Coordinate conversion
        # ----------------------------------------------------

        north, east = self.pixel_to_local(
            u,
            v,
            width,
            height
        )

        # ----------------------------------------------------
        # Additional map-bound protection
        # ----------------------------------------------------

        north = max(
            LOCAL_MIN,
            min(
                LOCAL_MAX,
                north
            )
        )

        east = max(
            LOCAL_MIN,
            min(
                LOCAL_MAX,
                east
            )
        )

        # ----------------------------------------------------
        # Create PoseStamped
        #
        # GeoNav convention:
        #
        # x = North
        # y = East
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

        # Goal altitude is handled by the
        # existing mission controller.
        goal.pose.position.z = 0.0

        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        # ----------------------------------------------------
        # Publish
        # ----------------------------------------------------

        self.goal_publisher.publish(
            goal
        )

        self.goal_published = True

        # ----------------------------------------------------
        # Logging
        # ----------------------------------------------------

        self.get_logger().info(
            "\n"
            "====================================\n"
            " GEO-NAV AI LANDING GOAL\n"
            "====================================\n"
            f"Source pixel       : "
            f"({u:.1f}, {v:.1f})\n"
            f"Semantic class     : "
            f"{class_name}\n"
            f"Safe probability   : "
            f"{safe_probability:.3f}\n"
            f"Selection score    : "
            f"{score:.3f}\n"
            f"Clearance          : "
            f"{clearance:.2f} px\n"
            "------------------------------------\n"
            f"Local North (x)    : "
            f"{north:.2f} m\n"
            f"Local East  (y)    : "
            f"{east:.2f} m\n"
            "------------------------------------\n"
            "Published topic     : "
            "/geonav/goal\n"
            "===================================="
        )


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = LandingGoalPublisher()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()
