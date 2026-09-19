#!/usr/bin/env bash

# ============================================================
# GeoNav-AI Complete System Launcher
#
# Starts:
#   1. PX4 SITL + Gazebo
#   2. QGroundControl
#   3. Micro XRCE-DDS Agent
#   4. Occupancy Grid
#   5. A* Planner
#   6. Planned Mission Controller
#   7. System Monitor
#
# AI landing goal is intentionally NOT triggered automatically.
# ============================================================

GEONAV_HOME="$HOME/geonav_ai"
ROS_WS="$GEONAV_HOME/ros2_ws"
PX4_DIR="$GEONAV_HOME/PX4-Autopilot"

QGC="$HOME/Downloads/QGroundControl-x86_64.AppImage"


# ============================================================
# CHECKS
# ============================================================

if ! command -v gnome-terminal >/dev/null 2>&1; then
    echo "ERROR: gnome-terminal not found."
    exit 1
fi

if [ ! -f "$QGC" ]; then
    echo "ERROR: QGroundControl not found:"
    echo "$QGC"
    exit 1
fi

chmod +x "$QGC"


# ============================================================
# ROS ENVIRONMENT
# ============================================================

ROS_ENV="
unset ROS_LOCALHOST_ONLY;
unset CYCLONEDDS_URI;
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp;
source /opt/ros/humble/setup.bash;
source $ROS_WS/install/setup.bash;
"


echo
echo "=========================================="
echo "       GeoNav-AI Complete Launcher"
echo "=========================================="
echo


# ============================================================
# 1. PX4 + GAZEBO
# ============================================================

echo "[1/7] Starting PX4 + Gazebo..."

gnome-terminal \
    --title="GeoNav | PX4 + GAZEBO" \
    -- bash -c "
        cd $PX4_DIR;

        echo '================================';
        echo ' GEO-NAV: PX4 + GAZEBO';
        echo '================================';
        echo;

        make px4_sitl gz_x500_lidar_front;

        exec bash
    " &


echo "Waiting for PX4/Gazebo..."
sleep 8


# ============================================================
# 2. QGROUNDCONTROL
# ============================================================

echo "[2/7] Starting QGroundControl..."

"$QGC" >/tmp/geonav_qgc.log 2>&1 &

QGC_PID=$!

echo "QGroundControl PID: $QGC_PID"

sleep 6


# ============================================================
# 3. MICRO XRCE-DDS
# ============================================================

echo "[3/7] Starting Micro XRCE-DDS Agent..."

gnome-terminal \
    --title="GeoNav | XRCE DDS" \
    -- bash -c "
        $ROS_ENV

        echo '================================';
        echo ' GEO-NAV: MICRO XRCE-DDS';
        echo '================================';
        echo;

        MicroXRCEAgent udp4 -p 8888;

        exec bash
    " &


sleep 4


# ============================================================
# 4. OCCUPANCY GRID
# ============================================================

echo "[4/7] Starting occupancy grid..."

gnome-terminal \
    --title="GeoNav | OCCUPANCY GRID" \
    -- bash -c "
        $ROS_ENV

        echo '================================';
        echo ' GEO-NAV: OCCUPANCY GRID';
        echo '================================';
        echo;

        ros2 run geonav_planning \
            occupancy_grid_publisher;

        exec bash
    " &


sleep 2


# ============================================================
# 5. A* PLANNER
# ============================================================

echo "[5/7] Starting A* planner..."

gnome-terminal \
    --title="GeoNav | A-STAR PLANNER" \
    -- bash -c "
        $ROS_ENV

        echo '================================';
        echo ' GEO-NAV: A-STAR PLANNER';
        echo '================================';
        echo;

        ros2 run geonav_planning \
            astar_planner;

        exec bash
    " &


sleep 2


# ============================================================
# 6. PLANNED MISSION
# ============================================================

echo "[6/7] Starting planned mission..."

gnome-terminal \
    --title="GeoNav | MISSION CONTROL" \
    -- bash -c "
        $ROS_ENV

        echo '================================';
        echo ' GEO-NAV: MISSION CONTROL';
        echo '================================';
        echo;

        ros2 run geonav_control \
            planned_mission;

        exec bash
    " &


sleep 4


# ============================================================
# 7. SYSTEM MONITOR
# ============================================================

echo "[7/7] Starting system monitor..."

gnome-terminal \
    --title="GeoNav | SYSTEM MONITOR" \
    -- bash -c "
        $ROS_ENV

        echo;
        echo '========================================';
        echo '       GEONAV-AI SYSTEM MONITOR';
        echo '========================================';
        echo;

        echo 'Waiting for system initialization...';
        sleep 6;

        echo;
        echo '---------- GEONAV NODES ----------';
        ros2 node list | grep geonav || true;


        echo;
        echo '---------- PX4 ODOMETRY ----------';

        if timeout 6 ros2 topic echo \
            /fmu/out/vehicle_odometry \
            --once \
            >/tmp/geonav_odom.txt \
            2>/dev/null;
        then
            echo 'PX4 odometry: OK';
        else
            echo 'PX4 odometry: NOT DETECTED';
        fi


        echo;
        echo '---------- VEHICLE STATUS --------';

        if timeout 6 ros2 topic echo \
            /fmu/out/vehicle_status_v4 \
            --once \
            >/tmp/geonav_status.txt \
            2>/dev/null;
        then

            echo 'PX4 vehicle status: OK';

            echo;

            grep \
                -E \
                'arming_state:|nav_state:|gcs_connection_lost:|pre_flight_checks_pass:' \
                /tmp/geonav_status.txt \
                || true;

        else

            echo 'PX4 vehicle status: NOT DETECTED';

        fi


        echo;
        echo '---------- OCCUPANCY GRID --------';

        ros2 topic info \
            /geonav/occupancy_grid \
            2>/dev/null \
            || true;


        echo;
        echo '---------- A-STAR PATH -----------';

        ros2 topic info \
            /geonav/planned_path \
            2>/dev/null \
            || true;


        echo;
        echo '========================================';
        echo;
        echo 'CHECK BEFORE MISSION:';
        echo;
        echo '  PX4 odometry              -> OK';
        echo '  gcs_connection_lost       -> false';
        echo '  pre_flight_checks_pass    -> true';
        echo;
        echo 'Then inspect Gazebo + QGroundControl.';
        echo;
        echo 'DO NOT trigger mission if preflight';
        echo 'checks are false.';
        echo;
        echo 'When everything is ready, run:';
        echo;
        echo 'ros2 run geonav_geospatial_bridge landing_goal_publisher';
        echo;
        echo '========================================';

        exec bash
    " &


echo
echo "=========================================="
echo "       GeoNav-AI stack launched"
echo "=========================================="
echo
echo "Started:"
echo "  [1] PX4 + Gazebo"
echo "  [2] QGroundControl"
echo "  [3] Micro XRCE-DDS"
echo "  [4] Occupancy Grid"
echo "  [5] A* Planner"
echo "  [6] Mission Control"
echo "  [7] System Monitor"
echo
echo "AI mission has NOT been triggered."
echo
echo "Check SYSTEM MONITOR first."
echo "=========================================="
