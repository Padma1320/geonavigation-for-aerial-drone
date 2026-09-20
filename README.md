# GeoNav-AI: Geospatial Intelligence for Autonomous UAV Navigation

**ROS 2 · PX4 · Gazebo · Geospatial AI · LiDAR Perception · State Estimation · Path Planning · Autonomous Systems**

GeoNav-AI is an **in-development autonomous UAV navigation system** that connects geospatial intelligence with a ROS 2 / PX4 autonomy stack.

The project explores how information extracted from satellite and geospatial imagery can be converted into actionable navigation goals for an autonomous vehicle. A selected landing region is transferred into the robotic autonomy pipeline, where perception, state estimation, path planning, mission control, and PX4 flight execution are integrated in simulation.

> **Project Status:** In Development  
> **Validation Environment:** PX4 SITL + Gazebo Sim

---

## Demo

The latest portfolio demonstration is included in this repository:

### [`GeoNav_AI_Portfolio_Demo.mp4`](./GeoNav_AI_Portfolio_Demo.mp4)

The demo presents the integrated GeoNav-AI workflow using the current simulation and autonomy stack.

---

# System Overview

GeoNav-AI connects two major layers:

### 1. Geospatial Intelligence

Remote-sensing and semantic information are processed to identify suitable mission and landing regions.

### 2. Autonomous UAV Navigation

The selected geospatial goal is transferred into a ROS 2 autonomy stack responsible for perception, planning, mission execution, and PX4 flight control.

The resulting pipeline is:

```text
Satellite / Geospatial Data
            |
            v
Geospatial Intelligence
            |
            v
Semantic Terrain Understanding
            |
            v
Landing-Zone Selection
            |
            v
AI-Selected Mission Goal
            |
            v
ROS 2 Geospatial Bridge
            |
            v
/geonav/goal
            |
            v
Occupancy Representation
            |
            v
       A* Planner
            |
            v
/geonav/planned_path
            |
            v
Mission Controller
            |
            v
PX4 Offboard Commands
            |
            v
       PX4 SITL
            |
            v
      Gazebo UAV
```

This creates a connection between **remote-sensing intelligence and closed-loop autonomous vehicle execution**.

---

# Geospatial AI Pipeline

The geospatial component analyzes environmental imagery and produces candidate regions for UAV mission planning and landing.

The project includes work with:

- Sentinel-2 multispectral imagery
- OpenEarthMap imagery and semantic labels
- RGB remote-sensing visualization
- NDVI vegetation analysis
- NDWI water analysis
- land-cover analysis
- semantic segmentation
- landing-zone candidate generation
- landing-zone scoring and selection
- geospatial mission-goal generation

---

## Sentinel-2 Processing

The geospatial pipeline works with Sentinel-2 bands including:

```text
B02 — Blue
B03 — Green
B04 — Red
B08 — Near Infrared
```

These bands support generation and analysis of products such as:

- RGB imagery
- vegetation information
- water information
- land-cover representations

NDVI and NDWI provide additional environmental information that can support interpretation of candidate operating regions.

---

## OpenEarthMap & Semantic Terrain Understanding

OpenEarthMap data is used for semantic land-cover analysis.

The geospatial workflow includes utilities for:

- dataset acquisition
- dataset validation
- region inspection
- image-patch generation
- class-coverage analysis
- dataset visualization
- semantic inference
- landing-region evaluation

The objective is to move beyond purely geometric navigation by introducing **environmental context before the UAV mission begins**.

---

# Landing-Zone Intelligence

The landing-zone pipeline evaluates semantic predictions and identifies candidate regions suitable for generating a UAV mission goal.

Example outputs are stored under:

```text
results/geospatial_v2/
├── semantic_prediction.png
├── landing_candidates.png
├── landing_selection.png
└── landing_zone_result.json
```

The generated result contains information such as:

- whether the candidate was accepted
- selected image coordinates
- predicted semantic class
- safe-region probability
- selection score
- obstacle / boundary clearance

An example stored inference result identifies an accepted **bareland** landing candidate.

---

# Geospatial AI → ROS 2 Mission Handoff

A key part of GeoNav-AI is converting the result of geospatial inference into a goal that can be used by the robotic autonomy stack.

The geospatial bridge reads the selected landing region and converts its image-space coordinates into the local navigation frame.

Conceptually:

```text
Semantic Prediction
        |
        v
Landing Candidate
        |
        v
Pixel Coordinate (u, v)
        |
        v
Pixel → Local Coordinate Conversion
        |
        v
Local North / East Goal
        |
        v
ROS 2 PoseStamped
        |
        v
/geonav/goal
```

The implemented bridge maps the selected image location into a bounded local North/East operating region before publishing the resulting goal.

Relevant ROS 2 components include:

```text
geospatial_ai_mission.py
landing_goal_publisher.py
```

This provides the interface between **geospatial AI and physical-autonomy software**.

---

# ROS 2 Autonomous Navigation Stack

The autonomous navigation system is implemented as a set of modular ROS 2 packages.

```text
ros2_ws/src/
├── geonav_control/
├── geonav_geospatial_bridge/
├── geonav_perception/
├── geonav_planning/
├── geonav_state_estimation/
└── geonav_telemetry/
```

Each package handles a different part of the autonomy pipeline.

---

## LiDAR Perception

The project includes a LiDAR perception pipeline for representing nearby obstacles.

Key components:

```text
geonav_perception/
├── lidar_interface.py
└── lidar_occupancy_grid.py
```

The perception layer processes LiDAR information for use by the navigation system.

A Gazebo LiDAR test utility is also included:

```text
perception/test_gz_lidar.py
```

The perception pipeline supports obstacle-aware navigation by converting sensor information into a representation suitable for planning.

---

# Occupancy Representation

Obstacle information is represented through an occupancy grid used by the path planner.

Relevant components include:

```text
lidar_occupancy_grid.py
occupancy_grid_publisher.py
```

The occupancy representation provides the spatial interface between perception and planning.

Conceptually:

```text
LiDAR
  |
  v
Range Measurements
  |
  v
Obstacle Processing
  |
  v
Occupancy Representation
  |
  v
A* Path Planner
```

---

# A* Path Planning

GeoNav-AI includes an implemented A* planning node:

```text
geonav_planning/astar_planner.py
```

The planner consumes mission-goal and occupancy information and produces a navigation path.

```text
/geonav/goal
      +
Occupancy Grid
      |
      v
  A* Search
      |
      v
Collision-Aware Path
      |
      v
/geonav/planned_path
```

The resulting path is passed to the mission-control layer for execution by the UAV.

---

# State Estimation

The repository includes a C++ state-estimation component:

```text
geonav_state_estimation/src/ekf_node.cpp
```

The state-estimation layer is designed around an **Extended Kalman Filter (EKF)** architecture for combining vehicle-state information relevant to autonomous navigation.

This part of the project develops experience with:

- state estimation
- IMU / GNSS information
- sensor fusion
- coordinate handling
- C++ ROS 2 development

State estimation forms the connection between raw vehicle measurements and the navigation/control stack.

---

# PX4 + ROS 2 Integration

PX4 is connected to ROS 2 through the PX4 DDS communication architecture.

```text
ROS 2 Humble
      |
      v
   px4_msgs
      |
      v
Micro XRCE-DDS
      |
      v
    PX4 SITL
      |
      v
  Gazebo UAV
```

The system communicates through PX4 topics including:

```text
/fmu/in/*
/fmu/out/*
```

Vehicle state and odometry are received from PX4, while offboard-control commands and trajectory setpoints are transmitted from ROS 2 to the flight controller.

---

# PX4 Offboard Control

The repository contains ROS 2 nodes for PX4 offboard operation.

Key components include:

```text
geonav_control/
├── offboard_control.py
├── waypoint_mission.py
└── planned_mission.py
```

The controller publishes:

```text
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/in/vehicle_command
```

and receives vehicle-state information including:

```text
/fmu/out/vehicle_odometry
```

The mission controller handles the conversion of planned ROS 2 waypoints into PX4-compatible trajectory setpoints.

---

# Autonomous Mission Execution

`planned_mission.py` connects the planner to PX4 flight execution.

The node:

- receives the A* generated path
- converts path points into PX4 NED-compatible commands
- streams offboard-control messages
- generates trajectory setpoints
- requests vehicle arming
- enters PX4 offboard mode
- tracks waypoint progress
- advances through the planned mission
- requests landing after mission completion
- monitors vehicle landing state

This creates the execution chain:

```text
A* Planned Path
      |
      v
ROS 2 Mission Controller
      |
      v
Waypoint Tracking
      |
      v
PX4 Trajectory Setpoints
      |
      v
PX4 Flight Controller
      |
      v
Gazebo UAV Motion
```

---

# Mid-Flight Replanning

The mission controller also contains support for accepting updated planner paths during flight.

When a new valid path is received, the controller can replace the remaining mission trajectory and begin tracking the updated path.

```text
Current Mission
      |
      v
Environment / Goal Update
      |
      v
New A* Path
      |
      v
Replan Accepted
      |
      v
Updated Waypoints
      |
      v
PX4 Execution Continues
```

Replanning is guarded during the landing phase to prevent a new navigation path from replacing a mission once landing has already been requested.

This provides a foundation for more adaptive autonomous navigation as the perception and planning layers continue to develop.

---

# Integrated Mission Flow

The current architecture connects the major subsystems as:

```text
        GEOSPATIAL INTELLIGENCE
                 |
                 v
      Semantic Terrain Analysis
                 |
                 v
        Landing-Zone Selection
                 |
                 v
         Local Mission Goal
                 |
                 v
             ROS 2
                 |
                 v
        /geonav/goal
                 |
        +--------+---------+
        |                  |
        v                  v
   LiDAR / Map       State Estimation
        |                  |
        v                  |
 Occupancy Grid             |
        |                  |
        +--------+---------+
                 |
                 v
            A* Planner
                 |
                 v
      /geonav/planned_path
                 |
                 v
        Mission Controller
                 |
                 v
        PX4 Offboard Control
                 |
                 v
             PX4 SITL
                 |
                 v
           Gazebo UAV
```

---

# System Launcher

The repository contains:

```text
launch_geonav.sh
```

for starting the integrated simulation environment.

The launcher coordinates:

1. PX4 SITL + Gazebo
2. QGroundControl
3. Micro XRCE-DDS Agent
4. occupancy-grid node
5. A* planner
6. planned-mission controller
7. system monitoring

The current launcher intentionally **does not automatically trigger the AI-selected mission**.

Instead, the system monitor checks the autonomy stack and PX4 state before the landing goal is manually released to the navigation pipeline.

The geospatial mission can then be initiated through:

```bash
ros2 run geonav_geospatial_bridge landing_goal_publisher
```

This separation provides a useful validation step before autonomous mission execution.

---

# Running GeoNav-AI

The integrated launcher can be started using:

```bash
./launch_geonav.sh
```

The primary development environment is:

```text
Ubuntu 22.04 LTS
ROS 2 Humble
PX4 Autopilot
PX4 SITL
Gazebo Sim
Micro XRCE-DDS
px4_msgs
Python
C++
```

Individual ROS 2 nodes can also be launched separately for subsystem development and testing.

---

# Repository Structure

```text
geonavigation-for-aerial-drone/
│
├── geospatial/
│   ├── data/
│   ├── outputs/
│   ├── src/
│   ├── inspect_v2.py
│   └── v2_landing_inference.py
│
├── perception/
│   └── test_gz_lidar.py
│
├── results/
│   └── geospatial_v2/
│       ├── semantic_prediction.png
│       ├── landing_candidates.png
│       ├── landing_selection.png
│       └── landing_zone_result.json
│
├── ros2_ws/
│   └── src/
│       ├── geonav_control/
│       ├── geonav_geospatial_bridge/
│       ├── geonav_perception/
│       ├── geonav_planning/
│       ├── geonav_state_estimation/
│       └── geonav_telemetry/
│
├── GeoNav_AI_Portfolio_Demo.mp4
├── launch_geonav.sh
├── .gitignore
└── README.md
```

Large datasets, ROS build artifacts, virtual environments, external PX4 source trees, and other generated files are intentionally excluded from Git version control.

---

# Technologies & Engineering Skills

## Robotics & Autonomous Systems

- ROS 2 Humble
- PX4 Autopilot
- PX4 SITL
- Gazebo Sim
- QGroundControl
- Micro XRCE-DDS
- `px4_msgs`
- ROS 2 publisher / subscriber architecture
- PX4 offboard control
- autonomous mission execution
- waypoint navigation
- mid-flight replanning

## Perception & Navigation

- LiDAR
- occupancy grids
- obstacle representation
- A* path planning
- waypoint planning
- autonomous path execution
- mission planning

## State Estimation

- Extended Kalman Filter
- IMU / GNSS state information
- sensor fusion
- coordinate transformations
- C++ ROS 2 development

## Geospatial AI & Remote Sensing

- Sentinel-2
- OpenEarthMap
- multispectral imagery
- GeoTIFF processing
- NDVI
- NDWI
- land-cover analysis
- semantic segmentation
- landing-zone inference
- geospatial mission planning

## Programming & Development

- Python
- C++
- NumPy
- ROS 2
- Linux / Ubuntu
- Git / GitHub
- CMake
- colcon
- Bash
- ROS 2 CLI

---

# Relevance to Physical AI & ADAS

Although GeoNav-AI uses a UAV as its current simulation platform, many of the engineering problems explored by the project are shared across autonomous robotic and intelligent-vehicle systems.

The project develops transferable experience in:

```text
Environment Perception
        ↓
State Estimation
        ↓
World Representation
        ↓
Path Planning
        ↓
Mission / Motion Decisions
        ↓
Closed-Loop Vehicle Execution
        ↓
Updated Sensor Information
```

These concepts are relevant to areas including:

- Physical AI
- autonomous robotics
- UAV autonomy
- autonomous vehicles
- ADAS perception and planning
- sensor fusion
- localization and state estimation
- motion and path planning
- intelligent vehicle control

GeoNav-AI is **not presented as an automotive ADAS implementation**. Instead, it demonstrates autonomy-stack concepts that transfer across aerial, ground, and other robotic vehicle platforms.

---

# What This Project Demonstrates

GeoNav-AI brings together several traditionally separate engineering areas:

**Satellite / Geospatial Intelligence**

↓

**Semantic Environmental Understanding**

↓

**Robotic Perception & State Estimation**

↓

**Autonomous Path Planning**

↓

**PX4 Flight Control**

↓

**Closed-Loop Vehicle Execution**

The project is intended to explore the interface between **AI-based environmental understanding and autonomous machines operating in the physical world**.

---

# Current Scope

GeoNav-AI is currently a **simulation-based development project**.

The repository demonstrates software integration and autonomous navigation using PX4 SITL and Gazebo.

It does not currently claim:

- deployment on a physical UAV
- production-ready flight autonomy
- safety-certified navigation
- automotive ADAS deployment
- fully validated real-world landing-zone safety
- complete real-world perception robustness

Future development can extend the system toward more complex perception, dynamic obstacle handling, improved planning, and hardware-based validation.

---

# Project Direction

GeoNav-AI is being developed as an exploration of:

**Geospatial AI + Robotics + Autonomous Systems + Physical AI**

The long-term engineering question behind the project is:

> How can large-scale environmental intelligence be converted into useful decisions for an autonomous machine operating locally in the physical world?

The current UAV implementation provides a simulation platform for investigating that connection.
