# GeoNav-AI: Geospatial-Aware Autonomous UAV Navigation

GeoNav-AI is an autonomous UAV navigation project integrating **geospatial intelligence, computer vision, LiDAR perception, state estimation, path planning, and PX4 flight control** using ROS 2.

The system connects satellite and geospatial analysis with real-time robotic autonomy. Geospatial data is processed to identify suitable mission and landing regions, while the onboard autonomy stack performs perception, localization, path planning, and mission execution in simulation.

---

##  Project Overview

GeoNav-AI combines two major autonomy layers:

### Geospatial Intelligence

Satellite and aerial imagery are processed to understand the environment and identify suitable UAV mission and landing regions.

The pipeline includes:

- Sentinel-2 multispectral imagery
- OpenEarthMap semantic data
- NDVI vegetation analysis
- NDWI water analysis
- Land-cover analysis
- Semantic segmentation
- Landing-zone candidate detection
- Landing-zone selection
- Geospatial mission-goal generation

### Autonomous UAV Navigation

The selected geospatial goal is transferred to the ROS 2 autonomy stack, where the UAV performs perception, state estimation, planning, and flight control.

The navigation pipeline includes:

- LiDAR perception
- Occupancy-grid generation
- IMU/GNSS state estimation
- A* path planning
- Waypoint and mission generation
- PX4 offboard control
- Gazebo-based UAV simulation

---

##  System Architecture

```text
Satellite / Geospatial Data
          |
          v
Sentinel-2 + OpenEarthMap
          |
          v
Geospatial AI Processing
  ├── NDVI / NDWI
  ├── Land-Cover Analysis
  ├── Semantic Segmentation
  └── Landing-Zone Selection
          |
          v
Geospatial Mission Goal
          |
          v
ROS 2 Geospatial Bridge
          |
          v
+--------------------------------------+
|        UAV AUTONOMY STACK            |
|                                      |
| LiDAR -------> Occupancy Grid        |
|                     |                |
|                     v                |
|                A* Planner            |
|                     |                |
|                     v                |
|              Planned Mission         |
|                     |                |
|                     v                |
|             PX4 Offboard Control     |
|                     |                |
|                     v                |
|                PX4 SITL UAV          |
|                                      |
| IMU / GNSS ---> State Estimation     |
|                    (EKF)             |
+--------------------------------------+
          |
          v
     Gazebo Simulation
```

---

##  Geospatial AI Pipeline

The geospatial component processes remote-sensing imagery before converting the resulting environmental information into a mission goal for the autonomous UAV.

### Satellite Data Processing

The project includes processing of Sentinel-2 multispectral imagery using bands such as:

- B02 — Blue
- B03 — Green
- B04 — Red
- B08 — Near Infrared

These bands are used to generate remote-sensing products including:

- RGB imagery
- NDVI
- NDWI
- Land-cover representations

### OpenEarthMap

OpenEarthMap imagery and semantic labels are used for land-cover understanding and semantic analysis.

The repository includes utilities for:

- Dataset downloading
- Dataset validation
- Region inspection
- Patch generation
- Class-coverage checking
- Dataset visualization
- Semantic inference

### Landing-Zone Intelligence

The geospatial inference pipeline identifies candidate landing regions and produces a selected landing goal.

Example outputs are stored in:

```text
results/geospatial_v2/
├── semantic_prediction.png
├── landing_candidates.png
├── landing_selection.png
└── landing_zone_result.json
```

This provides the connection between **remote-sensing intelligence** and **physical autonomous navigation**.

---

##  ROS 2 Autonomous Navigation Stack

The autonomy system is divided into modular ROS 2 packages.

### `geonav_geospatial_bridge`

Connects the geospatial AI pipeline with the robotic navigation system.

Key components:

- `geospatial_ai_mission.py`
- `landing_goal_publisher.py`

The package converts geospatial landing information into mission goals that can be consumed by the UAV autonomy stack.

---

### `geonav_perception`

Handles onboard LiDAR perception.

Key components:

- `lidar_interface.py`
- `lidar_occupancy_grid.py`

The LiDAR pipeline converts sensor measurements into an occupancy representation that can be used for obstacle-aware navigation.

---

### `geonav_planning`

Implements autonomous path planning.

Key components:

- `astar_planner.py`
- `occupancy_grid_publisher.py`

The planner uses occupancy information and mission goals to generate navigation paths.

---

### `geonav_state_estimation`

Implements the state-estimation layer in C++.

Key component:

- `src/ekf_node.cpp`

The package provides an Extended Kalman Filter based state-estimation component for the autonomous navigation pipeline.

---

### `geonav_control`

Connects navigation and mission planning with PX4 flight control.

Key components:

- `offboard_control.py`
- `planned_mission.py`
- `waypoint_mission.py`

This package handles waypoint missions, planned-path execution, and PX4 offboard command generation.

---

### `geonav_telemetry`

Provides vehicle telemetry interfaces for monitoring the state of the UAV during autonomous operation.

---

##  Navigation Pipeline

```text
Geospatial Landing Goal
          |
          v
ROS 2 Mission Interface
          |
          +----------------------+
          |                      |
          v                      v
       LiDAR                 IMU / GNSS
          |                      |
          v                      v
   Occupancy Grid              EKF
          |                      |
          +----------+-----------+
                     |
                     v
                A* Planner
                     |
                     v
                Planned Path
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

##  PX4 + ROS 2 Integration

PX4 communicates with the ROS 2 autonomy stack through its DDS interface.

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

PX4 `/fmu/in/*` and `/fmu/out/*` topics provide the communication interface between ROS 2 autonomy nodes and the PX4 flight controller.

---

##  Technologies & Tools

### Robotics and Autonomous Systems

- ROS 2 Humble
- PX4 Autopilot
- PX4 SITL
- Gazebo Sim
- Micro XRCE-DDS
- `px4_msgs`
- ROS 2 publisher/subscriber architecture
- PX4 offboard control

### Perception and Navigation

- LiDAR
- Occupancy grids
- A* path planning
- Waypoint navigation
- Mission planning
- Obstacle representation

### State Estimation

- Extended Kalman Filter (EKF)
- IMU/GNSS state estimation
- Sensor fusion
- Coordinate transformations
- C++ ROS 2 development

### Geospatial AI and Remote Sensing

- Sentinel-2
- OpenEarthMap
- Multispectral imagery
- GeoTIFF processing
- NDVI
- NDWI
- Land-cover analysis
- Semantic segmentation
- Landing-zone inference
- Geospatial mission planning

### Programming and Development

- Python
- C++
- Ubuntu Linux
- Git
- GitHub
- CMake
- colcon
- ROS 2 package development

---

##  Project Structure

```text
geonav_ai/
├── control/
├── datasets/
├── docs/
├── geospatial/
│   ├── data/
│   ├── outputs/
│   └── src/
├── maps/
├── models/
├── perception/
├── planning/
├── results/
│   └── geospatial_v2/
├── ros2_ws/
│   └── src/
│       ├── geonav_control/
│       ├── geonav_geospatial_bridge/
│       ├── geonav_perception/
│       ├── geonav_planning/
│       ├── geonav_state_estimation/
│       └── geonav_telemetry/
├── launch_geonav.sh
└── README.md
```

Large datasets, virtual environments, trained model weights, ROS build artifacts, and other generated files are intentionally excluded from Git version control.

---

## Running GeoNav-AI

The project contains a launcher for the integrated system:

```bash
./launch_geonav.sh
```

Individual ROS 2 packages and nodes can also be launched independently for development and testing.

### Development Environment

```text
Ubuntu 22.04 LTS
ROS 2 Humble
PX4 Autopilot / PX4 SITL
Gazebo Sim
Python
C++
```

---

##  Current Outputs

The repository contains generated outputs from the geospatial pipeline, including:

- Sentinel RGB visualization
- NDVI map
- NDWI map
- Land-cover visualization
- OpenEarthMap dataset visualizations
- Semantic prediction
- Landing-zone candidates
- Selected landing region

These outputs demonstrate the geospatial perception and landing-zone selection stages of the system.

---

## What This Project Demonstrates

GeoNav-AI demonstrates an autonomous-system pipeline spanning:

**Geospatial AI → Perception → Sensor Fusion → State Estimation → Path Planning → Flight Control**

The project develops skills relevant to:

- Physical AI
- Robotics AI
- Autonomous Systems
- UAV Autonomy
- ADAS-style perception and planning
- Sensor Fusion
- State Estimation
- Autonomous Path Planning
- Geospatial AI
- Satellite Computer Vision
- Remote Sensing
- ROS 2
- PX4 flight-control integration
- Python and C++ robotics development

---
