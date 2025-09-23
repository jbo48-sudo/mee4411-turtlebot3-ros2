# Setup
Before building this package, you need to install the following packages
```
sudo apt install libcgal-dev libarmadillo-dev liblapack-dev
```

# File Layout

This package contains the following files and directories:

## Top-Level Files

- [`README.md`](README.md): Setup instructions for building the package.
- [`CMakeLists.txt`](CMakeLists.txt): CMake build configuration for ROS 2.
- [`package.xml`](package.xml): ROS 2 package manifest with dependencies and metadata.

## Source Code

- [`src/laser_simulator.cc`](src/laser_simulator.cc): Main node implementation for the laser simulator.
- [`src/LaserSimulator.cc`](src/LaserSimulator.cc): Core logic for simulating laser scans.

## Headers

- [`include/laser_simulator/LaserSimulator.h`](include/laser_simulator/LaserSimulator.h): Class definition for the laser simulator.

## Messages

- [`msg/PoseStampedNamed.msg`](msg/PoseStampedNamed.msg): Custom message for a named pose.
- [`msg/PoseStampedNamedArray.msg`](msg/PoseStampedNamedArray.msg): Custom message for an array of named poses.
- [`msg/ScanPair.msg`](msg/ScanPair.msg): Custom message for scan pairs.

## Launch Files

- [`launch/laser_sim.xml`](launch/laser_sim.xml): Launch file for running the laser simulator node.
- [`launch/test_sim.xml`](launch/test_sim.xml): Launch file for testing the simulator with a robot and map.

## Models

- [`models/LDS_01.yaml`](models/LDS_01.yaml): Parameters for LDS_01 laser model.
- [`models/URG_04LX.yaml`](models/URG_04LX.yaml): Parameters for URG_04LX laser model.
- [`models/UTM_30.yaml`](models/UTM_30.yaml): Parameters for UTM_30 laser model.

## Configurations

- [`config/models.yaml`](config/models.yaml): List of robot types and models for simulation.

## Visualization

- [`rviz/test_laser_sim.rviz`](rviz/test_laser_sim.rviz): RViz configuration for visualizing laser scans and map.
