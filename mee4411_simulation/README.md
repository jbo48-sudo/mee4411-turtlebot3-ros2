# Setup

This package requires the Python `collision` library. You can install this using:

```
pip3 install collision
```

If you get an error that `pip3` is not installed, you can install it using the command:

```
sudo apt install python3-pip
```

You also need to install the turtlebot3 libraries

```
sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-simulations ros-humble-tf2-msgs
```

# File Layout

This directory contains the following files and folders:

- [`LICENSE`](LICENSE): License information for the package.
- [`package.xml`](package.xml): ROS 2 package manifest with dependencies and metadata.
- [`pytest.ini`](pytest.ini): Pytest configuration file.
- [`setup.cfg`](setup.cfg): Configuration for Python packaging and linting.
- [`setup.py`](setup.py): Python setup script for installing the package.
- [`README.md`](README.md): Setup instructions and file layout overview.

## Launch Files

- [`launch/collision_detector.py`](launch/collision_detector.py): Launch script for collision detection node.
- [`launch/simulation.xml`](launch/simulation.xml): XML launch file for simulation.
- [`launch/tb3_simulation.py`](launch/tb3_simulation.py): Launch script for TurtleBot3 simulation node.
- [`launch/tb3_simulation.xml`](launch/tb3_simulation.xml): XML launch file for TurtleBot3 simulation.

## Python Package

This package has useful helper functions to make the simulation work properly. The `collision_detection` node will detect collisions between the TB3 and the map, shutting the simulation down when there is a collision. The `repub_simulated_sensors` node will republish simulated sensor data into a new ROS namespace.

- [`mee4411_simulation/__init__.py`](mee4411_simulation/__init__.py): Initializes the mee4411_simulation Python package.
- [`mee4411_simulation/collision_detection.py`](mee4411_simulation/collision_detection.py): Collision detection logic for the simulation.
- [`mee4411_simulation/repub_simulated_sensors.py`](mee4411_simulation/repub_simulated_sensors.py): Republishes simulated sensor data in a different namespace.

## Parameters

These files are copies of those from the [`turtlebo3_fake_node` package](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/9be186fb03d84ed4f293e5c0db71d8c05bbc91f3/turtlebot3_fake_node/param), which have been modified to work in different robot namespaces.

- [`param/burger.yaml`](param/burger.yaml): Parameters for TurtleBot3 Burger model.
- [`param/waffle.yaml`](param/waffle.yaml): Parameters for TurtleBot3 Waffle model.
- [`param/waffle_pi.yaml`](param/waffle_pi.yaml): Parameters for TurtleBot3 Waffle Pi model.

## Resources

- [`resource/mee4411_simulation`](resource/mee4411_simulation): Resource file for ROS 2 package index.

## RViz Configuration

- [`rviz/mee4411.rviz`](rviz/mee4411.rviz): RViz configuration for visualizing the simulation.

## TurtleBot3 Utilities Package

This package creates a class to store useful parameters for the TB3. This is a helper used inside of some other packages in the project.

- [`tb3_utils/__init__.py`](tb3_utils/__init__.py): Initializes the tb3_utils Python package.
- [`tb3_utils/tb3_params.py`](tb3_utils/tb3_params.py): Utility functions and parameters for TurtleBot3 models.

## Tests

- [`test/test_copyright.py`](test/test_copyright.py): Test for copyright compliance.
- [`test/test_flake8.py`](test/test_flake8.py): Test for PEP8 style compliance using flake8.
- [`test/test_pep257.py`](test/test_pep257.py): Test for docstring style compliance using pep257.
- [`test/test_transform2d_utils.py`](test/test_transform2d_utils.py): Unit tests for 2D transform utilities.

## Transform Utilities Package

This package has utilities to convert between different types of coordinate representations. You will be asked to fill these in (see below).

- [`transform2d_utils/__init__.py`](transform2d_utils/__init__.py): Initializes the transform2d_utils Python package.
- [`transform2d_utils/lookup_transform.py`](transform2d_utils/lookup_transform.py): Utility for looking up transforms.
- [`transform2d_utils/transform2d_utils.py`](transform2d_utils/transform2d_utils.py): 2D transformation utility functions.

# Instructions

Your task is to implement the code in the []`transform2d_utils`](transform2d_utils) package. This function has code to switch between three different data structures that can hold a transformation: a [`geometry_msgs/msg/Transform`](https://docs.ros.org/en/humble/p/geometry_msgs/msg/Transform.html), a list of $(x, y, \theta)$ values, and a 2D homogeneous transformation matrix, which has the form:
```math
H = \begin{bmatrix}
\cos(\theta) & -\sin(\theta) & x \\
\sin(\theta) & \cos(\theta) & y \\
0 & 0 & 1
\end{bmatrix}.
```

## Code

There are six functions to transform between each pair of these three representations.

1. [`transform2xyt`](transform2d_utils/transform2xyt): This converts from a `geometry_msgs/msg/Transform` to a list of $(x, y, \theta)$ values.
2. [`xyt2transform`](transform2d_utils/xyt2transform): This converts from a list of $(x, y, \theta)$ values to a `geometry_msgs/msg/Transform`.
3. [`homogeneous2xyt`](transform2d_utils/homogeneous2xyt):  This converts from a 2D homogeneous transformation matrix to a list of $(x, y, \theta)$ values.
4. [`xyt2homogeneous`](transform2d_utils/xyt2homogeneous): This converts from a list of $(x, y, \theta)$ values to a 2D homogeneous transformation matrix.
5. [`transform2homogeneous`](transform2d_utils/transform2homogeneous): This converts from a `geometry_msgs/msg/Transform` to a 2D homogeneous transformation matrix.
6. [`homogeneous2transform`](transform2d_utils/homogeneous2transform): This converts from a 2D homogeneous transformation matrix to a `geometry_msgs/msg/Transform`.

I already wrote the final two, so you need to implement the first four.

## Testing

You can test to make sure this code is working correctly using the command:

```
colcon test --packages-select mee4411_simulation
```

You can then view the results of the test with

```
colcon test-results --verbose
```

## Simulation Integration

The `transform2d_utils` are required for the `collision_detector` to work correctly (meaning the simulation will shut down when the robot drives into a wall). To test this, you can launch the simulation with

```
ros2 launch mee4411_simulation simulation.xml
```

Then in a second terminal run the following:

```
ros2 run turtlebot3_teleop teleop_keyboard
```

This will let you run drive the turtlebot with your keyboard. Make sure you have this window selected so that it registers your keyboard commands (similar to the [turtlesim in the ROS tutorials](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html#use-turtlesim)).
