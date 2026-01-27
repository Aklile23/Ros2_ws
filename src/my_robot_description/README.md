# my_robot_description - Robot Description Package

I created this package to learn how to define robot descriptions using URDF (Unified Robot Description Format) in ROS2. This package contains the complete 3D model and physical description of my robot.

## Package Overview

This is a ROS2 description package that defines the physical structure, visual appearance, and kinematic properties of a mobile robot using URDF. Description packages are essential for visualization, simulation, and motion planning in ROS2.

**Package Type:** Description Package (ament_cmake)  
**Build System:** CMake  
**Purpose:** Define robot physical structure and visual appearance using URDF

---

## Package Contents

### `urdf/` - Robot Description Files
Contains URDF (Unified Robot Description Format) files that define the robot's structure:

#### `my_robot.urdf`
The main robot description file that defines:
- **Links**: Physical components of the robot (base, wheels, laser, caster wheel)
- **Joints**: Connections between links (fixed, continuous)
- **Visual properties**: Colors and geometries for visualization
- **Kinematic structure**: How components are connected and move

**Robot Structure:**
- `base_footprint` - Reference frame (no visual)
- `base_link` - Main body (green box: 0.6m × 0.4m × 0.2m)
- `laser` - LiDAR sensor (white cylinder on top)
- `left_wheel` - Left drive wheel (grey cylinder, continuous joint)
- `right_wheel` - Right drive wheel (grey cylinder, continuous joint)
- `caster_wheel` - Support caster wheel (grey sphere, fixed)

**Materials Defined:**
- `grey` - Used for wheels (rgba: 0.5 0.5 0.5 1)
- `green` - Used for base link (rgba: 0 0.6 0 1)
- `white` - Used for laser (rgba: 1 1 1 1)

### `rviz/` - Visualization Configuration
Contains RViz configuration files for visualizing the robot:

#### `config.rviz`
Pre-configured RViz settings for displaying the robot model. This file is automatically loaded when using the display launch file.

### `CMakeLists.txt` - Build Configuration
CMake configuration file that:
- Installs the `urdf/` directory to share location
- Installs the `rviz/` directory to share location
- Makes robot description files available to other packages

**Installed directories:**
- `share/my_robot_description/urdf/` - URDF files
- `share/my_robot_description/rviz/` - RViz config files

### `package.xml` - Package Metadata
Package manifest file defining:
- Package name, version, description
- Build tool dependencies (`ament_cmake`)
- Test dependencies (`ament_lint_auto`, `ament_lint_common`)

---

## What I Learned

This package helped me learn:
- ✅ Creating URDF files to describe robot structure
- ✅ Defining links (physical components) in URDF
- ✅ Defining joints (connections) between links
- ✅ Understanding joint types (fixed, continuous, revolute, prismatic)
- ✅ Adding visual properties (colors, materials, geometries)
- ✅ Understanding the robot kinematic tree structure
- ✅ Using `base_footprint` as a reference frame
- ✅ Configuring RViz for robot visualization
- ✅ Installing description files for use by other packages

---

## Building and Using

**Build the package:**
```bash
cd /home/aklile/ros2_ws
colcon build --packages-select my_robot_description
source install/setup.bash
```

**Visualize the robot:**
The easiest way to visualize the robot is using the launch files from `my_robot_bringup`:
```bash
ros2 launch my_robot_bringup display.launch.py
# OR
ros2 launch my_robot_bringup display.launch.xml
```

These launch files will:
1. Start `robot_state_publisher` to publish the robot description
2. Start `joint_state_publisher_gui` to manually control joint positions
3. Launch RViz2 with the pre-configured visualization

**📖 Detailed descriptions of the launch files can be found in [`my_robot_bringup/README.md`](../my_robot_bringup/README.md)**

**Manual visualization:**
```bash
# Terminal 1: Start robot state publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(xacro $(ros2 pkg prefix --share my_robot_description)/urdf/my_robot.urdf)"

# Terminal 2: Start joint state publisher GUI (optional, for testing joints)
ros2 run joint_state_publisher_gui joint_state_publisher_gui

# Terminal 3: Launch RViz
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix --share my_robot_description)/rviz/config.rviz
```

**View URDF file:**
```bash
cat $(ros2 pkg prefix --share my_robot_description)/urdf/my_robot.urdf
```

**Check URDF for errors:**
```bash
check_urdf $(ros2 pkg prefix --share my_robot_description)/urdf/my_robot.urdf
```

---

## Package Structure

```
my_robot_description/
├── CMakeLists.txt          # Build configuration
├── package.xml             # Package metadata
├── urdf/                    # URDF robot description files
│   └── my_robot.urdf       # Main robot description
└── rviz/                    # RViz configuration files
    └── config.rviz         # Pre-configured RViz settings
```

---

## Robot Description Details

### Link Hierarchy
```
base_footprint (reference frame)
    └── base_link (main body)
        ├── laser (sensor)
        ├── left_wheel (drive wheel)
        ├── right_wheel (drive wheel)
        └── caster_wheel (support wheel)
```

### Joint Types Used

1. **Fixed Joint** (`type="fixed"`)
   - `base_footprint_base_link_joint` - Connects base_footprint to base_link
   - `base_laser_joint` - Connects base_link to laser (sensor is fixed)
   - `base_caster_wheel_joint` - Connects base_link to caster wheel

2. **Continuous Joint** (`type="continuous"`)
   - `base_left_wheel_joint` - Allows unlimited rotation of left wheel
   - `base_right_wheel_joint` - Allows unlimited rotation of right wheel

### Geometry Specifications

- **Base Link**: Box (0.6m × 0.4m × 0.2m), positioned at z=0.1m
- **Laser**: Cylinder (radius: 0.1m, length: 0.05m), positioned at z=0.225m
- **Wheels**: Cylinders (radius: 0.1m, length: 0.05m), rotated 90° around x-axis
- **Caster Wheel**: Sphere (radius: 0.05m), positioned at front of base

### Joint Positions

- **Left Wheel**: x=-0.15m, y=0.225m (left side of base)
- **Right Wheel**: x=-0.15m, y=-0.225m (right side of base)
- **Laser**: z=0.225m (on top of base)
- **Caster Wheel**: x=0.20m, z=-0.025m (front of base, slightly below)

---

## URDF Concepts

### Links
Links represent physical components of the robot. Each link can have:
- **Visual**: How the link appears in visualization
- **Collision**: Geometry for collision detection (not defined in this simple model)
- **Inertial**: Mass and inertia properties (not defined in this simple model)

### Joints
Joints define how links are connected and how they can move:
- **Fixed**: No movement (e.g., sensor mounted on base)
- **Continuous**: Unlimited rotation (e.g., wheels)
- **Revolute**: Limited rotation (not used in this robot)
- **Prismatic**: Linear movement (not used in this robot)

### Materials
Materials define colors for visualization:
- Defined once and referenced by name
- Use RGBA format (Red, Green, Blue, Alpha)
- Alpha = 1.0 means fully opaque

---

## Dependencies

**This package provides:**
- Robot URDF description for visualization
- RViz configuration for easy visualization setup
- Robot description accessible to other packages (via `get_package_share_directory`)

**This package depends on:**
- `ament_cmake` - Build system
- `robot_state_publisher` - For publishing robot state (runtime dependency)
- `rviz2` - For visualization (runtime dependency)
- `joint_state_publisher_gui` - For interactive joint control (runtime dependency)

**Packages that use this:**
- `my_robot_bringup` - Uses this package's URDF and RViz config in launch files

---

## Usage in Launch Files

This package is typically used in launch files like this:

```python
from ament_index_python.packages import get_package_share_directory
import os

# Get URDF path
urdf_path = os.path.join(
    get_package_share_directory("my_robot_description"), 
    "urdf", 
    "my_robot.urdf"
)

# Use in robot_state_publisher
robot_description = ParameterValue(
    Command(["xacro ", urdf_path]), 
    value_type=str
)
```

See `my_robot_bringup/launch/display.launch.py` (Python) or `my_robot_bringup/launch/display.launch.xml` (XML) for complete examples. Detailed descriptions can be found in [`my_robot_bringup/README.md`](../my_robot_bringup/README.md).

---

## Notes

- URDF files are XML-based and human-readable
- `base_footprint` is a common convention for the robot's reference frame
- Continuous joints are used for wheels that can rotate indefinitely
- Visual properties are separate from collision properties (this model only has visual)
- For simulation, you would also need to add collision and inertial properties
- Xacro can be used to make URDF files more modular and parameterized
- The robot description is published on the `/robot_description` topic by `robot_state_publisher`

---

## Next Steps (Ideas for Future Learning)

- Add collision properties for each link (for simulation)
- Add inertial properties (mass, inertia) for physics simulation
- Use Xacro to make the URDF more modular and parameterized
- Add more sensors (cameras, IMU, etc.)
- Create a more detailed visual model with meshes
- Add joint limits and dynamics for continuous joints
- Create separate URDF files for different robot configurations
- Learn about SDF (Simulation Description Format) for Gazebo
- Add transmission elements for actuators
- Create a robot description with arms or manipulators

