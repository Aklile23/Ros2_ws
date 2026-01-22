# my_robot_bringup - ROS2 Launch Configuration Package

I created this package to learn how to use ROS2 launch files to start multiple nodes together with proper configuration. This is a "bringup" package, which is a common ROS2 pattern for organizing launch files and configuration files in one place.

## Package Overview

This is a ROS2 bringup package that contains launch files and configuration files for starting my robot system. Bringup packages are typically used to launch entire systems or subsystems with all their nodes and configurations.

**Package Type:** C++ (ament_cmake)  
**Build System:** CMake  
**Purpose:** Launch files and configuration management

---

## Package Contents

### `launch/` - Launch Files
Contains launch files in both Python and XML formats:
- **`number_app.launch.py`** - Python launch file that starts multiple nodes with parameters
- **`number_app.launch.xml`** - XML version of the same launch file
- **`lifecycle_test.launch.py`** - Python launch file for lifecycle nodes and their manager
- **`lifecycle_test.launch.xml`** - XML version of the lifecycle launch file
- **`README.md`** - Detailed explanations of launch files

**📖 Detailed explanations of launch files can be found in [`launch/README.md`](launch/README.md)**

### `config/` - Configuration Files
Contains YAML configuration files for node parameters:
- **`number_app.yaml`** - Parameter configuration for the number publisher node
  - Sets `number: 10.0` and `timer_period: 0.5` parameters
  - Used by launch files to configure nodes

### `CMakeLists.txt` - Build Configuration
CMake configuration file that:
- Installs the `launch/` and `config/` directories to the package share directory
- Makes launch files and config files available after installation
- No executables are built (this package only contains launch/config files)

### `package.xml` - Package Metadata
Package manifest file defining:
- Package name, version, description
- Build tool dependencies (`ament_cmake`)
- Runtime dependencies:
  - `my_robot_controller` - the package whose nodes are launched
  - `lifecycle_py` - the package containing lifecycle nodes

---

## What I Learned

This package helped me learn:
- ✅ Creating launch files in both Python and XML formats
- ✅ Loading parameter configuration files (YAML) via launch files
- ✅ Starting multiple nodes simultaneously
- ✅ Organizing launch files in a dedicated bringup package
- ✅ ROS2 package structure for launch and configuration files
- ✅ Differences between Python and XML launch file syntax
- ✅ Launching lifecycle nodes and their managers
- ✅ Passing parameters directly to nodes (dictionary in Python, `<param>` in XML)
- ✅ Using variables in launch files (`<let>` in XML, Python variables)

---

## Building and Running

**Build the package:**
```bash
cd /home/aklile/ros2_ws
colcon build --packages-select my_robot_bringup
source install/setup.bash
```

**Run launch files:**
```bash
# Number app launch files
ros2 launch my_robot_bringup number_app.launch.py
# OR
ros2 launch my_robot_bringup number_app.launch.xml

# Lifecycle test launch files
ros2 launch my_robot_bringup lifecycle_test.launch.py
# OR
ros2 launch my_robot_bringup lifecycle_test.launch.xml
```

---

## Package Structure

```
my_robot_bringup/
├── CMakeLists.txt          # Build configuration
├── package.xml             # Package metadata
├── launch/                 # Launch files
│   ├── README.md          # Detailed explanations
│   ├── number_app.launch.py
│   ├── number_app.launch.xml
│   ├── lifecycle_test.launch.py
│   └── lifecycle_test.launch.xml
├── config/                 # Configuration files
│   └── number_app.yaml
└── README.md              # This file
```

---

## What the Launch Files Do

### `number_app` Launch Files
The launch files start:
1. `test_node` from `my_robot_controller`
2. `number_publisher_W_Param` from `my_robot_controller` with parameters loaded from `config/number_app.yaml`

### `lifecycle_test` Launch Files
The launch files start:
1. `number_publisher_lifecycle` from `lifecycle_py` - A lifecycle node that publishes numbers
2. `lifecycle_node_manager` from `lifecycle_py` - A manager that automatically transitions the lifecycle node through states (Unconfigured → Inactive → Active)

This demonstrates how to:
- Launch multiple nodes together
- Load parameter configuration files
- Use both Python and XML launch file formats
- Launch lifecycle nodes and their managers
- Pass parameters directly to nodes
- Use variables in launch files

---

## Notes

- This is a "bringup" package - a common ROS2 pattern for organizing launch files
- Launch files can be in Python (`.launch.py`) or XML (`.launch.xml`) format
- Configuration files are typically in YAML format
- The package depends on:
  - `my_robot_controller` - launches nodes from this package
  - `lifecycle_py` - launches lifecycle nodes from this package
- For detailed explanations of launch files, see [`launch/README.md`](launch/README.md)
- Lifecycle nodes require state management - the manager node handles state transitions automatically

