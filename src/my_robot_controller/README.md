# my_robot_controller - ROS2 Python Controller Package

I created this Python package to learn ROS2 programming in Python and practice various ROS2 concepts. This package contains Python nodes organized by topic, demonstrating different ROS2 communication patterns and features.

## Package Overview

This is a Python ROS2 package built with `ament_python` that contains various controller nodes and examples organized by learning topic.

**Package Type:** Python (ament_python)  
**Build System:** setuptools  
**Dependencies:** `rclpy`, `geometry_msgs`, `turtlesim`, `my_robot_interfaces`, `std_msgs`

---

## Package Contents

### `my_robot_controller/` - Main Python Package
Contains all Python source code organized into subdirectories by learning topic:

#### `Topics/` - Topic Communication Examples
Python nodes demonstrating publisher/subscriber patterns:
- `my_first_node.py` - Basic first node
- `draw_circle.py` - Publisher that controls turtle movement
- `pose_subscriber.py` - Subscriber that reads turtle pose
- `bundle.py` - Combined publisher and subscriber

**📖 Detailed explanations can be found in [`my_robot_controller/Topics/README.md`](my_robot_controller/Topics/README.md)**

#### `Services/` - Service Communication Examples
Python nodes demonstrating service client/server patterns:
- `add_two_ints_server.py` - Service server that adds two integers
- `add_two_ints_client_no_oop.py` - Service client that calls the server

**📖 Detailed explanations can be found in [`my_robot_controller/Services/README.md`](my_robot_controller/Services/README.md)**

#### `ParameterDemo/` - Parameter Examples
Python nodes demonstrating ROS2 parameters:
- `number_publisher_W_Param.py` - Publisher node that uses parameters for configuration

**📖 Detailed explanations can be found in [`my_robot_controller/ParameterDemo/README.md`](my_robot_controller/ParameterDemo/README.md)**

#### `MsgTester/` - Custom Message Examples
Python nodes demonstrating custom message types:
- `hw_status_publisher.py` - Publisher that uses custom `HardwareStatus` message

**📖 Detailed explanations can be found in [`my_robot_controller/MsgTester/README.md`](my_robot_controller/MsgTester/README.md)**

### `setup.py` - Python Package Setup
Python setup configuration file that:
- Defines the package and its structure
- Registers executable entry points for each node
- Makes nodes available as ROS2 executables

**Executables defined:**
- `test_node` - From `Topics/my_first_node.py`
- `draw_circle` - From `Topics/draw_circle.py`
- `pose_subscriber` - From `Topics/pose_subscriber.py`
- `bundle` - From `Topics/bundle.py`
- `add_two_ints_server` - From `Services/add_two_ints_server.py`
- `add_two_ints_client` - From `Services/add_two_ints_client_no_oop.py`
- `hw_status_publisher` - From `MsgTester/hw_status_publisher.py`
- `number_publisher_W_Param` - From `ParameterDemo/number_publisher_W_Param.py`

### `package.xml` - Package Metadata
Package manifest file defining:
- Package name, version, description
- Build tool dependencies (`ament_python`)
- Runtime dependencies (`rclpy`, `geometry_msgs`, `turtlesim`, `my_robot_interfaces`, `std_msgs`)
- Test dependencies

### `test/` - Test Files
Contains test files for package validation:
- `test_copyright.py` - Copyright header tests
- `test_flake8.py` - Code style tests
- `test_pep257.py` - Documentation style tests

### `resource/` - Resource Files
Contains resource marker file for the package.

### `setup.cfg` - Setup Configuration
Configuration file for setuptools.

---

## What I Learned

This package helped me learn:
- ✅ Creating ROS2 nodes in Python
- ✅ Publisher/subscriber communication patterns
- ✅ Service client/server communication patterns
- ✅ Using ROS2 parameters for node configuration
- ✅ Creating and using custom message types
- ✅ Organizing Python packages in ROS2
- ✅ Building Python ROS2 packages with setuptools
- ✅ Working with turtlesim for visualization
- ✅ Using custom interfaces from other packages

---

## Building and Running

**Build the package:**
```bash
cd /home/aklile/ros2_ws
colcon build --packages-select my_robot_controller
source install/setup.bash
```

**Run executables:**
```bash
ros2 run my_robot_controller test_node
ros2 run my_robot_controller draw_circle
ros2 run my_robot_controller pose_subscriber
ros2 run my_robot_controller bundle
ros2 run my_robot_controller add_two_ints_server
ros2 run my_robot_controller add_two_ints_client
ros2 run my_robot_controller hw_status_publisher
ros2 run my_robot_controller number_publisher_W_Param
```

---

## Package Structure

```
my_robot_controller/
├── setup.py                    # Python package setup
├── setup.cfg                    # Setup configuration
├── package.xml                  # Package metadata
├── resource/                    # Resource files
│   └── my_robot_controller
├── test/                        # Test files
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
└── my_robot_controller/         # Main Python package
    ├── __init__.py
    ├── Topics/                  # Topic examples
    │   ├── README.md
    │   ├── my_first_node.py
    │   ├── draw_circle.py
    │   ├── pose_subscriber.py
    │   └── bundle.py
    ├── Services/                # Service examples
    │   ├── README.md
    │   ├── add_two_ints_server.py
    │   └── add_two_ints_client_no_oop.py
    ├── ParameterDemo/           # Parameter examples
    │   ├── README.md
    │   └── number_publisher_W_Param.py
    └── MsgTester/              # Custom message examples
        ├── README.md
        └── hw_status_publisher.py
```

---

## Notes

- This package uses the standard ROS2 Python API (`rclpy`)
- Nodes are organized by learning topic for easy navigation
- Each subdirectory has its own README with detailed explanations
- The package uses custom interfaces from `my_robot_interfaces`
- Some nodes work with `turtlesim` for visualization
- For detailed explanations of each topic, see the README files in each subdirectory

