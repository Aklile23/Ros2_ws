# Source Packages - Detailed Descriptions

This directory contains all the ROS2 packages I created for learning. Each package focuses on different aspects of ROS2 development.

## Package Overview

```
src/
├── my_cpp_pkg/          # C++ package with basic ROS2 node examples
├── my_robot_interfaces/ # Custom message, service, and action definitions
├── my_robot_controller/ # Python package with various controllers and demos
├── my_robot_bringup/    # Launch files and configuration for system bringup
├── my_robot_description/ # Robot description package with URDF files
├── actions_py/          # Python package for learning ROS2 actions
├── lifecycle_py/        # Python package for learning lifecycle nodes
├── simple_commander/    # Navigation examples using Nav2 Simple Commander
└── URDF/                # Robot description files (URDF)
```

**📖 Each package has its own README with detailed explanations. See individual package directories for more information.**

---

## Package Descriptions

### 1. `my_cpp_pkg` (C++ Package)

**Type:** C++ (ament_cmake)  
**What I did:** I created this package to practice writing ROS2 nodes in C++

**What I built:**
- `my_first_node.cpp` - My first simple "Hello ROS2" node
- `my_first_nodeOOP.cpp` - I rewrote it using object-oriented programming
- `publisher.cpp` - I created a publisher node to practice publishing messages
- `subscriber.cpp` - I created a subscriber node to practice receiving messages
- `client.cpp` - I created a service client to practice making service calls
- `server.cpp` - I created a service server to practice handling service requests

**Dependencies:**
- `rclcpp` - ROS2 C++ client library
- `example_interfaces` - Standard ROS2 example interfaces

**Build:** Uses `ament_cmake` build system

**📖 See [`my_cpp_pkg/README.md`](my_cpp_pkg/README.md) for detailed explanations**

---

### 2. `my_robot_interfaces` (Interface Definitions)

**Type:** Interface Package (ament_cmake)  
**What I did:** I created this package to define custom message, service, and action types that I use across my workspace

**What I defined:**

**Messages:**
- `HardwareStatus.msg` - I created a custom message for hardware status with:
  - `float64 temperature` - Temperature reading
  - `bool are_motors_ready` - Motor readiness status
  - `string debug_message` - Debug information

**Services:**
- `ComputeRectangleArea.srv` - I created a service to calculate rectangle area:
  - Request: `float64 length`, `float64 width`
  - Response: `float64 area`

**Actions:**
- `CountUntil.action` - I created a custom action for counting with feedback:
  - Goal: `int64 target_number`, `float64 period`
  - Result: `int64 reached_number`
  - Feedback: `int64 current_number`

**Dependencies:**
- `rosidl_default_generators` - For generating message/service/action code
- `rosidl_default_runtime` - Runtime support for interfaces

**How I use it:** I use this package in other packages (`my_robot_controller`, `actions_py`) to communicate using my custom message, service, and action types.

**📖 See [`my_robot_interfaces/README.md`](my_robot_interfaces/README.md) for detailed explanations**

---

### 3. `my_robot_controller` (Python Controller Package)

**Type:** Python (ament_python)  
**What I did:** I created this package to practice writing ROS2 nodes in Python and learn various ROS2 concepts

**How I organized it:**
```
my_robot_controller/
├── Topics/              # I grouped topic-based communication examples here
├── Services/            # I grouped service client/server examples here
├── ParameterDemo/       # I grouped ROS2 parameter examples here
└── MsgTester/           # I grouped custom message testing here
```

**What I built:**

#### Topics/
- `my_first_node.py` - My first Python node
- `draw_circle.py` - I created a node that draws circles using turtlesim
- `pose_subscriber.py` - I created a node that subscribes to pose messages
- `bundle.py` - I combined multiple functionalities into one node

#### Services/
- `add_two_ints_server.py` - I created a service server that adds two integers
- `add_two_ints_client_no_oop.py` - I created a service client (non-OOP style)

#### ParameterDemo/
- `number_publisher_W_Param.py` - I created a publisher node to practice using ROS2 parameters

#### MsgTester/
- `hw_status_publisher.py` - I created a node that publishes my custom `HardwareStatus` messages

**Dependencies:**
- `rclpy` - ROS2 Python client library
- `geometry_msgs` - Geometry message types
- `turtlesim` - Turtle simulation package
- `my_robot_interfaces` - Custom interfaces from this workspace
- `std_msgs` - Standard ROS2 message types

**📖 See [`my_robot_controller/README.md`](my_robot_controller/README.md) for detailed explanations**

---

### 4. `my_robot_bringup` (Launch Configuration)

**Type:** C++ (ament_cmake)  
**What I did:** I created this package to practice using launch files to start multiple nodes together

**What I created:**

**Launch Files:**
- `number_app.launch.py` - I created a Python launch file that:
  - Launches `test_node` from `my_robot_controller`
  - Launches `number_publisher_W_Param` with parameters from config file
- `number_app.launch.xml` - I also created an XML version of the launch file
- `lifecycle_test.launch.py` - Launch file for lifecycle node testing
- `lifecycle_test.launch.xml` - XML version of lifecycle launch file
- `display.launch.py` - Python launch file for visualizing the robot in RViz using `my_robot_description`
- `display.launch.xml` - XML version of the display launch file

**Configuration:**
- `config/number_app.yaml` - I created a YAML configuration file for node parameters

**Dependencies:**
- `my_robot_controller` - This launches nodes from my controller package
- `my_robot_description` - Used by display launch files for robot visualization

**How I use it:** I use this package to launch multiple nodes together with proper configuration, including robot visualization.

**📖 See [`my_robot_bringup/README.md`](my_robot_bringup/README.md) and [`my_robot_bringup/launch/README.md`](my_robot_bringup/launch/README.md) for detailed explanations**

---

### 5. `my_robot_description` (Robot Description Package)

**Type:** Description Package (ament_cmake)  
**What I did:** I created this package to learn how to define robot descriptions using URDF (Unified Robot Description Format) in ROS2

**What I created:**

**URDF Files:**
- `urdf/my_robot.urdf` - Main robot description file that defines:
  - Links: base_footprint, base_link, laser, left_wheel, right_wheel, caster_wheel
  - Joints: Fixed and continuous joints connecting the links
  - Visual properties: Colors and geometries for visualization
  - Robot structure: Mobile robot with base, wheels, laser sensor

**RViz Configuration:**
- `rviz/config.rviz` - Pre-configured RViz settings for displaying the robot model

**Key concepts learned:**
- Creating URDF files to describe robot structure
- Defining links (physical components) and joints (connections)
- Understanding joint types (fixed, continuous)
- Adding visual properties (colors, materials, geometries)
- Using robot_state_publisher to publish robot description
- Configuring RViz for robot visualization

**Dependencies:**
- `ament_cmake` - Build system
- `robot_state_publisher` - For publishing robot state (runtime)
- `rviz2` - For visualization (runtime)
- `joint_state_publisher_gui` - For interactive joint control (runtime)

**How I use it:** The robot description is used by launch files in `my_robot_bringup` to visualize the robot in RViz.

**📖 See [`my_robot_description/README.md`](my_robot_description/README.md) for detailed explanations**

---

### 6. `actions_py` (Actions Package)

**Type:** Python (ament_python)  
**What I did:** I created this package to learn about ROS2 actions, which are like services but with feedback and cancellation support

**What I built:**
- `count_until_server.py` - Action server that counts to a target number with periodic feedback
- `count_until_client.py` - Action client that sends goals and receives feedback

**Key concepts learned:**
- Actions vs services (actions support feedback and cancellation)
- Action goals, results, and feedback
- Long-running tasks with progress updates
- Action state management

**Dependencies:**
- `rclpy` - ROS2 Python client library
- `my_robot_interfaces` - Uses `CountUntil` action

**📖 See [`actions_py/README.md`](actions_py/README.md) for detailed explanations**

---

### 7. `lifecycle_py` (Lifecycle Nodes Package)

**Type:** Python (ament_python)  
**What I did:** I created this package to learn about ROS2 lifecycle nodes, which have managed states and transitions

**What I built:**
- `number_publisher_lifecycle.py` - Lifecycle node that publishes numbers with state management
- `number_publisher.py` - Regular node for comparison
- `lifecycle_node_manager.py` - Node that manages lifecycle node states

**Key concepts learned:**
- Lifecycle node states (unconfigured, inactive, active, etc.)
- State transitions and callbacks
- Managing lifecycle nodes
- Differences between regular and lifecycle nodes

**Dependencies:**
- `rclpy` - ROS2 Python client library
- `lifecycle_msgs` - Lifecycle message types
- `std_msgs` - Standard message types

**📖 See [`lifecycle_py/README.md`](lifecycle_py/README.md) for detailed explanations**

---

### 8. `simple_commander` (Navigation Package)

**Type:** Python scripts  
**What I did:** I created this directory to learn about robot navigation using Nav2 Simple Commander API

**What I built:**
- `nav2_test.py` - Basic navigation examples
- `nav2_test2.py` - Additional navigation examples

**Key concepts learned:**
- Using Nav2 Simple Commander API
- Sending navigation goals
- Robot path planning and execution
- Navigation stack integration

**Dependencies:**
- `nav2_simple_commander` - Nav2 Simple Commander API
- `rclpy` - ROS2 Python client library

**📖 See [`simple_commander/README.md`](simple_commander/README.md) for detailed explanations**

---

### 9. `URDF` (Robot Description)

**Type:** URDF files  
**What I did:** I created this directory to learn about URDF (Unified Robot Description Format) for describing robot models

**What I created:**
- `my_robot.urdf` - URDF file describing a robot model

**Key concepts learned:**
- Robot description format
- Links and joints
- Visual and collision properties
- Robot model definition

**📖 See [`URDF/README.md`](URDF/README.md) for detailed explanations**

---

## Package Dependencies

**Build Order:**
1. `my_robot_interfaces` - Must be built first (other packages depend on it)
2. `my_cpp_pkg` - Independent C++ package
3. `my_robot_controller` - Depends on `my_robot_interfaces`
4. `my_robot_description` - Independent description package
5. `actions_py` - Depends on `my_robot_interfaces`
6. `lifecycle_py` - Independent package
7. `my_robot_bringup` - Depends on `my_robot_controller`, `lifecycle_py`, and `my_robot_description`
8. `simple_commander` - Independent scripts
9. `URDF` - Independent files

---

## Learning Progression

I created these packages in a learning progression:

1. **Fundamentals** (`my_cpp_pkg`, `my_robot_controller`)
   - Basic nodes, topics, services
   - Both C++ and Python

2. **Custom Interfaces** (`my_robot_interfaces`)
   - Custom messages, services, actions

3. **System Integration** (`my_robot_bringup`)
   - Launch files, configuration

4. **Robot Description** (`my_robot_description`)
   - URDF files, robot visualization

5. **Advanced Features** (`actions_py`, `lifecycle_py`)
   - Actions, lifecycle nodes

6. **Specialized Topics** (`simple_commander`, `URDF`)
   - Navigation, robot description

---

## Notes

- Each package has its own README with detailed explanations
- Package-specific READMEs are located in each package directory
- Some packages have subdirectory READMEs for specific topics (e.g., `Topics/README.md`)
- Build order matters - always build interface packages first
- For detailed information, see the README in each package directory

