# ROS2 Workspace - Learning and Practice

I created this ROS2 workspace to learn ROS2 fundamentals and practice various concepts. I'm writing this README as a reference guide for myself to understand what I built when I come back to it in the future.

## Workspace Structure

```
src/
├── my_cpp_pkg/          # C++ package with basic ROS2 node examples
├── my_robot_interfaces/ # Custom message and service definitions
├── my_robot_controller/ # Python package with various controllers and demos
└── my_robot_bringup/    # Launch files and configuration for system bringup
```

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

---

### 2. `my_robot_interfaces` (Interface Definitions)

**Type:** Interface Package (ament_cmake)  
**What I did:** I created this package to define custom message and service types that I use across my workspace

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

**Dependencies:**
- `rosidl_default_generators` - For generating message/service code
- `rosidl_default_runtime` - Runtime support for interfaces

**How I use it:** I use this package in `my_robot_controller` to communicate using my custom message and service types.

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

**Configuration:**
- `config/number_app.yaml` - I created a YAML configuration file for node parameters

**Dependencies:**
- `my_robot_controller` - This launches nodes from my controller package

**How I use it:** I use this package to launch multiple nodes together with proper configuration.

---

## Building the Workspace

From the workspace root (`/home/aklile/ros2_ws`):

```bash
colcon build
```

To build a specific package:
```bash
colcon build --packages-select <package_name>
```

## Running Nodes

After building, source the workspace:
```bash
source install/setup.bash
```

Then run nodes individually:
```bash
ros2 run <package_name> <executable_name>
```

Or use launch files:
```bash
ros2 launch my_robot_bringup number_app.launch.py
```

## What I Learned

I created examples and practiced:
1. **Basic Nodes** - I learned how to create simple ROS2 nodes in both C++ and Python
2. **Topics** - I practiced publisher/subscriber communication patterns
3. **Services** - I practiced request/response service patterns
4. **Custom Interfaces** - I learned how to create and use custom messages and services
5. **Parameters** - I learned how to configure nodes using parameters
6. **Launch Files** - I learned how to bring up systems and launch multiple nodes together
7. **Object-Oriented Design** - I practiced OOP patterns in ROS2 nodes

---

## Notes for Future Me

- This is my learning workspace - the code contains examples and practice implementations I created
- Some package descriptions in `package.xml` files may still have TODO placeholders
- I tried to follow ROS2 best practices with proper package structure and dependencies

