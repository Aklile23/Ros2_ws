# my_cpp_pkg - ROS2 C++ Learning Package

I created this C++ package to learn ROS2 programming in C++. This package contains basic exercises and examples that helped me understand how to write ROS2 nodes, publishers, subscribers, and services using the C++ API.

## Package Overview

This is a C++ ROS2 package built with `ament_cmake` that demonstrates fundamental ROS2 concepts in C++.

**Package Type:** C++ (ament_cmake)  
**Build System:** CMake  
**Dependencies:** `rclcpp`, `example_interfaces`

---

## Package Contents

### `src/` - Source Files
Contains all the C++ source code files with basic ROS2 exercises:
- **`my_first_node.cpp`** - My first simple ROS2 node (procedural style)
- **`my_first_nodeOOP.cpp`** - Object-oriented version with timer
- **`publisher.cpp`** - Publisher node that publishes messages
- **`subscriber.cpp`** - Subscriber node that receives messages
- **`server.cpp`** - Service server that handles requests
- **`client.cpp`** - Service client that makes requests

**📖 Detailed explanations of each script can be found in [`src/README.md`](src/README.md)**

### `include/` - Header Files
Directory for header files (currently minimal, as most examples are self-contained).

### `CMakeLists.txt` - Build Configuration
CMake configuration file that:
- Defines the package and dependencies
- Compiles each C++ source file into an executable
- Installs executables to the package lib directory

**Executables created:**
- `hello_node` - From `my_first_node.cpp`
- `hello_node_oop` - From `my_first_nodeOOP.cpp`
- `publisher` - From `publisher.cpp`
- `subscriber` - From `subscriber.cpp`
- `server` - From `server.cpp`
- `client` - From `client.cpp`

### `package.xml` - Package Metadata
Package manifest file defining:
- Package name, version, description
- Build tool dependencies (`ament_cmake`)
- Runtime dependencies (`rclcpp`, `example_interfaces`)

---

## What I Learned

This package helped me learn:
- ✅ Basic ROS2 node creation in C++
- ✅ Object-oriented programming with ROS2 nodes
- ✅ Publisher/subscriber communication patterns
- ✅ Service client/server patterns
- ✅ C++ specific concepts (smart pointers, templates, std::bind, callbacks)
- ✅ Building C++ ROS2 packages with CMake
- ✅ Differences between C++ and Python ROS2 programming

---

## Building and Running

**Build the package:**
```bash
cd /home/aklile/ros2_ws
colcon build --packages-select my_cpp_pkg
source install/setup.bash
```

**Run executables:**
```bash
ros2 run my_cpp_pkg hello_node
ros2 run my_cpp_pkg publisher
ros2 run my_cpp_pkg subscriber
# etc.
```

---

## Package Structure

```
my_cpp_pkg/
├── CMakeLists.txt          # Build configuration
├── package.xml             # Package metadata
├── include/                # Header files directory
│   └── my_cpp_pkg/
├── src/                    # C++ source files
│   ├── README.md          # Detailed explanations
│   ├── my_first_node.cpp
│   ├── my_first_nodeOOP.cpp
│   ├── publisher.cpp
│   ├── subscriber.cpp
│   ├── server.cpp
│   └── client.cpp
└── README.md              # This file
```

---

## Notes

- This package uses the standard ROS2 C++ API (`rclcpp`)
- All examples use `example_interfaces` for messages and services
- Code follows object-oriented patterns (except the first basic example)
- For detailed explanations of each script, see [`src/README.md`](src/README.md)

