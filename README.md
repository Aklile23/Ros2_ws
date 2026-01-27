# ROS2 Workspace - Learning and Practice

I created this ROS2 workspace to learn ROS2 fundamentals and practice various concepts. I'm writing this README as a reference guide for myself to understand what I built when I come back to it in the future.

## Workspace Overview

This workspace contains multiple ROS2 packages organized for learning different aspects of ROS2:
- **C++ packages** - Learning ROS2 in C++
- **Python packages** - Learning ROS2 in Python
- **Interface packages** - Custom message, service, and action definitions
- **Bringup packages** - Launch files and configuration
- **Specialized packages** - Actions, lifecycle nodes, navigation, and URDF

For detailed information about each package, see [`src/README.md`](src/README.md).

---

## Workspace Structure

```
ros2_ws/
├── build/          # Build artifacts (generated)
├── install/        # Installed packages (generated)
├── log/            # Build logs (generated)
└── src/            # Source packages
    ├── my_cpp_pkg/
    ├── my_robot_interfaces/
    ├── my_robot_controller/
    ├── my_robot_bringup/
    ├── my_robot_description/
    ├── actions_py/
    ├── lifecycle_py/
    ├── simple_commander/
    └── URDF/
```

---

## Building the Workspace

From the workspace root (`/home/aklile/ros2_ws`):

**Build all packages:**
```bash
colcon build
```

**Build a specific package:**
```bash
colcon build --packages-select <package_name>
```

**Build with symlink install (faster for development):**
```bash
colcon build --symlink-install
```

---

## Running Nodes

After building, source the workspace:
```bash
source install/setup.bash
```

**Run nodes individually:**
```bash
ros2 run <package_name> <executable_name>
```

**Use launch files:**
```bash
ros2 launch <package_name> <launch_file>
```

---

## What I Learned

This workspace covers various ROS2 concepts:

1. **Basic Nodes** - Creating ROS2 nodes in both C++ and Python
2. **Topics** - Publisher/subscriber communication patterns
3. **Services** - Request/response service patterns
4. **Actions** - Long-running tasks with feedback and cancellation
5. **Custom Interfaces** - Creating custom messages, services, and actions
6. **Parameters** - Configuring nodes using parameters
7. **Launch Files** - System bringup and multi-node execution
8. **Lifecycle Nodes** - State-managed nodes with lifecycle transitions
9. **Robot Description** - Creating URDF files and visualizing robots
10. **Navigation** - Using Nav2 for robot navigation
11. **URDF** - Robot description files

---

## Package Organization

Packages are organized by learning topic:
- **Fundamentals** - Basic nodes, topics, services
- **Advanced Features** - Actions, lifecycle nodes
- **System Integration** - Launch files, configuration
- **Robot Description** - URDF files

Each package has its own README with detailed explanations. See [`src/README.md`](src/README.md) for package descriptions.

---

## Notes for Future Me

- This is my learning workspace - the code contains examples and practice implementations I created
- Some package descriptions in `package.xml` files may still have TODO placeholders
- I tried to follow ROS2 best practices with proper package structure and dependencies
- Build order matters: interface packages must be built before packages that use them
- Always source the workspace after building: `source install/setup.bash`
