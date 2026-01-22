# lifecycle_py - ROS2 Python Lifecycle Nodes Package

I created this Python package to learn ROS2 lifecycle nodes, which provide a state machine-based approach to managing node states (unconfigured, inactive, active, etc.) for better system control and safety. This package contains lifecycle node implementations and a manager node demonstrating the ROS2 lifecycle pattern.

## Package Overview

This is a Python ROS2 package built with `ament_python` that contains lifecycle node examples and a lifecycle node manager for controlling lifecycle node states.

**Package Type:** Python (ament_python)  
**Build System:** setuptools  
**Dependencies:** `rclpy`, `lifecycle_msgs`, `example_interfaces`

---

## Package Contents

### `lifecycle_py/` - Main Python Package
Contains all Python source code for lifecycle node implementations:

#### Lifecycle Node Files
Python nodes demonstrating lifecycle node patterns:
- `number_publisher.py` - Regular (non-lifecycle) number publisher node for comparison
- `number_publisher_lifecycle.py` - Lifecycle version of the number publisher
- `lifecycle_node_manager.py` - Manager node that controls lifecycle node state transitions

**📖 Detailed explanations can be found in [`lifecycle_py/README.md`](lifecycle_py/README.md)**

### `setup.py` - Python Package Setup
Python setup configuration file that:
- Defines the package and its structure
- Registers executable entry points for each node
- Makes nodes available as ROS2 executables

**Executables defined:**
- `number_publisher` - From `lifecycle_py/number_publisher.py`
- `number_publisher_lifecycle` - From `lifecycle_py/number_publisher_lifecycle.py`
- `lifecycle_node_manager` - From `lifecycle_py/lifecycle_node_manager.py`

### `package.xml` - Package Metadata
Package manifest file defining:
- Package name, version, description
- Build tool dependencies (`ament_python`)
- Runtime dependencies (`rclpy`)
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
- ✅ Creating lifecycle nodes using `LifecycleNode` instead of `Node`
- ✅ Implementing lifecycle state callbacks (`on_configure`, `on_activate`, `on_deactivate`, `on_cleanup`, `on_shutdown`)
- ✅ Understanding the lifecycle state machine and state transitions
- ✅ Managing lifecycle nodes programmatically with a manager node
- ✅ Using lifecycle services (`ChangeState`) to control node states
- ✅ Differences between regular nodes and lifecycle nodes
- ✅ When to use lifecycle nodes (hardware control, system safety, resource management)
- ✅ Creating lifecycle publishers and timers
- ✅ Proper resource cleanup in lifecycle nodes
- ✅ Building Python ROS2 packages with setuptools

---

## Building and Running

**Build the package:**
```bash
cd /home/aklile/ros2_ws
colcon build --packages-select lifecycle_py
source install/setup.bash
```

**Run executables:**

**Option 1: Using the lifecycle node manager (recommended)**

In one terminal, start the lifecycle number publisher:
```bash
ros2 run lifecycle_py number_publisher_lifecycle
```

In another terminal, run the lifecycle node manager:
```bash
ros2 run lifecycle_py lifecycle_node_manager --ros-args -p managed_node_name:=number_publisher
```

The manager will automatically:
1. Configure the node (transition to inactive)
2. Wait 3 seconds
3. Activate the node (transition to active)

**Option 2: Manual state transitions**

Start the lifecycle node:
```bash
ros2 run lifecycle_py number_publisher_lifecycle
```

Then manually control it using ROS2 services:
```bash
# Configure (unconfigured → inactive)
ros2 service call /number_publisher/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1, label: configure}}"

# Activate (inactive → active)
ros2 service call /number_publisher/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3, label: activate}}"
```

**Option 3: Compare with regular node**

To see the difference, run the regular number publisher:
```bash
ros2 run lifecycle_py number_publisher
```

This node starts publishing immediately without any state management.

---

## Package Structure

```
lifecycle_py/
├── setup.py                    # Python package setup
├── setup.cfg                    # Setup configuration
├── package.xml                  # Package metadata
├── resource/                    # Resource files
│   └── lifecycle_py
├── test/                        # Test files
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
└── lifecycle_py/               # Main Python package
    ├── __init__.py
    ├── README.md               # Detailed explanations
    ├── number_publisher.py     # Regular node (for comparison)
    ├── number_publisher_lifecycle.py  # Lifecycle node
    └── lifecycle_node_manager.py      # Lifecycle manager
```

---

## What the Lifecycle Nodes Do

### Lifecycle State Machine

Lifecycle nodes go through a structured state machine:
1. **Unconfigured** - Initial state, no resources allocated
2. **Inactive** - Configured but not active (resources created but not publishing)
3. **Active** - Fully operational (publishing data)
4. **Shutdown** - Final state, node is shutting down

### State Transitions

- `configure`: Unconfigured → Inactive
  - Creates publishers, timers, and other resources
  - Implemented in `on_configure()` callback
  
- `activate`: Inactive → Active
  - Enables publishing and activates the node
  - Implemented in `on_activate()` callback
  
- `deactivate`: Active → Inactive
  - Disables publishing but keeps resources
  - Implemented in `on_deactivate()` callback
  
- `cleanup`: Inactive → Unconfigured
  - Cleans up resources (publishers, timers)
  - Implemented in `on_cleanup()` callback
  
- `shutdown`: Any state → Shutdown
  - Final cleanup and shutdown
  - Implemented in `on_shutdown()` callback

### Node Descriptions

1. **`number_publisher_lifecycle`** - A lifecycle node that:
   - Starts in "Unconfigured" state
   - Creates publisher and timer when configured (Inactive state)
   - Only publishes numbers when in "Active" state
   - Properly cleans up resources when deactivated or shut down

2. **`lifecycle_node_manager`** - A manager node that:
   - Takes a `managed_node_name` parameter
   - Automatically transitions lifecycle nodes through states
   - Uses the `ChangeState` service to control node states
   - Demonstrates programmatic lifecycle node management

3. **`number_publisher`** - A regular node (for comparison) that:
   - Starts publishing immediately
   - No state management
   - Demonstrates the difference between regular and lifecycle nodes

This demonstrates how to:
- Use ROS2 lifecycle nodes for better system control
- Implement state callbacks for proper resource management
- Manage lifecycle nodes programmatically
- Handle state transitions safely
- Clean up resources properly

---

## Notes

- Lifecycle nodes are especially useful for hardware control, where you want explicit control over when hardware is initialized and activated
- The lifecycle node manager demonstrates how to programmatically control lifecycle nodes
- `number_publisher_lifecycle.py` is a lifecycle version of `number_publisher.py` - they do the same thing, but the lifecycle version provides state management
- Lifecycle nodes must be explicitly activated before they start publishing
- The state machine ensures proper resource cleanup and prevents nodes from operating in invalid states
- Lifecycle nodes use `create_lifecycle_publisher()` instead of `create_publisher()`
- State callbacks return `TransitionCallbackReturn.SUCCESS` or `TransitionCallbackReturn.FAILURE`
- For detailed explanations of lifecycle node implementation, see [`lifecycle_py/README.md`](lifecycle_py/README.md)

