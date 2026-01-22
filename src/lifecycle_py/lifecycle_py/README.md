# Lifecycle Nodes - ROS2 Lifecycle Node Package

This package contains my ROS2 lifecycle node implementations. I created these scripts to learn and practice ROS2 lifecycle nodes, which provide a state machine-based approach to managing node states (unconfigured, inactive, active, etc.) for better system control and safety.

## Overview

Lifecycle nodes in ROS2 provide a structured way to manage node states through a state machine. Unlike regular nodes that start immediately, lifecycle nodes go through specific states:
- **Unconfigured** → **Inactive** → **Active** → **Inactive** → **Unconfigured** → **Shutdown**

This allows for better control over when nodes start publishing, connecting to hardware, and cleaning up resources.

## Files

### `number_publisher.py`
This is a regular (non-lifecycle) number publisher node that:
- Creates a publisher for `Int64` messages on the `number` topic
- Publishes incrementing numbers starting from 1
- Starts publishing immediately when the node is created
- Demonstrates the standard node behavior for comparison

### `number_publisher_lifecycle.py`
This is a **lifecycle version** of the number publisher script. It:
- Extends `LifecycleNode` instead of regular `Node`
- Implements lifecycle state callbacks:
  - `on_configure()` - Creates the publisher and timer when transitioning to inactive state
  - `on_activate()` - Activates the node, enabling publishing
  - `on_deactivate()` - Deactivates the node, disabling publishing
  - `on_cleanup()` - Cleans up resources (publisher, timer) when transitioning back to unconfigured
  - `on_shutdown()` - Handles shutdown state
- Only publishes numbers when in the **active** state
- Requires explicit state transitions via lifecycle services

### `lifecycle_node_manager.py`
This is a lifecycle node manager that:
- Manages lifecycle nodes by sending state transition requests
- Takes a `managed_node_name` parameter to specify which node to manage
- Provides an `initialization_sequence()` method that:
  1. Transitions the managed node from **unconfigured** to **inactive** (configure)
  2. Waits 3 seconds
  3. Transitions the managed node from **inactive** to **active** (activate)
- Uses the `ChangeState` service to control lifecycle node states
- Demonstrates how to programmatically control lifecycle nodes

## How to Use

### Prerequisites
Make sure you have:
1. Built the workspace
2. Sourced your ROS2 workspace setup file

### Running the Lifecycle Node

In one terminal, start the lifecycle number publisher:
```bash
ros2 run lifecycle_py number_publisher_lifecycle
```

You should see:
```
[INFO] [number_publisher]: IN CONSTRUCTOR
```

The node is now in the **unconfigured** state and won't publish anything yet.

### Running the Manager

In another terminal, run the lifecycle node manager:
```bash
ros2 run lifecycle_py lifecycle_node_manager --ros-args -p managed_node_name:=number_publisher
```

The manager will:
1. Configure the node (transition to inactive)
2. Wait 3 seconds
3. Activate the node (transition to active)

### Observing the Behavior

**Lifecycle node terminal** will show:
```
[INFO] [number_publisher]: IN CONSTRUCTOR
[INFO] [number_publisher]: IN on_configure
[INFO] [number_publisher]: IN on_activate
```

**Manager terminal** will show:
```
[INFO] [lifecycle_manager]: Trying to switch to configuring
[INFO] [lifecycle_manager]: Configuring OK, now inactive
[INFO] [lifecycle_manager]: Trying to switch to activating
[INFO] [lifecycle_manager]: Activating OK, now active
```

Once activated, you can check the published numbers:
```bash
ros2 topic echo /number
```

### Manual State Transitions

You can also manually control the lifecycle node using ROS2 services:
```bash
# Configure (unconfigured → inactive)
ros2 service call /number_publisher/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1, label: configure}}"

# Activate (inactive → active)
ros2 service call /number_publisher/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3, label: activate}}"

# Deactivate (active → inactive)
ros2 service call /number_publisher/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 4, label: deactivate}}"

# Cleanup (inactive → unconfigured)
ros2 service call /number_publisher/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 2, label: cleanup}}"
```

### Comparing Regular vs Lifecycle Node

To see the difference, run the regular number publisher:
```bash
ros2 run lifecycle_py number_publisher
```

This node starts publishing immediately without any state management, demonstrating the difference between regular and lifecycle nodes.

## Lifecycle States

The lifecycle node goes through these states:
1. **Unconfigured** - Initial state, no resources allocated
2. **Inactive** - Configured but not active (resources created but not publishing)
3. **Active** - Fully operational (publishing data)
4. **Shutdown** - Final state, node is shutting down

State transitions:
- `configure`: Unconfigured → Inactive
- `activate`: Inactive → Active
- `deactivate`: Active → Inactive
- `cleanup`: Inactive → Unconfigured
- `shutdown`: Any state → Shutdown

## What I Learned

This package helped me learn:
- ✅ Creating lifecycle nodes using `LifecycleNode` instead of `Node`
- ✅ Implementing lifecycle state callbacks (`on_configure`, `on_activate`, etc.)
- ✅ Understanding the lifecycle state machine
- ✅ Managing lifecycle nodes programmatically
- ✅ Using lifecycle services to control node states
- ✅ Differences between regular nodes and lifecycle nodes
- ✅ When to use lifecycle nodes (hardware control, system safety, resource management)

## Notes

- Lifecycle nodes are especially useful for hardware control, where you want explicit control over when hardware is initialized and activated
- The lifecycle node manager demonstrates how to programmatically control lifecycle nodes
- `number_publisher_lifecycle.py` is a lifecycle version of `number_publisher.py` - they do the same thing, but the lifecycle version provides state management
- Lifecycle nodes must be explicitly activated before they start publishing
- The state machine ensures proper resource cleanup and prevents nodes from operating in invalid states

