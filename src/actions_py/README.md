# actions_py - ROS2 Python Actions Package

I created this Python package to learn ROS2 actions, which are useful for long-running tasks that can provide feedback and be canceled. This package contains action client and server implementations demonstrating the ROS2 action communication pattern.

## Package Overview

This is a Python ROS2 package built with `ament_python` that contains action client and server nodes for a counting action example.

**Package Type:** Python (ament_python)  
**Build System:** setuptools  
**Dependencies:** `rclpy`, `my_robot_interfaces`

---

## Package Contents

### `actions_py/` - Main Python Package
Contains all Python source code for action implementations:

#### Action Files
Python nodes demonstrating action client/server patterns:
- `count_until_server.py` - Action server that counts from 1 to a target number
- `count_until_client.py` - Action client that sends goals and receives feedback

**📖 Detailed explanations can be found in [`actions_py/README.md`](actions_py/README.md)**

### `setup.py` - Python Package Setup
Python setup configuration file that:
- Defines the package and its structure
- Registers executable entry points for each node
- Makes nodes available as ROS2 executables

**Executables defined:**
- `count_until_server` - From `actions_py/count_until_server.py`
- `count_until_client` - From `actions_py/count_until_client.py`

### `package.xml` - Package Metadata
Package manifest file defining:
- Package name, version, description
- Build tool dependencies (`ament_python`)
- Runtime dependencies (`rclpy`, `my_robot_interfaces`)
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
- ✅ Creating ROS2 action servers in Python
- ✅ Creating ROS2 action clients in Python
- ✅ Understanding the action communication pattern (Goal, Result, Feedback)
- ✅ Handling goal acceptance and execution
- ✅ Publishing feedback during action execution
- ✅ Handling goal cancellation gracefully
- ✅ Using async callbacks for action responses
- ✅ Working with custom action definitions from other packages
- ✅ Building Python ROS2 packages with setuptools

---

## Building and Running

**Build the package:**
```bash
cd /home/aklile/ros2_ws
colcon build --packages-select actions_py
source install/setup.bash
```

**Run executables:**

In one terminal, start the action server:
```bash
ros2 run actions_py count_until_server
```

In another terminal, run the action client:
```bash
ros2 run actions_py count_until_client
```

**Note:** The client sends a goal, receives feedback, and then cancels the goal after 3 seconds to demonstrate cancellation handling.

---

## Package Structure

```
actions_py/
├── setup.py                    # Python package setup
├── setup.cfg                    # Setup configuration
├── package.xml                  # Package metadata
├── resource/                    # Resource files
│   └── actions_py
├── test/                        # Test files
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
└── actions_py/                 # Main Python package
    ├── __init__.py
    ├── README.md               # Detailed explanations
    ├── count_until_server.py   # Action server
    └── count_until_client.py   # Action client
```

---

## What the Action Does

The `CountUntil` action demonstrates a simple counting task:

1. **Goal**: The client sends a goal with:
   - `target_number` (int64) - The number to count up to
   - `period` (float64) - The delay in seconds between each count

2. **Execution**: The server:
   - Counts from 1 up to the target number
   - Sleeps for the specified period between each count
   - Publishes feedback with the current number after each increment
   - Can be canceled at any time during execution

3. **Result**: When complete or canceled, returns:
   - `reached_number` (int64) - The number reached when the action completed or was canceled

4. **Feedback**: During execution, publishes:
   - `current_number` (int64) - The current count value

This demonstrates how to:
- Use ROS2 actions for long-running tasks
- Provide feedback during execution
- Handle goal cancellation gracefully
- Use custom action definitions from other packages

---

## Notes

- This package uses the standard ROS2 Python API (`rclpy`)
- The action definition (`CountUntil`) comes from the `my_robot_interfaces` package
- The server handles cancellation gracefully - if a goal is canceled, it stops counting and returns the number it reached
- Feedback is published after each count increment
- The action server can handle multiple goals sequentially (one at a time)
- The client uses async callbacks for handling goal responses, feedback, and results
- For detailed explanations of the action implementation, see [`actions_py/README.md`](actions_py/README.md)

