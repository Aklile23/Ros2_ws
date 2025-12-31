# ParameterDemo Folder - ROS2 Parameters

I created this folder to learn how to use ROS2 parameters. This is where I learned to create and use parameters inside ROS2 nodes, allowing me to configure node behavior without changing the code.

## Overview

This folder contains a script that demonstrates how to:
- Declare parameters inside a ROS2 node
- Get parameter values at runtime
- Use parameters to configure node behavior
- Load parameters from YAML configuration files via launch files
- Make nodes configurable without code changes

---

## Script

### `number_publisher_W_Param.py`

**What I did:** I created a number publisher node that uses ROS2 parameters to configure what number it publishes and how often.

**What it does:**
- Creates a node named `number_publisher`
- Declares two parameters:
  - `number` - The number value to publish (default: 2.0)
  - `timer_period` - How often to publish in seconds (default: 0.5)
- Publishes the number value to the `/number` topic as a `Float64` message
- Uses the `timer_period` parameter to control the publishing frequency
- Can be configured via launch file with a YAML config file

**Key concepts I learned:**

1. **Declaring Parameters:**
   ```python
   self.declare_parameter("number", 2.0)
   self.declare_parameter("timer_period", 0.5)
   ```
   - `declare_parameter()` creates a parameter with a name and default value
   - If the parameter isn't provided externally, it uses the default value
   - Parameters must be declared before they can be used

2. **Getting Parameter Values:**
   ```python
   self.number_ = self.get_parameter("number").value
   self.timer_period = self.get_parameter("timer_period").value
   ```
   - `get_parameter()` retrieves the parameter value
   - Use `.value` to get the actual value from the parameter object
   - Can be called multiple times (useful for dynamic parameter updates)

3. **Using Parameters Dynamically:**
   ```python
   msg.data = self.get_parameter("number").value
   ```
   - I can get the parameter value at any time, not just in `__init__`
   - This allows the parameter to be changed at runtime (if using parameter callbacks)

4. **Parameter Configuration:**
   - Parameters can be set via command line, launch files, or YAML files
   - Default values in code provide fallback if no external config is provided
   - This makes nodes flexible and reusable

**How to run:**

**Method 1: Run with default parameters**
```bash
ros2 run my_robot_controller number_publisher_W_Param
```
- Uses default values: `number=2.0`, `timer_period=0.5`

**Method 2: Run with command-line parameters**
```bash
ros2 run my_robot_controller number_publisher_W_Param --ros-args -p number:=10.0 -p timer_period:=1.0
```
- Overrides default values via command line

**Method 3: Run with launch file (uses YAML config)**
```bash
ros2 launch my_robot_bringup number_app.launch.py
```
- Loads parameters from `my_robot_bringup/config/number_app.yaml`
- Sets `number=10.0` and `timer_period=0.5` from the config file

**What you'll see:**
- The node logs: "Number publisher has been started"
- The node continuously publishes the number value to the `/number` topic
- You can check the published values with: `ros2 topic echo /number`

---

## Parameter Configuration File

I also wrote a YAML configuration file to set parameters via launch file:

**Location:** `my_robot_bringup/config/number_app.yaml`

**Contents:**
```yaml
/number_publisher:
  ros__parameters:
    number: 10.0
    timer_period: 0.5
```

**How it works:**
- The node name (`/number_publisher`) matches the node name in the script
- `ros__parameters:` is the required namespace for ROS2 parameters
- Parameters are listed under this namespace
- This YAML file is imported and used in the launch file located at `my_robot_bringup/launch/`

**Benefits:**
- Separate configuration from code
- Easy to change values without recompiling
- Can have different config files for different scenarios
- Version control friendly (can track config changes separately)

---

## Launch File Integration

I integrated this node into a launch file to demonstrate parameter loading:

**Location:** `my_robot_bringup/launch/number_app.launch.py`

**Relevant section:**
```python
param_config = os.path.join(get_package_share_directory("my_robot_bringup"), "config", "number_app.yaml")

number_publisher_W_Param = Node(
    package="my_robot_controller",
    executable="number_publisher_W_Param",
    parameters=[param_config]
)
```

**What I learned:**
- Launch files can load YAML config files
- Use `get_package_share_directory()` to find package paths
- Pass the config file path to the `parameters` argument of the Node
- Multiple nodes can share the same config file or have separate ones

---

## Why Parameters Are Useful

I learned that parameters are great for:

1. **Configuration without code changes:**
   - Change behavior by editing YAML files
   - No need to recompile or modify source code

2. **Different scenarios:**
   - Development vs. production settings
   - Different robot configurations
   - Testing different values easily

3. **Runtime flexibility:**
   - Can change parameters while the node is running (with parameter callbacks)
   - Useful for tuning and debugging

4. **Best practices:**
   - Makes code more reusable
   - Separates configuration from logic
   - Easier to maintain and document

---

## Parameter vs. Hardcoded Values

**Before learning parameters (hardcoded):**
```python
self.number_ = 2.0  # Hardcoded - requires code change to modify
```

**After learning parameters:**
```python
self.declare_parameter("number", 2.0)  # Configurable - can be changed externally
self.number_ = self.get_parameter("number").value
```

**Benefits:**
- No code changes needed to adjust values
- Can test different configurations easily
- More professional and maintainable code
- Follows ROS2 best practices

---

## Dependencies

The script uses:
- `rclpy` - ROS2 Python client library
- `std_msgs.msg.Float64` - Standard message type for floating-point numbers

---

## Notes for Future Me

- Parameters must be declared before they can be used (in `__init__` or before first use)
- Default values in `declare_parameter()` are used if no external value is provided
- Parameter names in YAML must match exactly (case-sensitive)
- The node name in YAML (`/number_publisher`) must match the node name in code
- I can get parameter values multiple times - useful for checking if they changed
- For dynamic parameter updates, I would need to add a parameter callback (not implemented here)
- Command-line parameters override YAML config, which overrides default values
- The order of precedence: command-line > YAML config > default value

---

## Next Steps (Ideas for Future Learning)

- Learn about parameter callbacks to handle runtime parameter changes
- Learn about parameter types (string, int, double, bool, arrays)
- Learn about parameter validation
- Learn about listing and describing parameters with `ros2 param list` and `ros2 param describe`

