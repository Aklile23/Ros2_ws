# Launch Folder - ROS2 Launch Files

I created this folder to learn how to create ROS2 launch files in both Python and XML formats. Launch files allow me to start multiple nodes together with proper configuration, making it much easier to run complex robot systems.

## Overview

This folder contains launch files that demonstrate how to:
- Launch multiple ROS2 nodes simultaneously
- Load parameter configuration files (YAML)
- Use both Python and XML launch file formats
- Organize launch files in a dedicated package
- Pass parameters to nodes via launch files

---

## Launch Files

### 1. `number_app.launch.py` (Python Launch File)

**What I did:** I created a Python launch file to learn the Python-based launch file API, which is more powerful and flexible than XML.

**What it does:**
- Launches two nodes from `my_robot_controller`:
  - `test_node` - A simple test node
  - `number_publisher_W_Param` - A number publisher with parameters loaded from YAML
- Loads parameters from `config/number_app.yaml` for the number publisher node
- Demonstrates the Python launch API structure

**Key concepts I learned:**

1. **Launch Description Structure:**
   ```python
   def generate_launch_description():
       ld = LaunchDescription()
       # Add actions here
       return ld
   ```
   - Must define `generate_launch_description()` function
   - Returns a `LaunchDescription` object
   - This is the entry point for Python launch files

2. **Creating Node Actions:**
   ```python
   my_first_node = Node(
       package="my_robot_controller",
       executable="test_node",
   )
   ```
   - Use `Node` from `launch_ros.actions`
   - `package` - The package name containing the executable
   - `executable` - The name of the executable to run
   - Can optionally set `name` to override the node name

3. **Loading Parameter Files:**
   ```python
   param_config = os.path.join(
       get_package_share_directory("my_robot_bringup"), 
       "config", 
       "number_app.yaml"
   )
   number_publisher_W_Param = Node(
       package="my_robot_controller",
       executable="number_publisher_W_Param",
       parameters=[param_config]
   )
   ```
   - Use `get_package_share_directory()` to find package paths
   - Use `os.path.join()` to build file paths
   - Pass parameter file path to `parameters` argument (as a list)

4. **Adding Actions to Launch Description:**
   ```python
   ld.add_action(my_first_node)
   ld.add_action(number_publisher_W_Param)
   ```
   - Must add all nodes/actions to the LaunchDescription
   - Order matters - nodes are started in the order they're added

**How to run:**
```bash
ros2 launch my_robot_bringup number_app.launch.py
```

**Advantages of Python launch files:**
- More powerful and flexible
- Can use Python logic (conditionals, loops, functions)
- Better for complex launch scenarios
- Can programmatically generate launch configurations
- Better error handling and debugging

---

### 2. `number_app.launch.xml` (XML Launch File)

**What I did:** I created an XML launch file to learn the XML-based launch file format, which is simpler and more declarative.

**What it does:**
- Launches the same two nodes as the Python version
- Uses XML syntax to define nodes and parameters
- Demonstrates the XML launch file structure

**Key concepts I learned:**

1. **XML Launch File Structure:**
   ```xml
   <launch>
       <!-- Nodes go here -->
   </launch>
   ```
   - Root element is `<launch>`
   - All nodes and configurations go inside this tag

2. **Defining Nodes:**
   ```xml
   <node pkg="my_robot_controller" exec="test_node" name="my_first_node">
   </node>
   ```
   - Use `<node>` element
   - `pkg` - Package name (equivalent to `package` in Python)
   - `exec` - Executable name (equivalent to `executable` in Python)
   - `name` - Optional node name override

3. **Loading Parameter Files:**
   ```xml
   <node pkg="my_robot_controller" exec="number_publisher_W_Param">
       <param from="$(find-pkg-share my_robot_bringup)/config/number_app.yaml"/>
   </node>
   ```
   - Use `<param>` element with `from` attribute
   - `$(find-pkg-share <package>)` - Substitution to find package share directory
   - Path to YAML file relative to package share directory

**How to run:**
```bash
ros2 launch my_robot_bringup number_app.launch.xml
```

**Advantages of XML launch files:**
- Simpler syntax for basic use cases
- More declarative and readable for simple scenarios
- Familiar to ROS1 users
- Good for straightforward node launching

---

### 3. `lifecycle_test.launch.py` (Python Launch File - Lifecycle Nodes)

**What I did:** I created a Python launch file to learn how to launch ROS2 lifecycle nodes and their managers.

**What it does:**
- Launches two nodes from `lifecycle_py`:
  - `number_publisher_lifecycle` - A lifecycle node that publishes numbers
  - `lifecycle_node_manager` - A manager node that controls the lifecycle node's state transitions
- Passes the managed node name as a parameter to the lifecycle manager
- Demonstrates launching lifecycle nodes and passing parameters programmatically

**Key concepts I learned:**

1. **Passing Parameters as Dictionary:**
   ```python
   lifecycle_node_manager = Node(
       package="lifecycle_py",
       executable="lifecycle_node_manager",
       parameters=[{"managed_node_name": managed_node_name}]
   )
   ```
   - Can pass parameters directly as a dictionary in a list
   - Useful for simple parameter passing without YAML files
   - Parameters are passed as key-value pairs

2. **Using Variables in Launch Files:**
   ```python
   managed_node_name = "number_publisher_lifecycle"
   
   my_number_publisher = Node(
       package="lifecycle_py",
       executable="number_publisher_lifecycle",
       name=managed_node_name
   )
   ```
   - Can use Python variables to avoid hardcoding values
   - Makes launch files more maintainable
   - Same variable can be reused for node name and parameter value

3. **Lifecycle Node Management:**
   - Lifecycle nodes have states: Unconfigured → Inactive → Active
   - Manager nodes control state transitions via services
   - Manager needs to know which node to manage (passed as parameter)

**How to run:**
```bash
ros2 launch my_robot_bringup lifecycle_test.launch.py
```

**What happens:**
1. The lifecycle node starts in "Unconfigured" state
2. The manager node automatically transitions it through:
   - Configure → Inactive
   - Activate → Active
3. Once active, the lifecycle node starts publishing

---

### 4. `lifecycle_test.launch.xml` (XML Launch File - Lifecycle Nodes)

**What I did:** I created an XML launch file to learn how to launch lifecycle nodes using XML syntax.

**What it does:**
- Launches the same two nodes as the Python version:
  - `number_publisher_lifecycle` - A lifecycle node that publishes numbers
  - `lifecycle_node_manager` - A manager node that controls the lifecycle node's state transitions
- Uses XML syntax to define nodes and pass parameters
- Demonstrates how to use variables and parameter passing in XML format

**Key concepts I learned:**

1. **Using Variables in XML:**
   ```xml
   <let name="managed_node_name" value="number_publisher_lifecycle" />
   ```
   - Use `<let>` element to define variables
   - Can reference variables using `$(var variable_name)`
   - Useful for avoiding repetition and maintaining consistency
   - Similar to Python variables but uses XML syntax

2. **Using Variables in Node Names:**
   ```xml
   <node pkg="lifecycle_py" exec="number_publisher_lifecycle" name="$(var managed_node_name)" />
   ```
   - Can use variable substitution in node `name` attribute
   - Uses `$(var variable_name)` syntax
   - Ensures the node name matches the variable value

3. **Passing Parameters in XML:**
   ```xml
   <node pkg="lifecycle_py" exec="lifecycle_node_manager">
       <param name="managed_node_name" value="$(var managed_node_name)" />
   </node>
   ```
   - Use `<param>` element with `name` and `value` attributes
   - Can use variable substitution with `$(var variable_name)`
   - Parameters are passed directly to the node
   - Equivalent to passing a dictionary in Python launch files

4. **Complete XML Structure:**
   ```xml
   <launch>
       <let name="managed_node_name" value="number_publisher_lifecycle" />
       <node pkg="lifecycle_py" exec="number_publisher_lifecycle" name="$(var managed_node_name)" />
       <node pkg="lifecycle_py" exec="lifecycle_node_manager">
           <param name="managed_node_name" value="$(var managed_node_name)" />
       </node>
   </launch>
   ```
   - Shows the complete structure for launching lifecycle nodes in XML
   - Demonstrates variable definition, node creation, and parameter passing

**How to run:**
```bash
ros2 launch my_robot_bringup lifecycle_test.launch.xml
```

**What happens:**
1. The lifecycle node starts in "Unconfigured" state
2. The manager node automatically transitions it through:
   - Configure → Inactive
   - Activate → Active
3. Once active, the lifecycle node starts publishing

**Comparison with Python version:**
- Both XML and Python versions do exactly the same thing
- XML uses `<let>` and `$(var name)` for variables
- Python uses regular Python variables
- XML uses `<param>` element for parameters
- Python uses dictionary in `parameters` list
- Both are equally valid - choose based on preference or complexity

---

### 5. `display.launch.py` (Python Launch File - Robot Visualization)

**What I did:** I created a Python launch file to learn how to visualize robots in RViz using URDF files from the `my_robot_description` package.

**What it does:**
- Launches three nodes for robot visualization:
  - `robot_state_publisher` - Publishes the robot description from URDF
  - `joint_state_publisher_gui` - GUI for manually controlling joint positions
  - `rviz2` - RViz visualization tool with pre-configured settings
- Loads URDF file from `my_robot_description` package
- Loads RViz configuration from `my_robot_description` package
- Uses `xacro` to process the URDF file

**Key concepts I learned:**

1. **Loading URDF Files from Other Packages:**
   ```python
   urdf_path = os.path.join(
       get_package_share_directory("my_robot_description"), 
       "urdf", 
       "my_robot.urdf"
   )
   ```
   - Use `get_package_share_directory()` to find files in other packages
   - Build paths using `os.path.join()` for cross-platform compatibility
   - URDF files are typically in `urdf/` directory

2. **Processing URDF with Xacro:**
   ```python
   robot_description = ParameterValue(
       Command(["xacro ", urdf_path]), 
       value_type=str
   )
   ```
   - Use `Command` substitution to run shell commands
   - `xacro` processes URDF files (can expand macros, handle includes)
   - Wrap in `ParameterValue` to pass as parameter
   - `value_type=str` specifies the parameter type

3. **Passing Robot Description to robot_state_publisher:**
   ```python
   robot_state_publisher_node = Node(
       package="robot_state_publisher",
       executable="robot_state_publisher",
       parameters=[{"robot_description": robot_description}]
   )
   ```
   - `robot_state_publisher` needs the URDF as a parameter
   - Parameter name is `robot_description`
   - Pass as dictionary in `parameters` list

4. **Loading RViz Configuration:**
   ```python
   rviz_config_path = os.path.join(
       get_package_share_directory("my_robot_description"), 
       "rviz", 
       "config.rviz"
   )
   rviz_node = Node(
       package="rviz2",
       executable="rviz2",
       output="screen",
       arguments=["-d", rviz_config_path]
   )
   ```
   - RViz config files are typically in `rviz/` directory
   - Use `arguments` parameter to pass command-line arguments
   - `-d` flag loads a configuration file
   - `output="screen"` shows RViz output in terminal

5. **Joint State Publisher GUI:**
   ```python
   joint_state_publisher_node = Node(
       package="joint_state_publisher_gui",
       executable="joint_state_publisher_gui",
   )
   ```
   - Provides GUI sliders for controlling joint positions
   - Useful for testing and visualizing joint movements
   - Publishes joint states that `robot_state_publisher` uses

**How to run:**
```bash
ros2 launch my_robot_bringup display.launch.py
```

**What you'll see:**
1. RViz window opens with the robot model displayed
2. Joint State Publisher GUI opens with sliders for each joint
3. You can move the sliders to see the robot's joints move in RViz
4. The robot model is displayed according to the URDF description

---

### 6. `display.launch.xml` (XML Launch File - Robot Visualization)

**What I did:** I created an XML launch file to learn how to visualize robots using XML syntax.

**What it does:**
- Launches the same three nodes as the Python version:
  - `robot_state_publisher` - Publishes the robot description
  - `joint_state_publisher_gui` - GUI for joint control
  - `rviz2` - RViz visualization tool
- Uses XML syntax with variables for file paths
- Demonstrates XML substitutions and command execution

**Key concepts I learned:**

1. **Using Variables in XML:**
   ```xml
   <let name="urdf_path" value="$(find-pkg-share my_robot_description)/urdf/my_robot.urdf"/>
   <let name="rviz_config_path" value="$(find-pkg-share my_robot_description)/rviz/config.rviz"/>
   ```
   - Use `<let>` element to define variables
   - `$(find-pkg-share <package>)` finds package share directory
   - Variables can be referenced using `$(var variable_name)`
   - Makes paths reusable and maintainable

2. **Processing URDF with Xacro in XML:**
   ```xml
   <node pkg="robot_state_publisher" exec="robot_state_publisher">
       <param name="robot_description" value="$(command 'xacro $(var urdf_path)')"/>
   </node>
   ```
   - Use `$(command '...')` to execute shell commands
   - Can reference variables using `$(var variable_name)`
   - `xacro` processes the URDF file
   - Result is passed as parameter value

3. **Passing Parameters in XML:**
   ```xml
   <param name="robot_description" value="$(command 'xacro $(var urdf_path)')"/>
   ```
   - Use `<param>` element with `name` and `value` attributes
   - `name` is the parameter name
   - `value` can use substitutions and commands

4. **Passing Arguments to Nodes:**
   ```xml
   <node pkg="rviz2" exec="rviz2" output="screen" args="-d $(var rviz_config_path)"/>
   ```
   - Use `args` attribute to pass command-line arguments
   - Can reference variables using `$(var variable_name)`
   - `output="screen"` shows output in terminal

5. **Complete XML Structure:**
   ```xml
   <launch>
       <let name="urdf_path" value="$(find-pkg-share my_robot_description)/urdf/my_robot.urdf"/>
       <let name="rviz_config_path" value="$(find-pkg-share my_robot_description)/rviz/config.rviz"/>
       
       <node pkg="robot_state_publisher" exec="robot_state_publisher">
           <param name="robot_description" value="$(command 'xacro $(var urdf_path)')"/>
       </node>
       
       <node pkg="joint_state_publisher_gui" exec="joint_state_publisher_gui" />
       
       <node pkg="rviz2" exec="rviz2" output="screen" args="-d $(var rviz_config_path)"/>
   </launch>
   ```
   - Shows complete structure for robot visualization in XML
   - Demonstrates variables, parameter passing, and command execution

**How to run:**
```bash
ros2 launch my_robot_bringup display.launch.xml
```

**What you'll see:**
1. RViz window opens with the robot model displayed
2. Joint State Publisher GUI opens with sliders for each joint
3. You can move the sliders to see the robot's joints move in RViz
4. The robot model is displayed according to the URDF description

**Comparison with Python version:**
- Both XML and Python versions do exactly the same thing
- XML uses `<let>` and `$(var name)` for variables
- Python uses regular Python variables
- XML uses `$(command '...')` for shell commands
- Python uses `Command()` substitution
- Both are equally valid - choose based on preference

---

## Comparison: Python vs. XML Launch Files

I learned that both formats can do the same thing, but have different strengths:

| Feature | Python Launch | XML Launch |
|---------|---------------|------------|
| **Syntax** | Python code | XML markup |
| **Complexity** | More powerful, can be complex | Simpler, more declarative |
| **Logic** | Can use Python conditionals, loops, functions | Limited to XML substitutions |
| **File paths** | Use `os.path.join()` and functions | Use `$(find-pkg-share)` substitution |
| **Best for** | Complex scenarios, dynamic configurations | Simple, straightforward launches |
| **Learning curve** | Steeper (need to know Python) | Gentler (just XML syntax) |

**My recommendation:** Start with XML for simple cases, use Python for complex scenarios.

---

## What the Launch Files Do

### `number_app` Launch Files

Both launch files start the same system:

1. **`test_node`** (my_first_node)
   - A simple test node from `my_robot_controller`
   - Runs independently

2. **`number_publisher_W_Param`**
   - Number publisher node that uses parameters
   - Parameters loaded from `config/number_app.yaml`:
     - `number: 10.0`
     - `timer_period: 0.5`
   - Publishes numbers to `/number` topic

**Result:** Both nodes run simultaneously, and the number publisher uses the configured parameters.

### `lifecycle_test` Launch Files

Both Python and XML launch files start the same lifecycle node system:

1. **`number_publisher_lifecycle`**
   - A lifecycle node that publishes numbers
   - Starts in "Unconfigured" state
   - Must be transitioned through states: Unconfigured → Inactive → Active
   - Only publishes when in "Active" state

2. **`lifecycle_node_manager`**
   - Manages the lifecycle node's state transitions
   - Receives the managed node name as a parameter (`managed_node_name`)
   - Automatically transitions the lifecycle node:
     - Calls configure service → moves to Inactive
     - Waits 3 seconds
     - Calls activate service → moves to Active

**Result:** The lifecycle node starts, gets managed by the manager, and automatically transitions to Active state where it begins publishing.

**Note:** Both Python and XML versions work identically - they launch the same nodes with the same parameters. The only difference is the syntax used to define them.

### `display` Launch Files

Both Python and XML launch files start the same robot visualization system:

1. **`robot_state_publisher`**
   - Publishes the robot description from `my_robot_description` package
   - Receives URDF file as `robot_description` parameter
   - Processes URDF with `xacro` before passing to node
   - Publishes robot state on `/robot_description` topic

2. **`joint_state_publisher_gui`**
   - Provides GUI with sliders for each joint
   - Allows manual control of joint positions
   - Publishes joint states that `robot_state_publisher` uses
   - Useful for testing and visualizing joint movements

3. **`rviz2`**
   - RViz visualization tool
   - Loads pre-configured settings from `my_robot_description/rviz/config.rviz`
   - Displays the robot model in 3D
   - Shows robot structure, links, and joints

**Result:** RViz opens with the robot model displayed, and you can use the Joint State Publisher GUI to move joints and see the robot move in RViz.

**Note:** Both Python and XML versions work identically - they launch the same nodes with the same configuration. The robot description is loaded from `my_robot_description` package.

---

## Parameter Configuration File

The launch files load parameters from:

**Location:** `my_robot_bringup/config/number_app.yaml`

**Contents:**
```yaml
/number_publisher:
  ros__parameters:
    number: 10.0
    timer_period: 0.5
```

**How it works:**
- The node name (`/number_publisher`) must match the node name in the code
- `ros__parameters:` is the required namespace for ROS2 parameters
- Parameters are listed under this namespace
- Both Python and XML launch files can load this same file

---

## Complete Workflow

Here's how I use launch files:

1. **Build the workspace:**
   ```bash
   cd /home/aklile/ros2_ws
   colcon build
   source install/setup.bash
   ```

2. **Run the launch file:**
   ```bash
   # Number app launch files
   ros2 launch my_robot_bringup number_app.launch.py
   # OR
   ros2 launch my_robot_bringup number_app.launch.xml
   
   # Lifecycle test launch files
   ros2 launch my_robot_bringup lifecycle_test.launch.py
   # OR 
   ros2 launch my_robot_bringup lifecycle_test.launch.xml
   
   # Display launch files (robot visualization)
   ros2 launch my_robot_bringup display.launch.py
   # OR
   ros2 launch my_robot_bringup display.launch.xml
   ```

3. **Verify nodes are running:**
   ```bash
   # In another terminal
   ros2 node list
   ros2 topic list
   ros2 topic echo /number
   ```

---

## Key Differences in Syntax

### Node Definition

**Python:**
```python
Node(
    package="my_robot_controller",
    executable="test_node",
    name="my_first_node"  # optional
)
```

**XML:**
```xml
<node 
    pkg="my_robot_controller" 
    exec="test_node" 
    name="my_first_node">  <!-- optional -->
</node>
```

### Parameter Loading

**Python:**
```python
param_config = os.path.join(
    get_package_share_directory("my_robot_bringup"), 
    "config", 
    "number_app.yaml"
)
Node(
    package="my_robot_controller",
    executable="number_publisher_W_Param",
    parameters=[param_config]
)
```

**XML:**
```xml
<node pkg="my_robot_controller" exec="number_publisher_W_Param">
    <param from="$(find-pkg-share my_robot_bringup)/config/number_app.yaml"/>
</node>
```

---

## Why Launch Files Are Important

I learned that launch files are essential because:

1. **Start Multiple Nodes:** Instead of running each node in separate terminals, launch files start them all together

2. **Configuration Management:** Load parameters, set namespaces, remap topics - all in one place

3. **Reproducibility:** Same launch file always starts the same system configuration

4. **Organization:** Keep launch files in a dedicated `bringup` package (common ROS2 practice)

5. **Easier Testing:** Can test entire systems with one command

6. **Production Ready:** Real robot systems use launch files, not manual node starting

---

## Launch File Best Practices

What I learned about organizing launch files:

1. **Dedicated Package:** Keep launch files in a `_bringup` package (like `my_robot_bringup`)

2. **Naming Convention:** Use descriptive names like `number_app.launch.py` or `robot_bringup.launch.xml`

3. **Configuration Files:** Keep YAML configs in a `config/` directory within the bringup package

4. **Documentation:** Add comments explaining what each node does

5. **Modularity:** Create separate launch files for different scenarios (simulation, real robot, testing)

---

## Dependencies

**Python Launch File:**
- `launch` - ROS2 launch framework
- `launch_ros.actions.Node` - For creating node actions
- `ament_index_python.packages.get_package_share_directory` - For finding package paths

**XML Launch File:**
- No Python dependencies (pure XML)
- Uses ROS2 launch XML parser

**Both require:**
- The packages being launched (`my_robot_controller`, `lifecycle_py`, `my_robot_description`) must be built
- Parameter files must exist at the specified paths (if using YAML configs)
- URDF files must exist in the description package (for display launch files)

---

## Notes for Future Me

- **File extensions:** Python launch files use `.launch.py`, XML use `.launch.xml`
- **Package structure:** Launch files go in `launch/` directory, configs in `config/` directory
- **Node names:** Can override node names in launch files (useful for running multiple instances)
- **Parameter precedence:** Command-line > Launch file > YAML config > Default values
- **Substitutions:** XML uses `$(find-pkg-share)` while Python uses `get_package_share_directory()`
- **Both formats work:** Choose based on complexity - XML for simple, Python for complex
- **XML limitations:** XML launch files cannot include Python launch files using `<include>` - only XML files
- **Variables:** XML uses `<let>` and `$(var name)`, Python uses regular variables
- **Parameter passing:** Can pass parameters as dictionary in Python, or use `<param>` in XML
- **Testing:** Can test launch files without running them using `ros2 launch --show-args`
- **Debugging:** Use `ros2 launch --debug` to see what's happening
- **Multiple instances:** Can launch the same node multiple times with different names/namespaces
- **Lifecycle nodes:** Need a manager node to transition states; manager needs the managed node name as parameter
- **Robot visualization:** Use `robot_state_publisher` to publish URDF; use `xacro` to process URDF files
- **RViz configuration:** Can load pre-configured RViz settings using `-d` argument
- **Command execution:** Use `Command()` in Python or `$(command '...')` in XML to run shell commands like `xacro`
- **Cross-package file access:** Use `get_package_share_directory()` in Python or `$(find-pkg-share)` in XML to access files from other packages

---

## Common Launch File Tasks

Here are things I can do with launch files:

1. **Launch multiple nodes** ✓ (learned this)
2. **Load parameters from YAML** ✓ (learned this)
3. **Set node names** ✓ (learned this)
4. **Pass parameters directly (dictionary)** ✓ (learned this - lifecycle launch files)
5. **Use variables in launch files** ✓ (learned this - lifecycle launch files)
6. **Launch lifecycle nodes** ✓ (learned this)
7. **Load URDF files from other packages** ✓ (learned this - display launch files)
8. **Use xacro to process URDF** ✓ (learned this - display launch files)
9. **Launch robot visualization tools** ✓ (learned this - display launch files)
10. **Pass command-line arguments to nodes** ✓ (learned this - display launch files)
11. **Remap topics** (for future learning)
12. **Set namespaces** (for future learning)
13. **Use conditionals** (Python only, for future learning)
14. **Include other launch files** (for future learning - note: XML can't include Python files)
15. **Set environment variables** (for future learning)
16. **Use groups and namespaces** (for future learning)

---

## Next Steps (Ideas for Future Learning)

- Learn about topic remapping in launch files
- Learn about namespaces and groups
- Learn about including other launch files
- Learn about conditional launching (Python)
- Learn about launch file arguments/parameters
- Learn about event handlers and lifecycle management
- Create launch files for different scenarios (simulation, real robot, testing)

