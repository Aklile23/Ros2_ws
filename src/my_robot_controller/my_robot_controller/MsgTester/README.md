# MsgTester Folder - Custom Message Testing

I created this folder to learn how to create custom message files and use them in ROS2 Python scripts. This is where I learned the complete workflow of defining custom messages and using them in my nodes.

## Overview

This folder contains a script that demonstrates how to:
- Create custom message (`.msg`) files in a separate interface package
- Import and use custom messages in Python nodes
- Publish custom message types to topics
- Understand the relationship between interface packages and controller packages

---

## Script

### `hw_status_publisher.py`

**What I did:** I created a publisher node that uses my custom `HardwareStatus` message type to publish hardware status information.

**What it does:**
- Creates a node named `hw_status_publisher`
- Imports the custom `HardwareStatus` message from `my_robot_interfaces`
- Creates a publisher for the `/hardware_status` topic using the custom message type
- Publishes hardware status messages every 1 second with:
  - Temperature: 45.0
  - Motor readiness: True
  - Debug message: "Everything is working fine"

**Key concepts I learned:**

1. **Importing Custom Messages:**
   ```python
   from my_robot_interfaces.msg import HardwareStatus
   ```
   - Custom messages are imported from the interface package
   - Format: `from <package_name>.msg import <MessageName>`
   - The package must be built and sourced before it can be imported

2. **Using Custom Messages in Publishers:**
   ```python
   self.publisher_ = self.create_publisher(HardwareStatus, "hardware_status", 10)
   ```
   - Use the custom message type just like standard messages (e.g., `Twist`, `Pose`)
   - The message type is the first argument to `create_publisher()`

3. **Creating and Populating Custom Messages:**
   ```python
   msg = HardwareStatus()
   msg.temperature = 45.0
   msg.are_motors_ready = True
   msg.debug_message = "Everything is working fine"
   ```
   - Create an instance of the custom message
   - Set each field using dot notation
   - Field names match exactly what's defined in the `.msg` file

4. **Publishing Custom Messages:**
   ```python
   self.publisher_.publish(msg)
   ```
   - Works exactly the same as publishing standard messages
   - The message type is automatically handled by ROS2

**How to run:**

1. **Build the workspace** (to generate message code):
   ```bash
   cd /home/aklile/ros2_ws
   colcon build --packages-select my_robot_interfaces
   source install/setup.bash
   ```

2. **Build the controller package:**
   ```bash
   colcon build --packages-select my_robot_controller
   source install/setup.bash
   ```

3. **Run the node:**
   ```bash
   ros2 run my_robot_controller <executable_name>
   ```

4. **View the published messages:**
   ```bash
   ros2 topic echo /hardware_status
   ```

**What you'll see:**
- The node logs: "Hardware status publisher has been started"
- Messages are published to `/hardware_status` topic every second
- Each message contains temperature, are_motors_ready, and debug_message fields

---

## Custom Message Definition

I created the custom message file in a separate interface package:

**Location:** `my_robot_interfaces/msg/HardwareStatus.msg`

**Contents:**
```
float64 temperature 
bool are_motors_ready
string debug_message
```

**What I learned about message files:**

1. **Message File Structure:**
   - Each line defines a field: `<type> <field_name>`
   - Types can be: `bool`, `int8`, `int16`, `int32`, `int64`, `uint8`, `uint16`, `uint32`, `uint64`, `float32`, `float64`, `string`, `time`, `duration`
   - Field names use lowercase with underscores (snake_case)
   - Comments can be added with `#`

2. **Why Separate Interface Package:**
   - Interface packages (`my_robot_interfaces`) define message/service types
   - Controller packages (`my_robot_controller`) use those types
   - This separation allows multiple packages to use the same interfaces
   - Follows ROS2 best practices for package organization

3. **Message Generation:**
   - ROS2 automatically generates Python/C++ code from `.msg` files
   - Generated code is placed in the install directory after building
   - The interface package must be built before other packages can use it

---

## Complete Workflow: Creating and Using Custom Messages

Here's the complete process I learned:

### Step 1: Create the Message File
1. Create a `msg/` directory in your interface package (`my_robot_interfaces/`)
2. Create a `.msg` file (e.g., `HardwareStatus.msg`)
3. Define the message fields with types and names

### Step 2: Configure the Interface Package
1. Update `CMakeLists.txt` to include message generation:
   ```cmake
   find_package(rosidl_default_generators REQUIRED)
   rosidl_generate_interfaces(${PROJECT_NAME}
     "msg/HardwareStatus.msg"
   )
   ```

2. Update `package.xml` to include dependencies:
   ```xml
   <buildtool_depend>rosidl_default_generators</buildtool_depend>
   <exec_depend>rosidl_default_runtime</exec_depend>
   <member_of_group>rosidl_interface_packages</member_of_group>
   ```

### Step 3: Build the Interface Package
```bash
colcon build --packages-select my_robot_interfaces
source install/setup.bash
```

### Step 4: Use the Message in Your Code
1. Import the message:
   ```python
   from my_robot_interfaces.msg import HardwareStatus
   ```

2. Use it in publishers/subscribers:
   ```python
   self.publisher_ = self.create_publisher(HardwareStatus, "topic_name", 10)
   ```

3. Create and populate message instances:
   ```python
   msg = HardwareStatus()
   msg.field_name = value
   ```

### Step 5: Build and Run
```bash
colcon build --packages-select my_robot_controller
source install/setup.bash
ros2 run my_robot_controller <executable_name>
```

---

## Custom Messages vs. Standard Messages

**Standard Messages (from ROS2 packages):**
```python
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from turtlesim.msg import Pose
```
- Pre-defined by ROS2 or other packages
- Available after installing packages
- Examples: `Twist`, `Pose`, `String`, `Int32`, etc.

**Custom Messages (from my interface package):**
```python
from my_robot_interfaces.msg import HardwareStatus
```
- Defined by me in my workspace
- Must be built before use
- Specific to my application needs
- Can be shared across multiple packages in my workspace

**Benefits of Custom Messages:**
- Tailored to my specific application
- Can include exactly the data I need
- Better semantic meaning (e.g., `HardwareStatus` vs. generic `String`)
- Type safety and validation
- Self-documenting (the message definition shows what data is included)

---

## Dependencies

The script uses:
- `rclpy` - ROS2 Python client library
- `my_robot_interfaces.msg.HardwareStatus` - My custom message type

**Important:** The `my_robot_interfaces` package must be:
- Built before `my_robot_controller` can use it
- Listed as a dependency in `my_robot_controller/package.xml`:
  ```xml
  <depend>my_robot_interfaces</depend>
  ```

---

## Message Field Types

From my `HardwareStatus.msg`, I learned about these types:

- **`float64`** - Double-precision floating-point number (temperature)
- **`bool`** - Boolean value (are_motors_ready)
- **`string`** - Text string (debug_message)

**Other common types I could use:**
- `int32`, `int64` - Integer numbers
- `uint32`, `uint64` - Unsigned integers
- `float32` - Single-precision floating-point
- Arrays: `int32[]`, `string[]` - Lists of values
- Nested messages: Can use other message types as fields

---

## Notes for Future Me

- **Build order matters:** Always build interface packages before packages that use them
- **Source after building:** Must `source install/setup.bash` after building to make messages available
- **Field names:** Must match exactly (case-sensitive) between `.msg` file and Python code
- **Message instances:** Create new instances for each message (don't reuse the same object)
- **Import path:** Format is `from <package_name>.msg import <MessageName>`
- **Package naming:** Interface packages often end with `_interfaces` or `_msgs`
- **Multiple messages:** One interface package can define many messages
- **Message validation:** ROS2 validates message types at runtime
- **Topic type checking:** Publishers and subscribers must use the same message type

---

## Testing Custom Messages

I can verify my custom message works by:

1. **Check message definition:**
   ```bash
   ros2 interface show my_robot_interfaces/msg/HardwareStatus
   ```

2. **List available topics:**
   ```bash
   ros2 topic list
   ```

3. **Check topic type:**
   ```bash
   ros2 topic type /hardware_status
   ```

4. **Echo the topic:**
   ```bash
   ros2 topic echo /hardware_status
   ```

5. **Check message info:**
   ```bash
   ros2 topic info /hardware_status
   ```

---

## Next Steps (Ideas for Future Learning)

- Create a subscriber node that receives `HardwareStatus` messages
- Create more custom message types for different purposes
- Learn about arrays and nested messages in custom types
- Learn about using custom messages in services (not just topics)
- Learn about message validation and constraints
- Learn about message documentation and annotations

