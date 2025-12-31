# my_robot_interfaces - Custom ROS2 Interface Definitions

I created this package to learn how to define custom message and service types in ROS2. This is an interface package that defines custom communication types used across my workspace.

## Package Overview

This is a ROS2 interface package that defines custom message (`.msg`) and service (`.srv`) types. Interface packages are used to define communication contracts that can be shared across multiple packages in a workspace.

**Package Type:** Interface Package (ament_cmake)  
**Build System:** CMake with `rosidl_default_generators`  
**Purpose:** Define custom message and service types

---

## Package Contents

### `msg/` - Message Definitions
Contains custom message type definitions:

#### `HardwareStatus.msg`
A custom message for reporting hardware status information:
```
float64 temperature      # Temperature reading
bool are_motors_ready    # Motor readiness status
string debug_message     # Debug information
```

**Usage:** This message is used by nodes in `my_robot_controller` to publish hardware status information. See `my_robot_controller/MsgTester/hw_status_publisher.py` for an example.

### `srv/` - Service Definitions
Contains custom service type definitions:

#### `ComputeRectangleArea.srv`
A custom service for calculating rectangle area:
```
# Request
float64 length
float64 width
---
# Response
float64 area
```

**Usage:** This service can be used by service servers and clients to calculate rectangle areas.

### `CMakeLists.txt` - Build Configuration
CMake configuration file that:
- Finds `rosidl_default_generators` package
- Generates Python, C++, and other language bindings from `.msg` and `.srv` files
- Registers the interface files for code generation

**Interface files registered:**
- `msg/HardwareStatus.msg`
- `srv/ComputeRectangleArea.srv`

### `package.xml` - Package Metadata
Package manifest file defining:
- Package name, version, description
- Build tool dependencies (`ament_cmake`, `rosidl_default_generators`)
- Runtime dependencies (`rosidl_default_runtime`)
- Member of `rosidl_interface_packages` group (identifies this as an interface package)

---

## What I Learned

This package helped me learn:
- ✅ Creating custom message (`.msg`) files
- ✅ Creating custom service (`.srv`) files
- ✅ Understanding interface packages in ROS2
- ✅ How ROS2 generates code from interface definitions
- ✅ Sharing custom types across multiple packages
- ✅ The relationship between interface packages and controller packages

---

## Building and Using

**Build the package:**
```bash
cd /home/aklile/ros2_ws
colcon build --packages-select my_robot_interfaces
source install/setup.bash
```

**Important:** This package must be built before any packages that use its interfaces (like `my_robot_controller`).

**Using the interfaces in Python:**
```python
from my_robot_interfaces.msg import HardwareStatus
from my_robot_interfaces.srv import ComputeRectangleArea
```

**Using the interfaces in C++:**
```cpp
#include "my_robot_interfaces/msg/hardware_status.hpp"
#include "my_robot_interfaces/srv/compute_rectangle_area.hpp"
```

**View interface definitions:**
```bash
ros2 interface show my_robot_interfaces/msg/HardwareStatus
ros2 interface show my_robot_interfaces/srv/ComputeRectangleArea
```

---

## Package Structure

```
my_robot_interfaces/
├── CMakeLists.txt          # Build configuration
├── package.xml             # Package metadata
├── msg/                     # Message definitions
│   └── HardwareStatus.msg
└── srv/                     # Service definitions
    └── ComputeRectangleArea.srv
```

---

## Interface Package Workflow

I learned the complete workflow for creating and using custom interfaces:

1. **Define interfaces** - Create `.msg` and `.srv` files in this package
2. **Build interface package** - Generate code from interface definitions
3. **Use in other packages** - Import and use the generated types
4. **Build dependent packages** - Build packages that use these interfaces

**Build order matters:**
```bash
# 1. Build interface package first
colcon build --packages-select my_robot_interfaces
source install/setup.bash

# 2. Then build packages that use the interfaces
colcon build --packages-select my_robot_controller
source install/setup.bash
```

---

## Message and Service Syntax

### Message (`.msg`) Syntax
- Each line defines a field: `<type> <field_name>`
- Types: `bool`, `int8-64`, `uint8-64`, `float32`, `float64`, `string`, `time`, `duration`
- Arrays: `int32[]`, `string[]`
- Comments: `# comment`

### Service (`.srv`) Syntax
- Request fields (above `---`)
- Separator: `---`
- Response fields (below `---`)

---

## Dependencies

**This package provides:**
- Custom message types for other packages to use
- Custom service types for other packages to use

**This package depends on:**
- `rosidl_default_generators` - For generating code from interfaces
- `rosidl_default_runtime` - Runtime support for interfaces

**Packages that use this:**
- `my_robot_controller` - Uses `HardwareStatus` message

---

## Notes

- Interface packages are special - they only define types, no executable code
- Interface files are compiled into language-specific code (Python, C++, etc.)
- The package must be built before dependent packages can use the interfaces
- Interface packages follow the `rosidl_interface_packages` group convention
- Custom interfaces allow for type-safe, application-specific communication
- Interface definitions are language-agnostic - one definition works for all languages

---

## Next Steps (Ideas for Future Learning)

- Create more custom message types for different purposes
- Create more custom service types
- Learn about action interfaces (`.action` files)
- Learn about arrays and nested messages in custom types
- Learn about message validation and constraints
- Learn about interface documentation and annotations

