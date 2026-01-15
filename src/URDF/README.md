# My Robot URDF Documentation

## What is URDF?

URDF (Unified Robot Description Format) is an XML file format used in ROS (Robot Operating System) to describe the physical structure of a robot, including its links (parts), joints (connections), materials, and visual properties.

## File Structure Overview

This URDF file describes a simple mobile robot with:
- A base body (green box)
- A laser sensor (white cylinder) on top
- Two drive wheels (grey cylinders) on the sides
- A caster wheel (grey sphere) at the front

---

## Line-by-Line Explanation

### XML Declaration and Robot Root
```xml
<?xml version = "1.0"?>
```
- **Line 1**: XML declaration specifying the XML version. This tells parsers how to interpret the file.

```xml
<robot name="my_robot">
```
- **Line 2**: Opens the root `<robot>` element and names the robot "my_robot". All robot components are defined within this tag.

---

### Material Definitions (Lines 3-14)

Materials define colors that can be reused throughout the robot description.

```xml
<material name="grey">
    <color rgba="0.5 0.5 0.5 1"/>
</material>
```
- **Lines 3-6**: Defines a material named "grey" with RGBA values (Red=0.5, Green=0.5, Blue=0.5, Alpha=1.0). Alpha=1 means fully opaque.

```xml
<material name="green">
    <color rgba="0 0.6 0 1"/>
</material>
```
- **Lines 8-10**: Defines a "green" material (Red=0, Green=0.6, Blue=0, Alpha=1).

```xml
<material name="white">
    <color rgba="1 1 1 1"/>
</material>
```
- **Lines 12-14**: Defines a "white" material (all RGB values at maximum=1).

---

### Base Footprint Link (Line 16)

```xml
<link name="base_footprint" />
```
- **Line 16**: Creates a link named "base_footprint" with no visual properties. This is a reference frame typically used as the robot's origin point in the world. The self-closing tag (`/>`) means it has no child elements.

---

### Base Link (Lines 18-26)

```xml
<link name="base_link">
```
- **Line 18**: Opens the definition for "base_link", the main body of the robot.

```xml
    <visual>
```
- **Line 19**: Starts the visual properties section, which defines how the link appears in simulation/visualization.

```xml
        <geometry>
            <box size="0.6 0.4 0.2"/>
        </geometry>
```
- **Lines 20-22**: Defines the shape as a box with dimensions: width=0.6m, depth=0.4m, height=0.2m (in x, y, z order).

```xml
        <origin xyz="0 0 0.1" rpy="0 0 0"/>
```
- **Line 23**: Sets the visual's position and orientation relative to the link's origin:
  - `xyz="0 0 0.1"`: Position offset (x=0, y=0, z=0.1m upward)
  - `rpy="0 0 0"`: Rotation in Roll-Pitch-Yaw angles (all zero = no rotation)

```xml
        <material name="green"/>
```
- **Line 24**: Applies the "green" material defined earlier to this visual element.

```xml
    </visual>
</link>
```
- **Lines 25-26**: Closes the visual section and the base_link definition.

---

### Base Footprint to Base Link Joint (Lines 28-32)

```xml
<joint name="base_footprint_base_link_joint" type="fixed">
```
- **Line 28**: Creates a joint connecting base_footprint to base_link. `type="fixed"` means the two links are rigidly connected (no movement).

```xml
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
```
- **Line 29**: Joint origin: positions base_link 0.1m above base_footprint with no rotation.

```xml
    <parent link="base_footprint"/>
    <child link="base_link"/>
```
- **Lines 30-31**: Defines the parent-child relationship. base_footprint is the parent (reference frame), base_link is the child (moves relative to parent).

```xml
</joint>
```
- **Line 32**: Closes the joint definition.

---

### Laser Link (Lines 34-42)

```xml
<link name="laser">
    <visual>
        <geometry>
            <cylinder radius="0.1" length="0.05"/>
        </geometry>
```
- **Lines 34-38**: Creates a "laser" link with a cylindrical visual shape (radius=0.1m, length=0.05m).

```xml
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <material name="white"/>
```
- **Lines 39-40**: No position offset, and applies white material.

```xml
    </visual>
</link>
```
- **Lines 41-42**: Closes the laser link definition.

---

### Base to Laser Joint (Lines 44-48)

```xml
<joint name="base_laser_joint" type="fixed">
    <origin xyz="0 0 0.225" rpy="0 0 0"/>
```
- **Lines 44-45**: Fixed joint positioning the laser 0.225m above the base_link center (0.1m base height + 0.1m base offset + 0.025m half laser length).

```xml
    <parent link="base_link"/>
    <child link="laser"/>
</joint>
```
- **Lines 46-48**: Connects laser to base_link as parent.

---

### Left Wheel Link (Lines 50-58)

```xml
<link name="left_wheel">
    <visual>
        <geometry>
            <cylinder radius="0.1" length="0.05"/>
        </geometry>
        <origin xyz="0 0 0" rpy="1.57 0 0"/>
```
- **Lines 50-55**: Creates left wheel as a cylinder. The `rpy="1.57 0 0"` rotates it 90 degrees (1.57 radians ≈ π/2) around the x-axis, so the cylinder lies horizontally (wheel orientation).

```xml
        <material name="grey"/>
    </visual>
</link>
```
- **Lines 56-58**: Applies grey material and closes the link.

---

### Base to Left Wheel Joint (Lines 60-65)

```xml
<joint name="base_left_wheel_joint" type="continuous">
```
- **Line 60**: Creates a "continuous" joint type, allowing unlimited rotation (like a wheel).

```xml
    <origin xyz="-0.15 0.225 0" rpy="0 0 0"/>
```
- **Line 61**: Positions the wheel at x=-0.15m (front), y=0.225m (left side), z=0 (at base level).

```xml
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <axis xyz=" 0 1 0"/>
```
- **Lines 62-64**: Sets parent-child relationship and defines rotation axis. `xyz="0 1 0"` means rotation around the y-axis (vertical axis for a horizontal wheel).

```xml
</joint>
```
- **Line 65**: Closes the joint.

---

### Right Wheel Link (Lines 67-75)

```xml
<link name="right_wheel">
    <visual>
        <geometry>
            <cylinder radius="0.1" length="0.05"/>
        </geometry>
        <origin xyz="0 0 0" rpy="1.57 0 0"/>
        <material name="grey"/>
    </visual>
</link>
```
- **Lines 67-75**: Identical structure to left_wheel, just with a different name. Same cylinder geometry, same 90-degree rotation, same grey material.

---

### Base to Right Wheel Joint (Lines 77-82)

```xml
<joint name="base_right_wheel_joint" type="continuous">
    <origin xyz="-0.15 -0.225 0" rpy="0 0 0"/>
```
- **Lines 77-78**: Continuous joint for right wheel. Note `y=-0.225` (negative = right side, opposite of left wheel).

```xml
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <axis xyz=" 0 1 0"/>
</joint>
```
- **Lines 79-82**: Same parent-child and axis configuration as left wheel.

---

### Caster Wheel Link (Lines 84-92)

```xml
<link name='caster_wheel'>
    <visual>
        <geometry>
            <sphere radius="0.05"/>
        </geometry>
```
- **Lines 84-87**: Creates a caster wheel as a sphere with radius 0.05m (used for support/balance).

```xml
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <material name="grey"/>
    </visual>
</link>
```
- **Lines 89-92**: No rotation needed for a sphere, applies grey material.

---

### Base to Caster Wheel Joint (Lines 94-98)

```xml
<joint name="base_caster_wheel_joint" type="fixed">
    <origin xyz="0.20 0 0.05" rpy="0 0 0"/>
```
- **Lines 94-95**: Fixed joint positioning caster at x=0.20m (front), y=0 (center), z=0.05m (slightly above base).

```xml
    <parent link="base_link"/>
    <child link="caster_wheel"/>
</joint>
```
- **Lines 96-98**: Connects caster to base_link.

---

### Closing Tag

```xml
</robot>
```
- **Line 100**: Closes the root `<robot>` element, ending the URDF file.

---

## Key Concepts Summary

- **Links**: Physical parts of the robot (base, wheels, sensors)
- **Joints**: Connections between links that define how they move relative to each other
- **Visual**: How the robot appears in visualization tools (Gazebo, RViz)
- **Origin**: Position (xyz) and orientation (rpy) of elements
- **Joint Types**:
  - `fixed`: No movement (rigid connection)
  - `continuous`: Unlimited rotation (wheels)
- **Geometry Types**: `box`, `cylinder`, `sphere`
- **Materials**: Reusable color definitions applied to visual elements

---

## Usage

This URDF file can be used with ROS 2 tools like:
- `robot_state_publisher`: Publishes robot's joint states
- `rviz2`: Visualizes the robot model
- `gazebo`: Simulates the robot in a physics environment

To view your robot in RViz2:
- Install the visualizer by running
```bash
sudo apt install ros-humble-urdf-tutorial
```
- Open the urdf file inside the visualizer
  - Replace the path (/home/aklile) with the actual absolute address of the file.
```bash
ros2 launch urdf_tutoral display.launch.py model:=/home/aklile/ros2_ws/src/URDF/my_robot.urdf
```
To see the TF tree of this URDF file run:
  - This will create a PDF file with the TF tree in the directory you are currently in the terminal.
```bash
ros2 run tf2_tools view_frames
```
