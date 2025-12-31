# Topics Folder - ROS2 Topic Communication Examples

I created this folder to practice ROS2 topic-based communication. All scripts in this folder demonstrate how to publish and subscribe to topics using the publisher/subscriber pattern.

## Overview

This folder contains Python scripts that I wrote to learn about:
- Creating ROS2 nodes in Python
- Publishing messages to topics
- Subscribing to topics and receiving messages
- Working with turtlesim (ROS2's turtle simulation)
- Combining multiple functionalities in a single node

---

## Scripts

### 1. `my_first_node.py`

**What I did:** I created my very first ROS2 node in Python.

**What it does:**
- Creates a simple node named `first_node`
- Logs "Hello from ROS2" when the node starts
- Demonstrates the basic structure of a ROS2 node using object-oriented programming

**Key concepts I learned:**
- How to inherit from `Node` class
- How to initialize ROS2 with `rclpy.init()`
- How to use `rclpy.spin()` to keep the node alive
- Basic node lifecycle (init → spin → shutdown)

**How to run:**
```bash
ros2 run my_robot_controller <executable_name>
```
(Note: The executable name depends on how it's configured in `setup.py`)

---

### 2. `draw_circle.py`

**What I did:** I created a node that controls a turtle in turtlesim to draw circles.

**What it does:**
- Publishes velocity commands (`Twist` messages) to the `/turtle1/cmd_vel` topic
- Sets linear velocity (forward movement) to 2.0 m/s
- Sets angular velocity (rotation) to 1.0 rad/s
- Uses a timer to publish commands every 0.5 seconds
- The combination of linear and angular velocity makes the turtle move in a circle

**Key concepts I learned:**
- How to create a publisher using `create_publisher()`
- How to publish messages to topics
- How to use timers to periodically execute code
- Working with `geometry_msgs.msg.Twist` messages
- Controlling robots through velocity commands

**Prerequisites:**
- Turtlesim must be running: `ros2 run turtlesim turtlesim_node`

**How to run:**
```bash
# Terminal 1: Start turtlesim
ros2 run turtlesim turtlesim_node

# Terminal 2: Run the draw_circle node
ros2 run my_robot_controller <executable_name>
```

---

### 3. `pose_subscriber.py`

**What I did:** I created a node that subscribes to the turtle's pose (position and orientation) and logs it.

**What it does:**
- Subscribes to the `/turtle1/pose` topic
- Receives `Pose` messages from turtlesim
- Logs the pose information (x, y, theta, linear_velocity, angular_velocity) whenever a new message arrives

**Key concepts I learned:**
- How to create a subscriber using `create_subscription()`
- How to define a callback function that gets called when messages arrive
- Working with `turtlesim.msg.Pose` messages
- Understanding the subscriber pattern in ROS2

**Prerequisites:**
- Turtlesim must be running: `ros2 run turtlesim turtlesim_node`

**How to run:**
```bash
# Terminal 1: Start turtlesim
ros2 run turtlesim turtlesim_node

# Terminal 2: Run the pose_subscriber node
ros2 run my_robot_controller <executable_name>
```

**What you'll see:** The node will continuously print the turtle's current pose information to the console.

---

### 4. `bundle.py`

**What I did:** I combined both publisher and subscriber functionality into a single node.

**What it does:**
- **Publishes** velocity commands to `/turtle1/cmd_vel` (like `draw_circle.py`)
- **Subscribes** to `/turtle1/pose` and logs the pose (like `pose_subscriber.py`)
- Does both simultaneously in one node

**Key concepts I learned:**
- How to combine multiple functionalities in a single node
- Running a publisher and subscriber at the same time
- Understanding that nodes can have multiple publishers and subscribers
- Practical example of a more complex node structure

**Prerequisites:**
- Turtlesim must be running: `ros2 run turtlesim turtlesim_node`

**How to run:**
```bash
# Terminal 1: Start turtlesim
ros2 run turtlesim turtlesim_node

# Terminal 2: Run the bundle node
ros2 run my_robot_controller <executable_name>
```

**What you'll see:** The turtle will move in a circle while the node continuously logs the turtle's pose information.

---

## Learning Progression

I created these scripts in a learning progression:

1. **`my_first_node.py`** - Started with the absolute basics (just a node that logs)
2. **`draw_circle.py`** - Learned how to publish messages (one-way communication)
3. **`pose_subscriber.py`** - Learned how to subscribe to messages (receiving data)
4. **`bundle.py`** - Combined both concepts (bidirectional communication in one node)

---

## Dependencies

All scripts use:
- `rclpy` - ROS2 Python client library
- `geometry_msgs.msg.Twist` - For velocity commands (publisher)
- `turtlesim.msg.Pose` - For pose information (subscriber)

---

## Notes for Future Me

- These scripts are designed to work with turtlesim, which is a great tool for learning ROS2
- The velocity values in `draw_circle.py` and `bundle.py` (linear.x=2.0, angular.z=1.0) create a circular motion
- The timer interval of 0.5 seconds controls how often velocity commands are published
- All nodes follow the same basic structure: class inheriting from Node, __init__, main function, and spin

