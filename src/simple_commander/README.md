# Nav2 Simple Commander Scripts

Notes for myself when I come back to revise these scripts.

Both scripts use the `nav2_simple_commander` package to control robot navigation. They tell the robot where to go in a map using coordinates and orientations.

## nav2_test.py - Simple Single Goal Navigation

This script sends the robot to a single goal location.

### Key Technical Details

**Creating a PoseStamped message:**

A `PoseStamped` message has two main parts:
- **Header**: Contains `frame_id` and `stamp`
  - `frame_id = 'map'`: This tells ROS2 that the coordinates are in the map frame. The map frame is the global coordinate system of the environment. All poses must be in the same frame for Nav2 to work correctly.
  - `stamp = nav.get_clock().now().to_msg()`: This is the timestamp when the pose was created. Nav2 uses this to know if the pose information is current. I get the current time from the navigator's clock and convert it to a ROS2 time message.

- **Pose**: Contains position and orientation
  - `position.x`, `position.y`, `position.z`: The 3D coordinates. For 2D navigation, z is always 0.0.
  - `orientation.x`, `orientation.y`, `orientation.z`, `orientation.w`: The quaternion representing rotation.

**Quaternion conversion:**

ROS2 uses quaternions (4 numbers: qx, qy, qz, qw) to represent 3D rotations, but I find it easier to think in Euler angles (roll, pitch, yaw). The conversion is done with:
```python
qx, qy, qz, qw = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)
```

The three parameters are (roll, pitch, yaw) in radians:
- Roll: rotation around x-axis (forward/backward tilt) - always 0.0 for 2D navigation
- Pitch: rotation around y-axis (left/right tilt) - always 0.0 for 2D navigation  
- Yaw: rotation around z-axis (turning left/right) - this is the only one that matters for 2D navigation

For the initial pose, I use (0.0, 0.0, 0.0) which means no rotation. For the goal pose, I use (0.0, 0.0, 1.57) where 1.57 radians = 90 degrees, so the robot will face 90 degrees (pointing along the positive y-axis).

**setInitialPose() vs goToPose():**

- `setInitialPose()`: This tells Nav2 where the robot currently is in the map. This is critical for localization - Nav2 needs to know the robot's starting position to plan paths correctly. This is NOT a navigation command, just setting the robot's known position.

- `waitUntilNav2Active()`: This blocks until Nav2's action servers are ready. It checks that the planner, controller, and other Nav2 components are up and running. Without this, navigation commands will fail.

- `goToPose()`: This actually sends the navigation command. It's non-blocking - it returns immediately and the robot navigates in the background. I need to check completion manually.

**The feedback loop:**

```python
while not nav.isTaskComplete():
    feedback = nav.getFeedback()
    print(feedback)
```

- `isTaskComplete()`: Returns True when the navigation task is done (either reached goal, failed, or was cancelled). The loop keeps running until this returns True.

- `getFeedback()`: Returns a feedback message from Nav2 that contains information like current position, estimated time remaining, distance to goal, etc. I print it to monitor progress, but I'm not using it for any logic here.

**Important:** The script will hang forever if the robot can't reach the goal. I should add timeout logic or check for failure states in the feedback.

## nav2_test2.py - Waypoint Following

This script sends the robot to multiple waypoints in sequence.

### Key Technical Details

**The create_pose_stamped() helper function:**

This function encapsulates all the pose creation logic to avoid repetition. It takes:
- `navigator`: The BasicNavigator instance (needed to get the clock for timestamps)
- `position_x`, `position_y`: The x and y coordinates in the map frame
- `orientation_z`: The yaw angle in radians (the rotation around z-axis)

Inside the function:
1. Converts the Euler angle to quaternion: `quaternion_from_euler(0.0, 0.0, orientation_z)` - roll and pitch are always 0 for 2D navigation
2. Creates a new `PoseStamped()` message
3. Sets the frame_id to 'map' (all poses must be in the same frame)
4. Sets the timestamp using the navigator's clock
5. Sets position and orientation components
6. Returns the complete pose

**Note:** There's a bug on line 17 - it sets `pose.pose.orientation.z = orientation_z` but it should be `pose.pose.orientation.z = qz` (the quaternion z component, not the Euler angle). The quaternion components are already calculated on line 8, so I should use those.

**followWaypoints() vs goToPose():**

- `goToPose()`: Takes a single `PoseStamped` and navigates to it.

- `followWaypoints()`: Takes a list of `PoseStamped` messages and visits them in order. The robot will:
  1. Navigate to the first waypoint
  2. Once reached, automatically start navigating to the second waypoint
  3. Continue until all waypoints are visited
  4. Only then will `isTaskComplete()` return True

The waypoints are created as:
- Goal 1: (3.5, 1.0) with 1.57 rad rotation (90 degrees)
- Goal 2: (2.0, 2.5) with 3.14 rad rotation (180 degrees) 
- Goal 3: (0.5, 1.0) with -1.57 rad rotation (-90 degrees)

The feedback loop works the same way - it monitors progress through all waypoints until the entire sequence is complete.

## Key Concepts

- **PoseStamped**: A ROS2 message combining a pose (position + orientation) with a header (frame_id + timestamp). The frame_id tells ROS2 which coordinate system the pose is in, and the timestamp ensures the pose information is current.

- **Quaternions**: A 4-number representation (qx, qy, qz, qw) of 3D rotations that avoids gimbal lock. ROS2 requires quaternions, but I convert from Euler angles which are easier to understand. For 2D navigation, only the yaw angle matters.

- **Frame IDs**: The coordinate system reference. 'map' is the global map frame. All poses must use the same frame_id for Nav2 to work. The robot's current position is tracked in the map frame through localization.

- **BasicNavigator**: The main class that wraps Nav2's action clients. It handles communication with Nav2's planning and control nodes, manages the navigation lifecycle, and provides feedback.

## How to Run

1. Make sure ROS2 and Nav2 are installed and running.

2. Make sure the robot's navigation stack is active (map, localization, and planning nodes running).

3. Run the script:
   ```bash
   python3 nav2_test.py
   ```
   or
   ```bash
   python3 nav2_test2.py
   ```

4. The script will print feedback messages showing navigation progress.
