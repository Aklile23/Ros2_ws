# CountUntil Action - Client and Server

This package contains my ROS2 action client and server implementations for a counting action. I created these scripts to learn and practice ROS2 actions, which are useful for long-running tasks that can provide feedback and be canceled.

## Overview

I implemented a simple counting action where:
- The **server** counts from 1 up to a target number, with a specified delay (period) between each count
- The **client** sends a goal to the server, receives feedback during execution, and can cancel the goal if needed

The action uses the `CountUntil` action definition from the `my_robot_interfaces` package, which includes:
- **Goal**: `target_number` (int64) and `period` (float64 seconds)
- **Result**: `reached_number` (int64) - the number reached when the action completes or is canceled
- **Feedback**: `current_number` (int64) - the current count value published during execution

## Files

### `count_until_server.py`
This is my action server implementation. It:
- Creates an action server that listens on the `count_until` action topic
- Accepts goals with a target number and period
- Counts from 1 to the target number, sleeping for the specified period between counts
- Publishes feedback with the current number after each count
- Handles goal cancellation gracefully - if canceled, it stops counting and returns the number it reached
- Returns the final reached number as the result

### `count_until_client.py`
This is my action client implementation. It:
- Creates an action client that connects to the `count_until` action server
- Sends a goal with `target_number=10` and `period=1.0` seconds
- Receives and logs feedback messages showing the current count
- Demonstrates goal cancellation by canceling the goal after 3 seconds
- Receives and logs the final result showing the reached number

## How to Use

### Prerequisites
Make sure you have:
1. Built the workspace (including `my_robot_interfaces` package which defines the action)
2. Sourced your ROS2 workspace setup file

### Running the Server

In one terminal, start the action server:
```bash
ros2 run actions_py count_until_server
```

You should see:
```
[INFO] [count_until_server]: Action server started
```

### Running the Client

In another terminal, run the client:
```bash
ros2 run actions_py count_until_client
```

The client will:
1. Send a goal to count to 10 with 1 second period
2. Wait 3 seconds
3. Cancel the goal
4. Display feedback messages and the final result

### Expected Output

**Server terminal** will show:
```
[INFO] [count_until_server]: Executing goal
[INFO] [count_until_server]: 1
[INFO] [count_until_server]: 2
[INFO] [count_until_server]: 3
[INFO] [count_until_server]: Goal canceled
```

**Client terminal** will show:
```
[INFO] [count_until_client]: Sending goal
[INFO] [count_until_client]: Feedback: current_number = 1
[INFO] [count_until_client]: Feedback: current_number = 2
[INFO] [count_until_client]: Feedback: current_number = 3
[INFO] [count_until_client]: Requesting goal cancellation
[INFO] [count_until_client]: Result: 3
```

## Customization

To modify the goal parameters in the client, edit the `main()` function in `count_until_client.py`:
```python
node.send_goal(10, 1.0)  # Change target_number and period here
```

You can also modify the cancellation timing:
```python
time.sleep(3.0)  # Change how long to wait before canceling
```

## Notes

- The server handles cancellation gracefully - if a goal is canceled, it stops counting and returns the number it reached
- Feedback is published after each count increment
- The action server can handle multiple goals sequentially (one at a time)
- I used async callbacks for the client to handle goal responses, feedback, and results

