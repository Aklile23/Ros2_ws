# Services Folder - ROS2 Service Communication Examples

I created this folder to practice ROS2 service-based communication. All scripts in this folder demonstrate how to create service servers and clients using the request/response pattern.

## Overview

This folder contains Python scripts that I wrote to learn about:
- Creating ROS2 service servers that handle requests
- Creating ROS2 service clients that make requests
- Understanding the request/response pattern (different from topics which are one-way)
- Working with the `AddTwoInts` service from `example_interfaces`
- Service discovery and waiting for services to be available

---

## Scripts

### 1. `add_two_ints_server.py`

**What I did:** I created a service server that receives two integers and returns their sum.

**What it does:**
- Creates a service server named `add_two_ints_server`
- Advertises a service called `add_two_ints` using the `AddTwoInts` service type
- When a client sends a request with two integers (`a` and `b`), it calculates the sum
- Returns the sum in the response
- Logs the calculation (e.g., "3 + 8 = 11")

**Key concepts I learned:**
- How to create a service server using `create_service()`
- How to define a callback function that handles service requests
- Understanding service request and response objects
- The service callback receives both `request` and `response` objects
- How to set response values and return the response
- Services use a request/response pattern (synchronous communication)

**Service details:**
- Service name: `add_two_ints`
- Service type: `example_interfaces.srv.AddTwoInts`
- Request: Two integers (`a` and `b`)
- Response: One integer (`sum`)

**How to run:**
```bash
ros2 run my_robot_controller <executable_name>
```

**What you'll see:** The server starts and waits for client requests. It will log "Add two ints server has been started" and then wait.

---

### 2. `add_two_ints_client_no_oop.py`

**What I did:** I created a service client that calls the `add_two_ints` service to add two numbers.

**Note:** Despite the name saying "no_oop", this script actually uses object-oriented programming (it inherits from `Node`). The name might refer to a simpler implementation style compared to other versions.

**What it does:**
- Creates a service client named `add_two_ints_client`
- Creates a client for the `add_two_ints` service
- Waits for the server to become available (service discovery)
- Creates a request with two integers: `a = 3` and `b = 8`
- Sends the request asynchronously to the server
- Waits for the response using `spin_until_future_complete()`
- Logs the result (the sum)

**Key concepts I learned:**
- How to create a service client using `create_client()`
- How to wait for a service to be available using `wait_for_service()`
- How to create a service request object
- How to call a service asynchronously using `call_async()`
- How to wait for the response using `spin_until_future_complete()`
- How to access the result from the future object
- Understanding that clients need to wait for servers to be available

**How to run:**
```bash
# Terminal 1: Start the server first
ros2 run my_robot_controller <server_executable_name>

# Terminal 2: Run the client
ros2 run my_robot_controller <client_executable_name>
```

**What you'll see:**
- Client terminal: "Waiting for the server to be available..." (if server isn't running yet)
- Client terminal: "Server is available" (once server is found)
- Client terminal: "11" (the result: 3 + 8)
- Server terminal: "3 + 8 = 11" (the server logs the calculation)

---

## How Services Work

I learned that ROS2 services are different from topics:

**Topics (Publisher/Subscriber):**
- One-way communication
- Continuous data stream
- Multiple subscribers can receive the same message
- Fire-and-forget (no response expected)

**Services (Client/Server):**
- Two-way communication (request/response)
- One request = one response
- Synchronous or asynchronous calls
- Client waits for the response
- One server can handle multiple clients, but each request gets its own response

---

## Complete Example Workflow

Here's how I use both scripts together:

1. **Start the server:**
   ```bash
   ros2 run my_robot_controller add_two_ints_server
   ```
   - Server starts and waits for requests
   - Logs: "Add two ints server has been started"

2. **Start the client:**
   ```bash
   ros2 run my_robot_controller add_two_ints_client_no_oop
   ```
   - Client looks for the server
   - Once found, sends request with `a=3, b=8`
   - Server calculates: 3 + 8 = 11
   - Server logs: "3 + 8 = 11"
   - Client receives response and logs: "11"
   - Client shuts down after receiving the response

---

## Dependencies

Both scripts use:
- `rclpy` - ROS2 Python client library
- `example_interfaces.srv.AddTwoInts` - Standard ROS2 example service interface

The `AddTwoInts` service is a built-in example service that:
- Takes two integers as input (`a` and `b`)
- Returns their sum (`sum`)

---

## Key Differences from Topics

When I was learning, I noticed these important differences:

1. **Service Discovery:** Clients must wait for servers to be available (using `wait_for_service()`)
2. **Request/Response:** Services always have a request and response, unlike topics which are one-way
3. **Synchronous Nature:** Even with async calls, you typically wait for the response
4. **One-to-One:** Each service request gets exactly one response
5. **Callback Structure:** Service callbacks receive both request and response objects

---

## Notes for Future Me

- The client script is named "no_oop" but it actually uses OOP (inherits from Node)
- The client uses hardcoded values (3 and 8) - I could modify it to accept command-line arguments
- The client shuts down immediately after getting the response - this is a one-time call pattern
- For continuous service calls, I would need to add a loop or timer
- Services are great for request/response operations like calculations, database queries, or command execution
- The server runs indefinitely (using `rclpy.spin()`) and can handle multiple client requests

