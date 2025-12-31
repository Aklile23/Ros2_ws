# C++ Source Folder - ROS2 C++ Learning Exercises

I created this folder to learn ROS2 programming in C++. This is where I did basic exercises to understand how to write ROS2 nodes, publishers, subscribers, and services using the C++ API.

## Overview

This folder contains C++ scripts that I wrote to learn:
- Basic ROS2 node creation in C++
- Object-oriented programming with ROS2 nodes
- Publisher/subscriber communication patterns
- Service client/server patterns
- C++ specific ROS2 concepts (smart pointers, templates, callbacks)

---

## Scripts

### 1. `my_first_node.cpp`

**What I did:** I created my very first ROS2 node in C++ using a simple procedural approach.

**What it does:**
- Initializes ROS2 with `rclcpp::init()`
- Creates a node using `std::make_shared<rclcpp::Node>()`
- Logs "Hello ROS2" using `RCLCPP_INFO()`
- Shuts down ROS2

**Key concepts I learned:**
- How to include ROS2 C++ headers: `#include "rclcpp/rclcpp.hpp"`
- ROS2 initialization: `rclcpp::init(argc, argv)`
- Creating nodes: `std::make_shared<rclcpp::Node>("node_name")`
- Logging: `RCLCPP_INFO(node->get_logger(), "message")`
- ROS2 shutdown: `rclcpp::shutdown()`
- Basic procedural approach (not object-oriented)

**How to run:**
```bash
ros2 run my_cpp_pkg hello_node
```

---

### 2. `my_first_nodeOOP.cpp`

**What I did:** I rewrote my first node using object-oriented programming, which is the recommended approach in ROS2 C++.

**What it does:**
- Creates a class `MyNode` that inherits from `rclcpp::Node`
- Uses a timer to periodically print messages
- Demonstrates OOP structure with public/private members
- Shows how to use callbacks with timers

**Key concepts I learned:**

1. **Class Inheritance:**
   ```cpp
   class MyNode : public rclcpp::Node
   ```
   - Inherit from `rclcpp::Node` to create custom node classes
   - This is the standard ROS2 C++ pattern

2. **Constructor Initialization:**
   ```cpp
   MyNode() : Node("hello_node"), counter_(0)
   ```
   - Initialize base class `Node` with node name
   - Initialize member variables in constructor initializer list

3. **Timers:**
   ```cpp
   timer_ = this->create_wall_timer(std::chrono::seconds(1), 
                                  std::bind(&MyNode::on_timer_cb, this));
   ```
   - `create_wall_timer()` creates a periodic timer
   - Use `std::bind()` to bind member functions as callbacks
   - Timers execute callbacks at specified intervals

4. **Member Variables:**
   ```cpp
   rclcpp::TimerBase::SharedPtr timer_;
   int counter_;
   ```
   - Use smart pointers (`SharedPtr`) for ROS2 objects
   - Regular types for simple data

5. **Callbacks:**
   ```cpp
   void on_timer_cb()
   {
       RCLCPP_INFO(this->get_logger(), "Hello %d", counter_);
       counter_++;
   }
   ```
   - Callback functions are called by ROS2
   - Access node methods using `this->`

**How to run:**
```bash
ros2 run my_cpp_pkg hello_node
```

**What you'll see:** The node continuously prints "Hello 0", "Hello 1", "Hello 2", etc., every second.

---

### 3. `publisher.cpp`

**What I did:** I created a publisher node to learn how to publish messages to topics in C++.

**What it does:**
- Creates a publisher for `String` messages on the `/topic` topic
- Publishes robot status messages every 0.5 seconds
- Uses a timer to trigger periodic publishing
- Demonstrates message creation and publishing

**Key concepts I learned:**

1. **Including Message Headers:**
   ```cpp
   #include "example_interfaces/msg/string.hpp"
   ```
   - Include message type headers
   - Message types are in namespace format

2. **Creating Publishers:**
   ```cpp
   publisher_ = this->create_publisher<example_interfaces::msg::String>("topic", 10);
   ```
   - Use `create_publisher<MessageType>()` template function
   - Specify topic name and queue size
   - Store as `SharedPtr` member variable

3. **Creating Messages:**
   ```cpp
   auto message = example_interfaces::msg::String();
   message.data = robot_name_ + " is moving";
   ```
   - Create message objects
   - Set message fields using dot notation
   - Use `auto` for type inference

4. **Publishing Messages:**
   ```cpp
   publisher_->publish(message);
   ```
   - Call `publish()` on the publisher
   - Message is sent to the topic

5. **Chrono Literals:**
   ```cpp
   using namespace std::chrono_literals;
   timer_ = this->create_wall_timer(0.5s, ...);
   ```
   - Use `chrono_literals` for readable time durations
   - `0.5s` is equivalent to `std::chrono::milliseconds(500)`

**How to run:**
```bash
ros2 run my_cpp_pkg publisher
```

**What you'll see:** The node publishes "R1 is moving" to the `/topic` topic every 0.5 seconds.

---

### 4. `subscriber.cpp`

**What I did:** I created a subscriber node to learn how to receive messages from topics in C++.

**What it does:**
- Subscribes to `String` messages on the `/topic` topic
- Receives messages and logs them
- Demonstrates subscription and callback handling

**Key concepts I learned:**

1. **Creating Subscribers:**
   ```cpp
   subscriber_ = this->create_subscription<example_interfaces::msg::String>(
       "topic", 10, std::bind(&RobotNode::subscribe_cb, this, _1));
   ```
   - Use `create_subscription<MessageType>()` template function
   - Specify topic name, queue size, and callback function
   - Use `std::bind()` with `_1` placeholder for message parameter

2. **Placeholders:**
   ```cpp
   using namespace std::placeholders;
   ```
   - `_1` is a placeholder for the first callback argument (the message)
   - Allows binding member functions with parameters

3. **Subscription Callbacks:**
   ```cpp
   void subscribe_cb(const example_interfaces::msg::String::SharedPtr msg)
   {
       RCLCPP_INFO(this->get_logger(), msg->data.c_str());
   }
   ```
   - Callback receives message as `SharedPtr`
   - Access message data using `->` operator
   - Convert C++ strings to C strings with `.c_str()` for logging

4. **Message Pointers:**
   - Messages are passed as shared pointers (`SharedPtr`)
   - Use `->` to access members, not `.`
   - Shared pointers manage memory automatically

**How to run:**
```bash
# Terminal 1: Run publisher
ros2 run my_cpp_pkg publisher

# Terminal 2: Run subscriber
ros2 run my_cpp_pkg subscriber
```

**What you'll see:** The subscriber receives and logs messages from the publisher.

---

### 5. `server.cpp`

**What I did:** I created a service server to learn how to handle service requests in C++.

**What it does:**
- Creates a service server for `AddTwoInts` service
- Handles requests to add two integers
- Returns the sum in the response
- Demonstrates service callback pattern

**Key concepts I learned:**

1. **Including Service Headers:**
   ```cpp
   #include "example_interfaces/srv/add_two_ints.hpp"
   ```
   - Include service type headers
   - Services have Request and Response types

2. **Creating Service Servers:**
   ```cpp
   server_ = this->create_service<example_interfaces::srv::AddTwoInts>(
       "add_two_ints", 
       std::bind(&ServerNode::add_two_ints_cb, this, _1, _2));
   ```
   - Use `create_service<ServiceType>()` template function
   - Specify service name and callback
   - Use `_1` and `_2` placeholders for request and response

3. **Service Callbacks:**
   ```cpp
   void add_two_ints_cb(
       const example_interfaces::srv::AddTwoInts::Request::SharedPtr request, 
       const example_interfaces::srv::AddTwoInts::Response::SharedPtr response)
   {
       response->sum = request->a + request->b;
       RCLCPP_INFO(this->get_logger(), "Sum: %ld", response->sum);
   }
   ```
   - Callback receives both request and response as `SharedPtr`
   - Read from `request->` fields
   - Write to `response->` fields
   - Return response (implicitly)

4. **Service Types:**
   - `ServiceType::Request` - Request message type
   - `ServiceType::Response` - Response message type
   - Both are passed as shared pointers

**How to run:**
```bash
ros2 run my_cpp_pkg server
```

**What you'll see:** The server starts and waits for client requests. It logs "Server has been started" and then waits.

---

### 6. `client.cpp`

**What I did:** I created a service client to learn how to make service requests in C++.

**What it does:**
- Creates a service client for `AddTwoInts` service
- Waits for the server to be available
- Sends a request with two integers (1 and 2)
- Receives and logs the response asynchronously

**Key concepts I learned:**

1. **Creating Service Clients:**
   ```cpp
   client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
   ```
   - Use `create_client<ServiceType>()` template function
   - Specify service name
   - Store as `SharedPtr` member variable

2. **Waiting for Service:**
   ```cpp
   while (!client_->wait_for_service(1s))
   {
       RCLCPP_WARN(this->get_logger(), "Waiting for the server to be available");
   }
   ```
   - `wait_for_service()` checks if server is available
   - Returns `true` when server is ready
   - Use loop to wait until server is available

3. **Creating Requests:**
   ```cpp
   auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
   request->a = a;
   request->b = b;
   ```
   - Create request as `SharedPtr` using `std::make_shared()`
   - Set request fields using `->` operator

4. **Asynchronous Service Calls:**
   ```cpp
   client_->async_send_request(
       request,
       std::bind(&ClientNode::response_callback, this, _1)
   );
   ```
   - `async_send_request()` sends request without blocking
   - Takes request and callback function
   - Callback is called when response is received

5. **Response Callbacks:**
   ```cpp
   void response_callback(rclcpp::Client<...>::SharedFuture future)
   {
       RCLCPP_INFO(this->get_logger(), "Sum: %ld", future.get()->sum);
   }
   ```
   - Callback receives a `SharedFuture` object
   - Call `future.get()` to get the response
   - Access response fields using `->`

6. **Spinning for Async Operations:**
   ```cpp
   rclcpp::spin(node);
   ```
   - Must call `spin()` to process async callbacks
   - Allows the response callback to be executed

**How to run:**
```bash
# Terminal 1: Start server
ros2 run my_cpp_pkg server

# Terminal 2: Run client
ros2 run my_cpp_pkg client
```

**What you'll see:**
- Client terminal: "Waiting for the server to be available" (if server isn't running)
- Client terminal: "Sum: 3" (the result: 1 + 2)
- Server terminal: "Sum: 3" (the server logs the calculation)

---

## C++ vs. Python in ROS2

I learned some key differences between C++ and Python ROS2 programming:

| Aspect | C++ | Python |
|--------|-----|--------|
| **Headers/Imports** | `#include "rclcpp/rclcpp.hpp"` | `import rclpy` |
| **Node Creation** | `std::make_shared<rclcpp::Node>()` | `Node("name")` |
| **Templates** | `create_publisher<Type>()` | `create_publisher(Type, ...)` |
| **Pointers** | Smart pointers (`SharedPtr`) | No pointers needed |
| **Callbacks** | `std::bind()` with placeholders | Direct function reference |
| **Memory** | Manual management (smart pointers) | Automatic (garbage collection) |
| **Performance** | Faster, compiled | Slower, interpreted |
| **Complexity** | More verbose, type-safe | Simpler, more concise |

---

## Key C++ Concepts I Learned

1. **Smart Pointers:**
   - `SharedPtr` - Shared ownership, automatic memory management
   - Used for all ROS2 objects (publishers, subscribers, timers, etc.)
   - No need to manually delete objects

2. **Templates:**
   - ROS2 C++ API uses templates extensively
   - `create_publisher<MessageType>()` - Template parameter specifies message type
   - Provides type safety at compile time

3. **std::bind():**
   - Binds member functions to callbacks
   - Placeholders (`_1`, `_2`) represent callback parameters
   - Essential for C++ callbacks

4. **Chrono:**
   - `std::chrono` for time durations
   - `chrono_literals` for readable syntax (`1s`, `0.5s`)
   - Used in timers and service timeouts

5. **Namespaces:**
   - ROS2 uses namespaces: `rclcpp::`, `example_interfaces::msg::`
   - Can use `using namespace` to simplify code

---

## Building C++ Packages

I learned that C++ packages require:

1. **CMakeLists.txt** - Build configuration
2. **package.xml** - Package dependencies
3. **Build process:**
   ```bash
   cd /home/aklile/ros2_ws
   colcon build --packages-select my_cpp_pkg
   source install/setup.bash
   ```

4. **Executables:**
   - C++ code is compiled into executables
   - Executables are defined in `CMakeLists.txt`
   - Run with: `ros2 run my_cpp_pkg <executable_name>`

---

## Learning Progression

I created these scripts in a learning progression:

1. **`my_first_node.cpp`** - Started with basic procedural node
2. **`my_first_nodeOOP.cpp`** - Learned object-oriented approach with timers
3. **`publisher.cpp`** - Learned to publish messages
4. **`subscriber.cpp`** - Learned to subscribe and receive messages
5. **`server.cpp`** - Learned to handle service requests
6. **`client.cpp`** - Learned to make service calls

Each script builds on previous concepts and introduces new C++ ROS2 patterns.

---

## Dependencies

All scripts use:
- `rclcpp` - ROS2 C++ client library
- `example_interfaces` - Standard ROS2 example message and service types

---

## Notes for Future Me

- **Compilation required:** C++ code must be compiled before running (unlike Python)
- **Type safety:** C++ catches type errors at compile time (advantage over Python)
- **Smart pointers:** Always use `SharedPtr` for ROS2 objects, never raw pointers
- **Templates:** Message/service types are template parameters
- **Callbacks:** Must use `std::bind()` for member function callbacks
- **Placeholders:** `_1` for first parameter, `_2` for second, etc.
- **Spinning:** Must call `rclcpp::spin()` for callbacks to execute
- **Async operations:** Use `async_send_request()` for non-blocking service calls
- **Chrono literals:** Use `using namespace std::chrono_literals` for readable time
- **Memory management:** Smart pointers handle memory automatically (no manual delete)

---

## Next Steps (Ideas for Future Learning)

- Learn about parameters in C++ nodes
- Learn about custom message types in C++
- Learn about lifecycle nodes
- Learn about action servers/clients in C++
- Learn about component nodes
- Learn about C++ best practices and performance optimization
- Learn about using C++ with custom interfaces from `my_robot_interfaces`

