cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(hello_node src/my_first_node.cpp)
ament_target_dependencies(hello_node rclcpp)

add_executable(hello_node_oop src/my_first_nodeOOP.cpp)
ament_target_dependencies(hello_node_oop rclcpp)

add_executable(publisher src/publisher.cpp)
ament_target_dependencies(publisher rclcpp example_interfaces)

add_executable(subscriber src/subscriber.cpp)
ament_target_dependencies(subscriber rclcpp example_interfaces)

add_executable(server src/server.cpp)
ament_target_dependencies(server rclcpp example_interfaces)

add_executable(client src/client.cpp)
ament_target_dependencies(client rclcpp example_interfaces)

install(TARGETS
  hello_node
  hello_node_oop
  publisher
  subscriber
  server
  client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
