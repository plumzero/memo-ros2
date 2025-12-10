cmake_minimum_required(VERSION 3.5)
project(demo3_package)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tutorial_interfaces REQUIRED)

# 添加代码
add_executable(publisher_member_function src/publisher_member_function.cpp)
ament_target_dependencies(publisher_member_function rclcpp tutorial_interfaces)
add_executable(subscriber_member_function src/subscriber_member_function.cpp)
ament_target_dependencies(subscriber_member_function rclcpp tutorial_interfaces)
add_executable(add_three_ints_server src/add_three_ints_server.cpp)
ament_target_dependencies(add_three_ints_server rclcpp tutorial_interfaces)
add_executable(add_three_ints_client src/add_three_ints_client.cpp)
ament_target_dependencies(add_three_ints_client rclcpp tutorial_interfaces)

install(TARGETS
  publisher_member_function
  subscriber_member_function
  add_three_ints_server
  add_three_ints_client
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
