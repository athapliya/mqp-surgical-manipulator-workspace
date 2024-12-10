cmake_minimum_required(VERSION 3.8)
project(dynamixel_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_sdk_custom_interfaces REQUIRED) # Ensure custom message package is found
find_package(rosidl_default_generators REQUIRED)

# Include directories
include_directories(include)

# Add the executables
add_executable(motor_subscriber src/motor_subscriber.cpp)
add_executable(curr_motor_pos src/curr_motor_pos.cpp)

# Link dependencies to the motor_subscriber target executable
ament_target_dependencies(
  motor_subscriber 
  rclcpp 
  std_msgs 
  dynamixel_sdk 
  dynamixel_sdk_custom_interfaces
)

# Link dependencies to the curr_motor_pos target executable
ament_target_dependencies(
  curr_motor_pos 
  rclcpp 
  std_msgs 
  dynamixel_sdk 
  dynamixel_sdk_custom_interfaces
)

# Install the executables
install(TARGETS
  motor_subscriber
  curr_motor_pos
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
