#include <memory>
#include <chrono>
#include <string>
#include <vector>
#include <cmath>


#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_positions.hpp"  // Custom message to hold multiple motor positions
#include "dynamixel_sdk/dynamixel_sdk.h"

#define ADDR_PRESENT_POSITION 132
#define PROTOCOL_VERSION 2.0
#define BAUDRATE 57600
#define DEVICE_NAME "/dev/ttyUSB0"
#define TICKS_PER_REVOLUTION 4096  // Conversion factor for degrees to ticks

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;

// Motor IDs to publish
const std::vector<int> motor_ids = {2, 4, 5};

class DynamixelPublisher : public rclcpp::Node
{
public:
  DynamixelPublisher()
  : Node("dynamixel_position_publisher")
  {
    // Publisher to send multiple motor positions
    motor_positions_publisher_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPositions>("curr_motor_positions", 10);

    // Set up a timer to periodically read motor positions
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),  // Query every 100 ms
      std::bind(&DynamixelPublisher::read_motor_positions, this)
    );
  }

private:
  rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPositions>::SharedPtr motor_positions_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  void read_motor_positions()
  {
    dynamixel_sdk_custom_interfaces::msg::SetPositions msg;
    msg.ids = motor_ids;

    for (auto motor_id : motor_ids) {
      uint32_t present_position = 0;
      uint8_t dxl_error = 0;
      int dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, motor_id, ADDR_PRESENT_POSITION, &present_position, &dxl_error);
      
      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Failed to read position for motor ID: %d", motor_id);
        msg.positions.push_back(0.0);  // Insert a default value if read fails
      } else {
        double ticks = ticks_to_deg(present_position);
        // double ticks = present_position;

        msg.positions.push_back(ticks);

        RCLCPP_INFO(this->get_logger(), "Motor ID: %d, Present Position: %.2f rad", motor_id, ticks);
      }
    }

    // Publish all motor positions at once
    motor_positions_publisher_->publish(msg);
  }

  double ticks_to_rad(uint32_t ticks)
  {
    return static_cast<double>(ticks) / TICKS_PER_REVOLUTION * (2*M_PI);
  }

  double ticks_to_deg(uint32_t ticks)
  {
    return static_cast<double>(ticks) / TICKS_PER_REVOLUTION * (360.0);
  }
};

int main(int argc, char * argv[])
{
  // Initialize Dynamixel communication
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (!portHandler->openPort()) {
    RCLCPP_ERROR(rclcpp::get_logger("dynamixel_position_publisher"), "Failed to open the port!");
    return -1;
  }
  RCLCPP_INFO(rclcpp::get_logger("dynamixel_position_publisher"), "Succeeded to open the port.");

  if (!portHandler->setBaudRate(BAUDRATE)) {
    RCLCPP_ERROR(rclcpp::get_logger("dynamixel_position_publisher"), "Failed to set the baudrate!");
    return -1;
  }
  RCLCPP_INFO(rclcpp::get_logger("dynamixel_position_publisher"), "Succeeded to set the baudrate.");

  rclcpp::init(argc, argv);
  auto dynamixel_publisher = std::make_shared<DynamixelPublisher>();

  rclcpp::spin(dynamixel_publisher);
  rclcpp::shutdown();

  // Clean up Dynamixel communication before exiting
  portHandler->closePort();
  return 0;
}
