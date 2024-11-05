#include <cstdio>
#include <memory>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_positions.hpp"  // Updated to include SetPositions for multiple motors
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

//#include "dynamixel_controller.hpp"

#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132

#define PROTOCOL_VERSION 2.0
#define BAUDRATE 57600
#define DEVICE_NAME "/dev/ttyUSB0"

#define TICKS_PER_REVOLUTION 4096  // Conversion factor for degrees to ticks

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;
std::shared_ptr<dynamixel::GroupSyncWrite> groupSyncWrite;  // For simultaneous motor control

uint8_t dxl_error = 0;
int dxl_comm_result = COMM_TX_FAIL;

class DynamixelController : public rclcpp::Node
{
public:
  DynamixelController()
  : Node("dynamixel_controller")
  {
    RCLCPP_INFO(this->get_logger(), "Control dynamixel motors simultaneously");

    this->declare_parameter("qos_depth", 10);
    int8_t qos_depth = 0;
    this->get_parameter("qos_depth", qos_depth);

    const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

    // Subscriber for multi-motor positions
    set_positions_subscriber_ = this->create_subscription<dynamixel_sdk_custom_interfaces::msg::SetPositions>(
      "set_positions",
      QOS_RKL10V,
      [this](const dynamixel_sdk_custom_interfaces::msg::SetPositions::SharedPtr msg) -> void
      {
        groupSyncWrite->clearParam();  // Clear previous parameters

        for (size_t i = 0; i < msg->ids.size(); ++i) {
          uint8_t motor_id = msg->ids[i];
          double position_degrees = msg->positions[i];
          uint32_t goal_position = degrees_to_ticks(position_degrees);

          // Convert goal_position to byte array
          uint8_t param_goal_position[4];
          param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(goal_position));
          param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(goal_position));
          param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(goal_position));
          param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(goal_position));

          // Add each motor's goal position to sync write packet
          if (!groupSyncWrite->addParam(motor_id, param_goal_position)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to add parameter for motor ID: %d", motor_id);
          }
        }

        // Send sync write packet
        dxl_comm_result = groupSyncWrite->txPacket();
        if (dxl_comm_result != COMM_SUCCESS) {
          RCLCPP_ERROR(this->get_logger(), "Failed to send sync write packet: %s", packetHandler->getTxRxResult(dxl_comm_result));
        } else {
          RCLCPP_INFO(this->get_logger(), "Successfully sent goal positions to motors");
        }
      }
    );
  }

private:
  rclcpp::Subscription<dynamixel_sdk_custom_interfaces::msg::SetPositions>::SharedPtr set_positions_subscriber_;

  uint32_t degrees_to_ticks(double degrees) {
    return static_cast<uint32_t>((degrees / 360.0) * TICKS_PER_REVOLUTION);
  }
};

void setupDynamixel(uint8_t dxl_id)
{
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, 3, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set Position Control Mode.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set Position Control Mode.");
  }

  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to enable torque.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to enable torque.");
  }
}

int main(int argc, char * argv[])
{
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  groupSyncWrite = std::make_shared<dynamixel::GroupSyncWrite>(portHandler, packetHandler, ADDR_GOAL_POSITION, 4);

  dxl_comm_result = portHandler->openPort();
  if (!dxl_comm_result) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to open the port!");
    return -1;
  }
  RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to open the port.");

  dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
  if (!dxl_comm_result) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set the baudrate!");
    return -1;
  }
  RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set the baudrate.");

  setupDynamixel(BROADCAST_ID);

  rclcpp::init(argc, argv);
  auto dynamixel_controller = std::make_shared<DynamixelController>();
  rclcpp::spin(dynamixel_controller);
  rclcpp::shutdown();

  packetHandler->write1ByteTxRx(portHandler, BROADCAST_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);

  return 0;
}