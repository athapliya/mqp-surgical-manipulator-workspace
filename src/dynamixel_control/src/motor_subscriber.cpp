#include <cstdio>
#include <memory>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_positions.hpp"  // Updated to include SetPositions for multiple motors
#include "rclcpp/rclcpp.hpp"

#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132
#define ADDR_VELOCITY_LIMIT 44
#define ADDR_PROFILE_VELOCITY 112


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

    // Compute ticks for each motor position
    for (size_t i = 0; i < msg->ids.size(); ++i) {
      uint8_t motor_id = msg->ids[i];
      double position_degrees = msg->positions[i];
      int32_t goal_position = degrees_to_ticks(position_degrees);

      // Convert goal_position to byte array
      uint8_t param_goal_position[4];
      param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(goal_position));
      param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(goal_position));
      param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(goal_position));
      param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(goal_position));

      RCLCPP_INFO(this->get_logger(),
                  "Motor ID: %d, Goal Position: %d ticks (%.2f degrees)",
                  motor_id, goal_position, position_degrees);

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

    // // Timer for periodically reading motor positions
    // timer_ = this->create_wall_timer(
    //   std::chrono::milliseconds(100),
    //   std::bind(&DynamixelController::read_motor_positions, this)
    // );
  }

private:
  rclcpp::Subscription<dynamixel_sdk_custom_interfaces::msg::SetPositions>::SharedPtr set_positions_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;

uint32_t degrees_to_ticks(double degrees) {
    int32_t ticks = static_cast<int32_t>((degrees / 360.0) * TICKS_PER_REVOLUTION);
    return static_cast<uint32_t>(2048 + ticks);  // Map to valid range
}

double ticks_to_degrees(uint32_t ticks) {
  return static_cast<double>(ticks) / TICKS_PER_REVOLUTION * 360.0;
}


//   void read_motor_positions() {
//     std::vector<uint8_t> motor_ids = {2, 4, 5};
    
//     for (auto motor_id : motor_ids) {
//       uint32_t present_position = 0;
//       dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, motor_id, ADDR_PRESENT_POSITION, &present_position, &dxl_error);
      
//       if (dxl_comm_result != COMM_SUCCESS) {
//         RCLCPP_ERROR(this->get_logger(), "Failed to read present position for motor ID: %d", motor_id);
//       } else {
//         double position_degrees = ticks_to_degrees(present_position);
//         RCLCPP_INFO(this->get_logger(), "Motor ID: %d, Present Position: %.2f degrees", motor_id, position_degrees);
//       }
//     }
//   }
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

void set_motor_velocity(uint8_t motor_id, uint32_t profile_velocity) {
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, motor_id, ADDR_PROFILE_VELOCITY, profile_velocity, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set profile velocity for ID: %d", motor_id);
    } else {
        RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Profile velocity set for ID: %d to %u ticks/s", motor_id, profile_velocity);
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
  set_motor_velocity(2, 50);  
  set_motor_velocity(4, 50);  
  set_motor_velocity(5, 50);  

  rclcpp::init(argc, argv);
  auto dynamixel_controller = std::make_shared<DynamixelController>();
  rclcpp::spin(dynamixel_controller);
  rclcpp::shutdown();

  packetHandler->write1ByteTxRx(portHandler, BROADCAST_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);

  return 0;
}
