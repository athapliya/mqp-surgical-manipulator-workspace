import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from dynamixel_sdk_custom_interfaces.msg import SetPositions


class TTYMotorController(Node):
    def __init__(self):
        super().__init__('tty_motor_controller')

        # One-time subscription to initialize motor positions
        self.init_subscription = self.create_subscription(
            SetPositions,
            '/curr_motor_positions',
            self.curr_motor_position_init_callback,
            10
        )

        # Publisher for motor positions
        self.position_publisher = self.create_publisher(SetPositions, '/set_positions', 10)

        # Current positions
        self.init_curr_robot_position = [0.0, 0.0, 0.0]
        self.curr_robot_position = [0.0, 0.0, 0.0]
        self.initialized = False  # Flag to track initialization

    def curr_motor_position_init_callback(self, data):
        """Callback to initialize motor positions."""
        for i, motor_id in enumerate([2, 4, 5]):
            try:
                index = data.ids.index(motor_id)
                self.curr_robot_position[i] = data.positions[index]-180.0  # Initialize positions
                self.init_curr_robot_position[i] = data.positions[index]-180.0
            except ValueError:
                self.get_logger().warn(f'Motor ID {motor_id} not found in the initial message')

        self.get_logger().info(f"Initialized positions: {self.curr_robot_position}")
        self.initialized = True  # Mark initialization complete

        # Unsubscribe after initialization
        self.destroy_subscription(self.init_subscription)

    def send_positions(self):
        msg = SetPositions()
        msg.ids = [2, 4, 5]
        msg.positions = self.curr_robot_position
        self.position_publisher.publish(msg)
        self.get_logger().info(f"Sent positions: {self.curr_robot_position}")

    def read_key(self):
        """Reads a single key press from the terminal."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)  # Read one character
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key


    def control_loop(self):
        """Control loop to read key presses and send motor positions."""
        self.get_logger().info("Use W/A/S/D for control, ESC to exit.")

        # Wait for initialization
        while not self.initialized and rclpy.ok():
            self.get_logger().info("Waiting for motor position initialization...")
            rclpy.spin_once(self)

        if not rclpy.ok():
            return

        while rclpy.ok():
            key = self.read_key()
            deg_step = 5.0
            if key == '\x1b':  # ESC key
                self.get_logger().info("Exiting...")
                break
            elif key.lower() == 'w':
                # Translation UP
                self.curr_robot_position[0] = self.curr_robot_position[0]+deg_step  
                self.curr_robot_position[1] = self.curr_robot_position[1]-deg_step  

            elif key.lower() == 's':
                # Translation DOWN
                self.curr_robot_position[0] = self.curr_robot_position[0]-deg_step
                self.curr_robot_position[1] = self.curr_robot_position[1]+deg_step 

            elif key.lower() == 'q':
                # Pitch FORWARD
                self.curr_robot_position[0] = self.curr_robot_position[0]-deg_step  
                self.curr_robot_position[1] = self.curr_robot_position[1]-deg_step  

            elif key.lower() == 'e':
                # Pitch BACKWARD
                self.curr_robot_position[0] = self.curr_robot_position[0]+deg_step  
                self.curr_robot_position[1] = self.curr_robot_position[1]+deg_step  

            elif key.lower() == 'a':
                # Yaw LEFT
                self.curr_robot_position[2] = self.curr_robot_position[2]-deg_step

            elif key.lower() == 'd':
                # Yaw RIGHT
                self.curr_robot_position[2] = self.curr_robot_position[2]+deg_step
            else:
                self.get_logger().info(f"Unrecognized key: {key}")
                continue

            if self.in_robot_workspace(self.curr_robot_position):
                self.send_positions()
            else:
                self.get_logger().warn(f"Out of workspace")

    def in_robot_workspace(self, curr_pos):
        # if abs(self.init_curr_robot_position[2]-self.curr_robot_position) > 35:  # Yaw range
        #     return False
        # elif abs(self.init_curr_robot_position[2]-self.curr_robot_position) > 35:  # Z translation range
        #     return False
        return True



def main(args=None):
    rclpy.init(args=args)
    node = TTYMotorController()

    try:
        node.control_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("Shutdown complete.")


if __name__ == '__main__':
    main()
