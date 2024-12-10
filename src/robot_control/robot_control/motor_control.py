import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Pose
from dynamixel_sdk_custom_interfaces.msg import SetPositions
from collections import deque
import numpy as np
import time
from time import sleep

# all link lengths in mm
l1 = 0.08128
l2 = 0.08125
l3 = 0.08128
l4 = 0.08125
l5 = 0.3371

motor_pose = [0, 0, 0]


class MotorController(Node):
    def __init__(self):
        # Initialize the class constructor
        super().__init__('motor_control')

        # Subscribe to the Sigma.7 position topic
        self.subscription = self.create_subscription(
            Pose,
            '/robot/feedback/gripper_pose',
            self.robot_pose_callback,
            10
        )

        self.subscription = self.create_subscription(
            SetPositions,
            '/curr_motor_positions',
            self.curr_motor_position_callback,
            10
        )

        self.last_update_time = time.time()

        # Publish motor IDs and positions
        self.position_publisher = self.create_publisher(SetPositions, '/set_positions', 10)
        self.jacobian = np.zeros((3,3))
        self.robot_vel = np.zeros((3,1))
        self.curr_robot_position = [0,0,0]
        self.motor_pose = [0, 0, 0]
        self.in_range = True
        self.in_workspace = True
        self.last_valid_motor_position = np.array([0.0, 0.0, 0.0])

        
        ### TUNABLES ####
        self.kP = 0.01
        self.error_multiplier = 0.01
        self.error_threshold = np.array([0.01, 0.01, 0.01])

        timer_period = 0.001 #time to publish position
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Define the moving average window size
        self.window_size = 10  # Adjust based on desired smoothing level
        self.motor_position_buffers = {
            2: deque(maxlen=self.window_size),
            4: deque(maxlen=self.window_size),
            5: deque(maxlen=self.window_size)
        }
        
    def curr_motor_position_callback(self, data):
        # to update self.motor_pose with the received motor positions
        for i, motor_id in enumerate([2, 4, 5]):
            try:
                index = data.ids.index(motor_id)
                self.motor_pose[i] = data.positions[index] #receives motor_pose in radian
            except ValueError:
                # If the motor_id is not in the received ids, it keeps the existing value
                self.get_logger().warn(f'Motor ID {motor_id} not found in the message')

    def timer_callback(self):
        # motor_position = self.get_motor_position(self.jacobian, self.robot_vel, self.curr_robot_position)
        # self.publish_motor_positions(motor_position*(180/math.pi))
        #self.get_logger().info(f'Published motor pos')
        return 0

    def robot_pose_callback(self, robot_data):
    # Extract the goal robot position from incoming data
        goal_robot_position = np.array([
            robot_data.orientation.z,  # pitch
            robot_data.orientation.y,  # yaw
            robot_data.position.z      # z translation
        ])

        # Check if the robot is within the workspace
        in_range = self.in_robot_workspace(goal_robot_position)

        if not in_range:
            # Outside workspace, log the state
            if self.in_workspace:
                self.get_logger().warn("Robot moved outside workspace, halting updates.")
                self.in_workspace = False
                #rclpy.sleep(0.5)


            # Exit early to stop motor calculations
            return

        # Re-entering workspace
        if not self.in_workspace:
            self.get_logger().info("Robot re-entered workspace.")
            self.in_workspace = True
            # Reset to the last valid motor positions
            self.curr_motor_position = self.last_valid_motor_position

        # Calculate motor positions
        self.curr_robot_position = self.get_robot_position(self.motor_pose)
        position_error = self.get_position_error(self.curr_robot_position, goal_robot_position)
        self.robot_vel = np.dot(self.kP, position_error)
        self.jacobian = self.get_jacobian(self.motor_pose)
        motor_position = self.get_motor_position(self.jacobian, self.robot_vel, self.curr_robot_position)

        # Save the valid motor positions
        self.last_valid_motor_position = motor_position

        # Publish motor positions
        self.publish_motor_positions(motor_position)
        # # Extract x, y, z values from the incoming data
        # x = robot_data.position.x
        # y = robot_data.position.y
        # z = robot_data.position.z
        # #roll = robot_data.orientation.x
        # pitch = robot_data.orientation.z
        # yaw = robot_data.orientation.y

        # goal_robot_position = np.array([pitch,yaw,z])
        # self.in_robot_workspace(goal_robot_position)

        # if (self.in_range == True):
        #     self.curr_robot_position = np.array(self.get_robot_position(self.motor_pose))

        #     position_error = self.get_position_error(self.curr_robot_position, goal_robot_position)
        #     self.robot_vel = np.dot(self.kP,position_error)
        #     self.jacobian = self.get_jacobian(self.motor_pose)

        #     motor_position = self.get_motor_position(self.jacobian, self.robot_vel, self.curr_robot_position)
        #     self.publish_motor_positions(motor_position)
        # else:
        #     self.get_logger().info(f'Goal outside workspace')
    def in_robot_workspace(self, goal):
        if goal[1] < -0.35 or goal[1] > 0.35:  # Yaw range
            return False
        elif goal[2] > 0.01 or goal[2] < -0.1:  # Z translation range
            return False
        return True

    def publish_motor_positions(self, motor_position):
        # Add new motor positions to the respective buffers
        self.motor_position_buffers[2].append(motor_position[0])
        self.motor_position_buffers[4].append(motor_position[2])
        self.motor_position_buffers[5].append(motor_position[1])

        # Calculate the smoothed positions by taking the average of the buffers
        smoothed_positions = [
            sum(self.motor_position_buffers[2]) / len(self.motor_position_buffers[2]),
            sum(self.motor_position_buffers[4]) / len(self.motor_position_buffers[4]),
            sum(self.motor_position_buffers[5]) / len(self.motor_position_buffers[5])
        ]

        # for i, pos in enumerate(smoothed_positions):
        #     if pos > 360: 
        #         motor_id = [2, 4, 5][i]
        #         self.get_logger().warn(f'Motor ID {motor_id} position exceeded 360: {pos}')

        pos = SetPositions()
        pos.ids = [2, 4, 5]
        #pos.positions = [motor_position[0], motor_position[1], motor_position[2]]
        pos.positions = smoothed_positions
        self.position_publisher.publish(pos)

        # Log published values for each motor
        # self.get_logger().info(f'Motor 2 position: {motor_command.data[0]}')
        # self.get_logger().info(f'Motor 4 position: {motor_command.data[1]}')
        # self.get_logger().info(f'Motor 5 position: {yaw}')

    # forward kinematics
    def get_robot_position(self, motor_pos):
        q1 = motor_pos[0]
        q2 = motor_pos[1]
        q6 = motor_pos[2]
        #theta = math.atan2(((l1*math.sin(q1))+(l2*math.sin(q1-q2))),((l1*math.cos(q1))+(l2*math.cos(q1-q2))))

        # robot_x = (((l1+l3)*math.cos(q1))+((l2+l4)*math.cos(q1-q2))-l5*math.cos(theta))*math.cos(q6)
        # robot_y = (((l1+l3)*math.cos(q1))+((l2+l4)*math.cos(q1-q2))-l5*math.cos(theta))*math.sin(q6)
        #robot_z = ((l1+l3)*math.cos(q1))+((l2+l4)*math.cos(q1-q2))-(l5*math.sin(theta))

        robot_pitch = math.atan2((((l1+l3)*math.sin(q1))+((l2+l4)*math.sin(q2))),(((l1+l3)*math.cos(q1))+((l2+l4)*math.cos(q2))))
        robot_yaw = l5 - math.sqrt((((l1+l3)*math.sin(q1))+((l2+l4)*math.sin(q2)))**2+(((l1+l3)*math.cos(q1))+((l2+l4)*math.cos(q2)))**2)
        robot_z = q6

        #self.get_logger().info(f'FK robot pos: {[robot_pitch, robot_yaw, robot_z]}')

        return [robot_pitch, robot_yaw, robot_z]
    
    #pose error
    def get_position_error(self, curr, goal):

        #self.get_logger().info(f'curr_motor_pos: {curr}')
        #self.get_logger().info(f'goal_robot_pos:{goal}')
        position_error = np.array([goal[0]-curr[0],
                                   goal[1]-curr[1],
                                   goal[2]-curr[2]])
        # if np.linalg.norm(position_error.all) < self.error_threshold.all:
        position_error *= self.error_multiplier
        #self.get_logger().info(f'error: {position_error}')
        return position_error

    # def in_robot_workspace(self, goal):
    #     # self.get_logger().info(f'Goal: {goal}')
    #     self.in_range = True
    #     #if(goal[0] < 0):goal[0]==0 # pitch range is 180 deg which is range of controller

    #     if(goal[1] < -0.35 or goal[1] > 0.35): # yaw range is -20 to 20 degrees
    #         self.in_range = False
            
    #     elif(goal[2] >= 0.01 or goal[2] <= -0.1): # z translation 10 cm down
    #             self.in_range = False
        
    #     return self.in_range
        
    def get_jacobian(self, motor_pos):
        q1 = motor_pos[0]
        q2 = motor_pos[1]
        q6 = motor_pos[2]

        a = (l1+l3)*(l2+l4)*math.sin(q2-q1)
        b = -(l1+l3)*(l2+l4)*math.sin(q1-q2)
        c = ((l1+l3)*(l2+l4)*math.cos(q1-q2))+(l1+l3)**2
        d = ((l1+l3)*(l2+l4)*math.cos(q1-q2))+(l2+l4)**2
        e = (((l1+l3)*math.cos(q1))+((l2+l4)*math.cos(q2)))**2+(((l1+l3)*math.sin(q1))+(l2+l4)*math.sin(q2))**2

        jacobian = np.array([[a/math.sqrt(e), b/math.sqrt(e), 0],
                             [c/math.sqrt(e), d/math.sqrt(e), 0],
                             [0, 0, 1]])
        return jacobian

    # inverse kinematics
    def get_motor_position(self, jacobian, robot_vel, curr_motor_position):
        # Compute motor velocities using the pseudoinverse of the Jacobian
        motor_vel = np.dot(np.linalg.pinv(jacobian), robot_vel).reshape(-1)

        # Custom motor velocity logic
        # Motor 2 and 4 control pitch and translation
        pitch = motor_vel[0]  # Extract pitch-related velocity
        translation = motor_vel[2]  # Extract translation-related velocity
        yaw = motor_vel[1]  # Yaw is independent (motor 5)

        # Motors 2 and 4 behavior
        motor_2_vel = pitch + translation  # Motor 2 adds both components
        motor_4_vel = pitch - translation  # Motor 4 subtracts translation

        # Scale motor 2 to spin faster than motor 4 for pitch + translation
        scaling_factor = 1  # Adjust this value based on the desired speed ratio
        motor_2_vel *= scaling_factor

        # Yaw (motor 5)
        motor_5_vel = yaw

        # Combine velocities into a single array
        motor_velocities = np.array([motor_2_vel, motor_4_vel, motor_5_vel])

        # Calculate time difference
        current_time = time.time()
        delta_t = current_time - self.last_update_time
        self.last_update_time = current_time

        # Euler integration to estimate new motor positions
        new_position_increment = motor_velocities * delta_t  # Element-wise scaling
        new_motor_positions = curr_motor_position + new_position_increment

        # # Check if motor positions are within limits
        # motor_limits = [6.19592, 6.19592, 6.19592]  # Replace with actual joint limits (in radians)
        # for i, pos in enumerate(new_motor_positions):
        #     if pos > motor_limits[i]:
        #         self.get_logger().warn(f'Motor {i+1} exceeds limit: {pos} (capping to {motor_limits[i]})')
        #         new_motor_positions[i] = motor_limits[i]
        #     elif pos < -motor_limits[i]:
        #         self.get_logger().warn(f'Motor {i+1} below -limit: {pos} (capping to {-motor_limits[i]})')
        #         new_motor_positions[i] = -motor_limits[i]

        deg_new_motor_positions = [x * (180/math.pi) for x in new_motor_positions]
        int_new_motor_positions = [int(x) for x in deg_new_motor_positions]

        return int_new_motor_positions



# The main function should be outside the class
def main(args=None):
    rclpy.init(args=args)
    motor_controller_node = MotorController()
    
    # Keep the node running
    rclpy.spin(motor_controller_node)
    
    # Cleanup
    motor_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# for yaw, the "zero" position in the motor should have the robot completely tilted to one side


#linear bearing
#control motor speeds
#workspace is a cone workspace
#clockwise or anticlockwise creates pitch 
# if you turn in opposite direction same speed, creates translation
# if one motor is faster than another, then pitch and translation
# can be achieved.

#for calculating workspace: 
    # 1. calculate using kinematics and mechanism analyis
