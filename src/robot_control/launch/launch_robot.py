import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess

from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([        
        
        # Node(package='force_dimension', executable='node'),
    
        # Node(package='force_dimension', executable='position_sub'),
    
        # Node(package='force_dimension', executable='pose_sub.py'),


        # Node(
        #     package='force_dimension',            # The package containing the Python node
        #     executable='pose_sub',            # The executable for the Python node
        #     name='pose_client',                  # The name of the node
        #     output='screen',               # Output to screen
        #     # parameters=[{'param_name': 'value'}],  # Optional parameters for node2
        # ),

    #     Node(
    #         package='dynamixel_control',            # The package containing the C++ node
    #         executable='curr_motor_pos',            # The executable for the C++ node
    #         name='dynamixel_position_publisher',                  # The name of the node
    #         output='screen',               # Output to screen
    #         # parameters=[{'param_name': 'value'}],  # Optional parameters for node1
    #     ),

    #    Node(
    #         package='robot_control',            # The package containing the Python node
    #         executable='motor_control',            # The executable for the Python node
    #         name='motor_control',                  # The name of the node
    #         output='screen',               # Output to screen
    #         # parameters=[{'param_name': 'value'}],  # Optional parameters for node2
    #     ),

    #     Node(
    #         package='dynamixel_control',            # The package containing the C++ node
    #         executable='motor_subscriber',            # The executable for the C++ node
    #         name='dynamixel_controller',                  # The name of the node
    #         output='screen',               # Output to screen
    #         # parameters=[{'param_name': 'value'}],  # Optional parameters for node1
    #     ),
        # You can continue to add nodes from other packages here
        # Node(
        #     package='package3',          # Another package
        #     executable='node3',          # Executable for node3
        #     name='node3',
        #     output='screen',
        # ),
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'ros2', 'run', 'force_dimension', 'node'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'ros2', 'run', 'force_dimension', 'position_sub'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'ros2', 'run', 'force_dimension', 'pose_sub.py'],
            output='screen'
        ),  
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'ros2', 'run', 'dynamixel_control', 'curr_motor_pos'],
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'ros2', 'run', 'robot_control', 'motor_control'],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'ros2', 'run', 'dynamixel_control', 'motor_subscriber'],
            output='screen'
        ),
])
