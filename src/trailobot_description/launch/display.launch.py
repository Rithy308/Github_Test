import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    # pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    # remappings = [('/tf', 'tf'),
    #             ('/tf_static', 'tf_static')]
    # pkg_share = FindPackageShare(package='AMR_description').find('AMR_description')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    urdf_file_name = 'diffbot.xacro'
    urdf = os.path.join(
        get_package_share_directory('trailobot_description'),
        'urdf',
        urdf_file_name)
    rviz_config_dir = os.path.join(get_package_share_directory('trailobot_description'),
                                   'rviz')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    model = LaunchConfiguration('model')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')

    declare_model_path_cmd = DeclareLaunchArgument(
    name='model', 
    default_value=urdf, 
    description='Absolute path to robot urdf file')

    start_joint_state_publisher_cmd = Node(
    # condition=UnlessCondition(gui),
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    remappings=[("/robot_description", "/unicycle_bot_robot_description")]
    )

    return LaunchDescription([
        
    # DeclareLaunchArgument(
    # 'use_sim_time',
    # default_value='true',
    # description='Use simulation (Gazebo) clock if true'),

    start_joint_state_publisher_cmd,
    declare_use_robot_state_pub_cmd,
    declare_model_path_cmd,

        Node(
            condition=IfCondition(use_robot_state_pub),
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'use_sim_time': use_sim_time, 
            'robot_description': Command(['xacro ', model])}],
            # remappings=remappings,
            arguments=[urdf]),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
        # Node(
        #     package='rplidar_ros',
        #     executable='rplidar_composition',
        #     output='screen',
        #     parameters=[{
        #         'serial_port': '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0',
        #         'frame_id': 'laser_frame',
        #         'angle_compensate': True,
        #         'scan_mode': 'Standard'
        #     }]
        # )

    #     IncludeLaunchDescription(
    # PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    # condition=IfCondition(use_simulator),
    # launch_arguments={'world': world}.items()
    # ),
    

])