from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess, GroupAction, IncludeLaunchDescription
import xacro
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("onix_description"), "urdf", "onix.urdf.xacro"]
            ),
            " ",
            "name:=onix",
            " ",
            "prefix:=''",
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    config_onix_velocity_controller = PathJoinSubstitution(
        [FindPackageShare("onix_control"),
        "config",
        "control_ar100.yaml"],
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    node_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, config_onix_velocity_controller],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )
    spawn_onix_velocity_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["onix_velocity_controller"],
        output="screen",
    )

    spawn_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    avr_manager = Node(
        package="avr_driver",
        executable="avr_node",
        output="screen",
    )
    lidar_obstacle_manager = Node(
        package="lidar_obstacle",
        executable="obstacle_detection",
        output="screen",
    )


    twist_mux = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('twist_mux'), 'launch', 'twist_mux_launch.py'],
            ))
        )
    lidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('lslidar_driver'), 'launch', 'lslidar_launch.py'],
            ))
        )
    fast_pub_manager = Node(
        package="follower",
        executable="fast_pub",
        output="screen",
    )


    ld = LaunchDescription()
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_controller_manager)
    ld.add_action(spawn_controller)
    ld.add_action(spawn_onix_velocity_controller)
    #ld.add_action(avr_manager)
    ld.add_action(twist_mux)
    # ld.add_action(fast_pub_manager)
    ld.add_action(lidar_launch)
    ld.add_action(lidar_obstacle_manager)
    return ld
