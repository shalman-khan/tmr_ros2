import os
import xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# doc = xacro.parse(open("/home/rosi/workspaces/techman_robot/src/tmr_ros2/tm_gazebo/urdf/tm12x-nominal.urdf"))
# doc = xacro.parse(open("/home/rosi/workspaces/techman_robot/install/tm12_moveit_config/share/tm12_moveit_config/config/tm12.urdf.xacro"))

# ---  Used to have error: no ros2 control tag
doc = xacro.parse(open("/home/rosi/workspaces/techman_robot/src/tmr_ros2/tm_gazebo_cf/xacro/moveit-tm12.urdf.xacro"))
xacro.process_doc(doc)

robot_description = {'robot_description': doc.toxml()}
params = {'robot_description': doc.toxml()}
print(robot_description)


def generate_launch_description():


    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'mybot'],
                        output='screen')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])]
        ),
        launch_arguments={"verbose": "true"}.items(),
    )

    # robot_controllers = os.path.join(
    #   '/home/rosi/workspaces/techman_robot/src/tmr_ros2/tm_gazebo_cf/',
    #   'config/',
    #   'ros2_controllers.yaml'
    # )
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("tm_gazebo_cf"),
            "config",
            "ros2_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    ) 

    joint_state_controller_file = PathJoinSubstitution(
        [
        FindPackageShare("tm_gazebo"),
        "config",
        "joint_state_controller.yaml",
        ]
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=spawn_entity,
        #         on_exit=[robot_controller_spawner],
        #     )
        # ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=robot_controller_spawner,
        #         on_exit=[load_diff_drive_base_controller],
        #     )
        # ),
        gazebo,
        node_robot_state_publisher,
        spawn_entity
        # control_node
    ])