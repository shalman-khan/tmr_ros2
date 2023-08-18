import xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# TODO: find file
doc = xacro.parse(open("/home/rosi/workspaces/techman_robot/src/tmr_ros2/tm_gazebo_cf/xacro/moveit-tm12.urdf.xacro"))
xacro.process_doc(doc)

robot_description = {'robot_description': doc.toxml()}

def generate_launch_description():


    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
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

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity
    ])