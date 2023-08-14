import os
from moveit_configs_utils import MoveItConfigsBuilder
import math
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from srdfdom.srdf import SRDF

from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)
from launch_ros.substitutions import FindPackageShare

def construct_angle_radians(loader, node):
    """Utility function to construct radian values from yaml."""
    value = loader.construct_scalar(node)
    try:
        return float(value)
    except SyntaxError:
        raise Exception("invalid expression: %s" % value)


def construct_angle_degrees(loader, node):
    """Utility function for converting degrees into radians from yaml."""
    return math.radians(construct_angle_radians(loader, node))


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        yaml.SafeLoader.add_constructor("!radians", construct_angle_radians)
        yaml.SafeLoader.add_constructor("!degrees", construct_angle_degrees)
    except Exception:
        raise Exception("yaml support not available; install python-yaml")

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None



def generate_servo_launch(moveit_config):

    ld = LaunchDescription()

    servo_yaml = load_yaml("tm12_moveit_config", "config/tm_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    print("passes...")
    # kinematics_solver_params = PathJoinSubstitution(
    #     [FindPackageShare("tm12_moveit_config"), "config", "kinematics.yaml"]
    # )


    ld.add_action(
        Node(
            package="moveit_servo",
            executable="servo_node_main",
            parameters=[
                servo_params,
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
               # kinematics_solver_params
                moveit_config.robot_description_kinematics,
            ],
            # ros_arguments= [{'use_intra_process_comms' : True}],
            output="screen",

        )
    )
    return ld

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("tm12", package_name="tm12_moveit_config").to_moveit_configs()
    return generate_servo_launch(moveit_config)
