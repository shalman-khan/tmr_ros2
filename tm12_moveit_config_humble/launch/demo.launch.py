# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_demo_launch


# def generate_launch_description():
#     moveit_config = MoveItConfigsBuilder("tm12", package_name="tm12_moveit_config_humble").to_moveit_configs()
#     return generate_demo_launch(moveit_config)
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
import launch
import launch_ros.actions
from launch_ros.actions import Node
import yaml
import os
import math
from ament_index_python.packages import get_package_share_directory

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



def generate_launch_description():
    # Generate the MoveIt configuration and demo launch
    moveit_config = MoveItConfigsBuilder("tm12", package_name="tm12_moveit_config_humble").to_moveit_configs()
    moveit_demo_launch = generate_demo_launch(moveit_config)


    servo_yaml = load_yaml("tm12_moveit_config_humble", "config/tm_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    moveit_servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
        ],
        # ros_arguments= [{'use_intra_process_comms' : True}],
        output="screen",
    )


    # Combine both launch descriptions
    full_launch_description = launch.LaunchDescription([
        moveit_demo_launch,
        moveit_servo_node,
    ])

    return full_launch_description
