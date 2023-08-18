# Gazebo Simulation

## Install dependencies

`apt install ros-humble-gazebo-ros2-control ros-humble-joint-state-publisher ros-humble-ros2-control ros-humble-ros2-controllers`

## Launch Gazebo Simulation

### 1. Modify path to file in the [launch file](launch/controller_manager.launch.py)
`doc = xacro.parse(open("[PATH_TO_WORKSPACE]src/tmr_ros2/tm_gazebo_cf/xacro/moveit-tm12.urdf.xacro"))`


### 2. Launch Gazebo with Robot model and ROS 2 Control plugin
```
export WS_DIR=/home/rosi/workspaces/techman_robot
cd $WS_DIR
#Export Gazebo model path, change directories accordingly
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$WS_DIR/install/tm_gazebo_cf/share/tm_gazebo_cf/models/:$WS_DIR/install/tm_description/share/
source install/setup.bash
ros2 launch tm_gazebo_cf controller_manager.launch.py
```

### 3. Launch as in [Servo Readme](../tm12_moveit_config/Servo_Readme.md)
Skip the launch in Terminal 1 as instructed there.


## Issues:

### Using source install of gazebo_ros2_control 
Results in 

`gzserver: symbol lookup error: /home/rosi/ros2_ws/install/gazebo_ros2_control/lib/libgazebo_ros2_control.so: undefined symbol: _ZN18controller_manager17ControllerManagerC1ESt10unique_ptrIN18hardware_interface15ResourceManagerESt14default_deleteIS3_EESt10shared_ptrIN6rclcpp8ExecutorEERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESI_RKNS8_11NodeOptionsE`