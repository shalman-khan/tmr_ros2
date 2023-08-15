## TM12 Servo control using Moveit Servo

The package is to teleoperate a tm_robot arm using joint angles

![servo_input](https://github.com/shalman-khan/tmr_ros2/blob/humble-shalman/figures/Keyboard_control.png)
![moveit_config](https://github.com/shalman-khan/tmr_ros2/blob/humble-shalman/figures/moveit_servo.png)


<br>

<br>

### Create a Workspace 

```
mkdir omron_ws
cd omron_ws && mkdir src && cd src
```

### Moveit2 Humble Setup [This process may take a while]

```
sudo apt install ros-humble-moveit 
```

### Clone the repository and build

```
cd ~/omron_ws/src
git clone -b humble-shalman https://github.com/shalman-khan/tmr_ros2.git
cd ~/omron_ws
colcon build
source install/setup.bash
```

### Terminal 1:[skip for simulation] [Robot Arm (client) side]
```
cd ~/omron_ws
source install/setup.bash
ros2 launch tm_driver tm_bringup.launch.py robot_ip:=192.168.0.25
```

### Terminal 2: [Robot Arm (client) side]
```
cd ~/omron_ws
source install/setup.bash
ros2 launch tm12_moveit_config demo.launch.py

```

### Terminal 3: [Robot Arm (client) side]
```
cd ~/omron_ws
source install/setup.bash
ros2 launch tm12_moveit_config servo_launch.py
```


### Terminal 4: [Server side]
```
cd ~/omron_ws
source install/setup.bash
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}
ros2 run tm12_moveit_config servo_keyboard_input 
```

### Tap repeatedly  1 (or) 2 (or) 3 (or) 4 (or) 5 (or) 6 based on desired joint angle for control
