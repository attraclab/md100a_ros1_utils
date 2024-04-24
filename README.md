# AT_MOTOR-DRIVER_100A_2ch Ver.9.0 ROS2 Utilities

This package is a helper to use motor driver in ROS1.

## Connection Diagram

Please follow the connection diagram as picture below,

![](images/atcrawler_ROS_connection.jpeg)

## Dependencies

- [ROS1 noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
- rosserial_python `sudo apt install ros-noetic-rosserial-python`
- make sure the user can access serial port `sudo usermod -a -G dialout $USER`
- clone and build this package
	```
	cd ~/catkin_ws/src/
	git clone https://github.com/attraclab/md100a_ros1_utils.git
	cd ~/catkin_ws
	catkin_make
	source ~/catkin_ws/devel/setup.bash
	```

## Setup

It is recommended to put general ROS env paramters i.e. ROS_MASTER_URI and etc. on .bashrc so every terminals will have the same of this.

### Firmwares

As default, output ROS topics with the namespace as `/md100a/...`, all of the firmware is stored at [firmwares folder](./firmwares), please check on the listed firmware below and description,

### Robot computer (remote)

login to robot computer and put these lines below on .bashrc file

```
## for ROS1
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=192.168.8.198
export ROS_HOSTNAME=$ROS_IP

```

make sure to change ip address on `ROS_IP` to the Robot's IP.

### Your computer (host)
```
## for ROS1
export ROS_MASTER_URI=http://192.168.8.198:11311
export ROS_IP=192.168.8.113
export ROS_HOSTNAME=$ROS_IP

```

make sure to change `ROS_MASTER_URI` to robot's ip, and `ROS_IP` to your computer ip.


### Source env

We need to source ROS enviroment according to which ROS version/package we would like to run.

When sourcing ROS1 env, we need to run the following lines before the package
```sh
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```
this is assume your ROS1 workspace is `catkin_ws`.


### udev rules

It's better to setup the udev-rules to avoid USB devices conflicts.

```sh
sudo cp ~/catkin_ws/src/md100a_ros1_utils/udev_rules/99-md100a.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
## try unplug and plug back the usb port
ls /dev/
## you should be able to see /dev/md100a
```

## Understand Topics and Node

Before running this, we better to understand what each topic is doing first. We can try this on ROS1 by running these following command.

```sh
# Terminal 1
# source ROS1 env
roscore

## Terminal 2
# source ROS1 env
rosrun rosserial_python serial_node.py  _port:=/dev/md100a _baud:=921600
# you can only use /dev/md100a if you've done setup udev rules from Setup step above. 
```

All of topics are in standard or existing message type, please check the following,

### Publish

- `/md100a/cart_mode` as std_msgs/Int8; this is cart mode, 0: hold, 1: manual, 2: auto. And it's reflected from ch5 of RC transmitter. When the RC transmitter is turned off, the mode will be changed to 2 automatically.
- `/md100a/sbus_rc_ch` as std_msgs/Int16MultiArray; this is array of sbus value from RC transmitter's channels.
- `/md100a/imu` as sensor_msgs/Imu; this is IMU data if motor driver recognized BNO055 on GROVE_I2C port.


### Subscribe

- `/md100a/cart_mode_cmd` as std_msgs/UInt8; we can change cart mode programmatically from this topic, 0: hold, 1: manual, 2: auto.
- `/md100a/pwm_cmd` as std_msgs/Int16MultiArray;  we can send PWM value of left/right wheels as [left_pwm, right_pwm], the range is **1000-1500-2000**.
- `/md100a/pwm_out` as std_msgs/Int16MultiArray; this topic depends on the firmware on ESC. If the firmware is *20231117c_AT_MOTOR-DRIVER_100A_2ch_ver9_PWMOUT.hex* then you could see this. We can send PWM extra output on PWM1, PWM2, and PWM3 ports to drive servo or any PWM driven devices, the value is [PWM1, PWM2, PWM3], the range is **1000-1500-2000**.

### Node

- `cmd_vel_converter.py` is to convert general `/cmd_vel` topic to `/md100a/pwm_cmd`, because this driver has no feedback or direct control of wheel speed, so we just estimate the maximum vx as 2 m/s as the max PWM 2000, also in the other side -2 m/s as min PWM 1000.

## Run

```sh
## Terminal 1
## source ROS1 env
roscore

## Termianl 2
## source ROS1 env
rosrun rosserial_python serial_node.py  _port:=/dev/md100a _baud:=921600

## Terminal 3
## source ROS2 env
## this is to convert cmd_vel to /md100a/pwm_cmd topic
rosrun md100a_ros1_utils cmd_vel_converter
``` 

You could drive the robot by sending `cmd_vel` from `rqt_robot_steering` node or your own ROS node program.