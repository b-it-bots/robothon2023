# Robothon2023

## Dependencies

- [kortex](https://github.com/Kinovarobotics/kortex)
- [ros_kortex](https://github.com/Kinovarobotics/ros_kortex)
- [ros_kortex_vision](https://github.com/Kinovarobotics/ros_kortex_vision)

## Setup

1. Connect an ethernet cable from the robot to the laptop
2. Open Settings (laptop) -> Network -> Wired (click on setting icon) -> Go to IPv4 -> Click on "Manual" IPv4 Method -> Under Addresses, in Address, type `192.168.1.10` | in Netmask, type `255.255.255.0` -> Apply
3. On the browser (Chrome), type URL `192.168.1.12`
4. Username: admin, Password: <type-password-here>

## Modify launch file 

1. roscd kortex_driver/launch
2. Open kortex_driver.launch
3. Change the IP address to `192.168.1.12`
4. Change password to the password you set
5. Add default gripper, it should look like this (line 13) `<arg name="gripper" default="robotiq_2f_85" if="$(eval arg('arm') == 'gen3')"/>`
6. Change the IP address to `192.168.1.12` (line 3) in `kinova_vision_rgbd.launch` file

## Launch

```
roslaunch kortex_driver kortex_driver.launch
roslaunch robothon2023 task_board_detector.launch
roslaunch kinova_vision kinova_vision_rgbd.launch
```

Visualize stuff in Rviz using [config/robothon.rviz](config/robothon.rviz)

# Manipulation First step 
# Moving above the board_link frame

> :warning: ** Below command will move the robot to holding position then moves down **

```
roslaunch robothon2023 kinova_arm.launch
```


