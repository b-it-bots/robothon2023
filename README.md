# Robothon2023
This repository contains our code and documentation of the approach used for the [Robothon® 2023 challenge](https://automatica-munich.com/en/munich-i/robothon/). We are a team from the Institute for AI and Autonomous Systems ([A<sup>2</sup>S](https://h-brs.de/en/a2s)) at Hochschule Bonn-Rhein-Sieg, Germany.


<p align="center">
  <img src="https://user-images.githubusercontent.com/47410011/230381047-89bfc69f-f113-4c27-846f-17bbb7ae878f.jpg" width=20% />
  <img src="https://user-images.githubusercontent.com/47410011/230386146-407067bd-04dd-4105-892f-4292a32af506.jpg" width=38% />
  <br></br>
</p>

# Documentation
A full description of our approach to solve each task can be found [here](docs/)

# Setup and Run
## Dependencies

- [kortex](https://github.com/Kinovarobotics/kortex)
- [ros_kortex](https://github.com/Kinovarobotics/ros_kortex)
- [ros_kortex_vision](https://github.com/Kinovarobotics/ros_kortex_vision)

## Setup the robot

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

Run each of the following in separate terminals:

```
roscore
roslaunch kortex_driver kortex_driver.launch
roslaunch robothon2023 task_sm.launch
roslaunch kinova_vision kinova_vision_rgbd.launch
```

Visualize stuff in Rviz using [config/robothon.rviz](config/robothon.rviz)

#### Expected behaviour
* The arm moves above the board
* Finds the board, reorients, and finds the board again
* Completes the tasks specified by the [task_order](config/task_params.yaml) parameter

### Testing
For testing individual components, you may want to launch the `task_board_detector` separately. Launch it as follows, and publish two `e_start` messages:
```
roslaunch robothon2023 task_board_detector.launch
rostopic pub /task_board_detector/event_in std_msgs/String e_start
rostopic pub /points_of_interest_publisher/event_in std_msgs/String e_start
```

