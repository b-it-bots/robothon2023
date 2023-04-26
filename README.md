# Robothon2023

<p align="center">
  <img src="https://user-images.githubusercontent.com/47410011/230381047-89bfc69f-f113-4c27-846f-17bbb7ae878f.jpg" width=20% />
  <img src="https://user-images.githubusercontent.com/47410011/230386146-407067bd-04dd-4105-892f-4292a32af506.jpg" width=38% />
  <br></br>
</p>

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
roslaunch robothon2023 task_board_detector.launch use_mockup:=true  #if you want to use the mockup or use the vision 
roslaunch kinova_vision kinova_vision_rgbd.launch
```

Visualize stuff in Rviz using [config/robothon.rviz](config/robothon.rviz)

## Run
The following launch file will move the arm to the perceive pose, trigger the `task_board_detector` and then execute the actions as specified by the [task_order](config/task_params.yaml) parameter.
```
roslaunch robothon2023 task_sm.launch
```
If you want to trigger the `task_board_detector` separately, run both of the following:
```
rostopic pub /task_board_detector/event_in std_msgs/String e_start
rostopic pub /points_of_interest_publisher/event_in std_msgs/String e_start
```

## Manipulation First step 
### Button Press example 

> :warning: **Below command will move the robot to holding position then moves down**
  
> :warning: check the mockup gui and adjust the board_link by verifying in rviz

```
roslaunch robothon2023 button_press_test.launch
```
#### Expected behaviour
* The arm moves above the button
* Moves slowly in velocity mode in down direction 
* The button is pressed and then the arm moves up
* Arm moves on top of the graphical interface





