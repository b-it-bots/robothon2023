# robothon2023

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


