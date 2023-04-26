# Platform
We use the Kinova Gen3 with a Robotiq 2F-85 gripper. The arm comes with a wrist-mounted RealSense depth module, which provides an RGB and depth stream. Additionally, the joint torques are used to estimate the force and torque at the end-effector. We use an Intel NUC with an Intel Core i7 processor as the main computer.

# Software
We mainly use Python for development (with one component in C++), and ROS Noetic for interprocess communication, including communication with the [kortex_driver](https://github.com/Kinovarobotics/ros_kortex). For vision, we use the [kinova_vision](https://github.com/Kinovarobotics/ros_kortex_vision) ROS package to publish RGB images and point clouds from the wrist-mounted camera, and [OpenCV](https://opencv.org/) for processing the RGB images and [PCL](https://pointclouds.org/) for some minor point cloud processing.

# Tasks
Below, we describe our general approach to solve each task. Some common elements are described at the end.
## Task 1: Press start trial button
### Board localization
The task board is placed at a random orientation on two velcro strips to hold it in place for a particular trial. Therefore, for all tasks (including this task), the task board needs to be localized.

The main idea to localize the board is to find a pair of blue and red circles next to each other. The steps are as follows:

1. Apply a blue filter on the RGB image after converting it into HSV space.
2. Apply a red filter on the RGB image after converting it into HSV space.
3. Detect circles on both filtered images using the Hough transform.
4. Select blue and red circle pairs which are closest to each other, and whose radius is roughly equal
5. For the selected pair, obtain their 3D positions by mapping pixels in the RGB image to points in the 3D point cloud
6. Determine the orientation (yaw) of the board by the vector through the centers of the circles
7. Determine the position of the board as the center of the blue circle.

The origin of the board is said to be at the center of the blue button, and is broadcast as a transform. In addition, static transformations with respect to the board origin are also broadcast, based on manual measurements of points of interest on the board (e.g. slider location, door knob, etc.)

The images below illustrate the blue and red filters, and the final result of the detected buttons.

<p float="left">
  <img src="images/button/blue_mask.jpg" width="250" />
  <img src="images/button/red_mask.jpg" width="250" />
  <img src="images/button/result.jpg" width="250" />
</p>

### Pressing the button
1. Based on the detected board origin, the arm is moved to a position just above the blue button (i.e. the start trial button)
2. Using velocity control, the arm is moved slowly down, until the vertical force felt at the end-effector exceeds a threshold. For more details about the force monitoring see [Force monitoring](#force-monitoring).
3. The arm is moved back up

## Task 2: Move slider to setpoints
1. The arm is moved above the initial location of the slider, and moved down until contact with the board
2. The arm moves the slider all the way to the end with one finger, then moves it back to its original position.
3. Forces are monitored during the slide to determine the end of the slider's path.

## Task 3: Plug probe in test port

## Task 4: Open door and probe circuit

## Task 5: Wrap cable replace probe

## Task 6: Press stop trial button


## Force monitoring
The `kortex_driver` publishes an estimated wrench (force + torque) at the end-effector based on the sensed torques at each joint. The estimated wrench is quite noisy, and inaccurate since it does not consider the dynamics of the arm for the estimation. This means, for example, that the wrench fluctuates a lot during arm motions (especially at high accelerations), and can be different in different positions of the arm. Therefore, we only monitor this wrench at low velocities, and only consider the relative change in wrench. To detect a contact with the board (for example to press a button), we monitor the difference between the latest estimated force along the Z axis to the mean Z force for a fixed history. When the difference exceeds a threshold, the arm is stopped.

## Visual servoing
For several tasks, the robot arm needs to be accurately aligned with parts of the board to complete the task successfully. Since we assume there are inaccuracies in the initially estimated position of the board, we cannot fully rely on fixed transformation from the origin of the board. Therefore, we use visual servoing to align more accurately.

Visual servoing is only performed in the X-Y plane, and depending on the task we have a target element which we align to (e.g. the red port, door knob, etc.). For the task, we define the target position for the selected element, and move the arm in velocity control mode until the selected element is at the target position.
