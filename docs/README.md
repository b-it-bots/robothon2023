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
### Grasp plug

To successfully grasp the plug from the board, the robotic arm first moves above the plug and then uses visual servoing to align itself with the plug. Once it is aligned, the arm moves down until it makes contact with the plug. The gripper is then closed to firmly grasp the plug. Finally, the arm moves back up, carefully holding the plug.

The images below show the original image, thresholded mask image,  detected red circle (to align the arm), and final result of the visual servoing.

<p float="left">
  <img src="images/plug_remove/image_original.png" width="220" />
  <img src="images/plug_remove/contour_mask.png" width="220" />
  <img src="images/plug_remove/hough_circles.png" width="220" />
  <img src="images/plug_remove/visual_servoing.png" width="220" />
</p>

### Insert plug in test port
To align with the red port, the robot first moves forward (with respect to its grasping position) for a fixed amount. To better align with the red port, visual servoing is performed using the ends of pairs of bolts on the board as targets.
The bolts are detected by applying adaptive thresholding on a cropped greyscale image, detecting circles, and selecting circles of equal sizes with an expected distance between their centers.

The images below show the original image, thresholded image, and the detected vertical (blue) and horizontal (green) pairs of circles. Red circles represent other detected circles which are rejected by the distance and pairing constraints.
The vertical pair of circles are used to align along the horizontal axis, and the horizontal pair for alignment along the vertical axis.

<p float="left">
  <img src="images/plug_insert/orig.jpg" width="250" />
  <img src="images/plug_insert/mask.jpg" width="250" />
  <img src="images/plug_insert/result.jpg" width="250" />
</p>

After alignment, the plug is inserted into the test port, by moving downwards with [force monitoring](#force-monitoring). This action is retried multiple times (with realignment) if it is detected that the plug has not been inserted (via a height and force threshold).

## Task 4: Open door and probe circuit
### Grasp and stow the probe
### Open the door
The arm is moved above the door knob, based on the fixed transformation from the origin of the board. To align more precisely with the door knob, we detect the door knob with the following steps:
1. Crop a region of interest
2. Apply Canny edge detection
3. Detect contours
4. Find circles in the contour image using the Hough transform

The images below illustrate the result of the Canny edge detection, contour detection and the final detected door knob.

<p float="left">
  <img src="images/door/canny.jpg" width="250" />
  <img src="images/door/contours.jpg" width="250" />
  <img src="images/door/result.jpg" width="250" />
</p>
The robot arm is aligned with the door knob using [visual servoing](#visual-servoing).

After alignment, the arm is moved down until contact with the door knob and retract back to a proper z position to grasp the door knob. The door is opened by following a predefined trajectory, and the arm is retracted back to a safe position position.

### Grasp probe and probe circuit

## Task 5: Wrap cable and re-place probe back in

### Pick up wire

<!-- - To successfully grasp the plug from the board, the robotic arm first moves above the plug and then uses visual servoing to align itself with the plug. Once it is aligned, the arm moves down until it makes contact with the plug. The gripper is then closed to firmly grasp the plug. Finally, the arm moves back up, carefully holding the plug. -->

The robot arm moves above the wire to grasp it and then utilizes visual servoing to align itself with the wire. After successful alignment, the arm moves down and firmly grasps the wire by closing the gripper.


<p float="left">
  <img src="images/pick_wire/image_ROI_rectangle.jpg" width="250" />
  <img src="images/pick_wire/image_canny.jpg" width="220" />
  <img src="images/pick_wire/image_filtered_contours.jpg" width="220" />
  <img src="images/pick_wire/visual_servoing.jpg" width="220" />
</p>


### Re-place probe back in

When re-placing the probe back in, the robot arm is first moved on top of the probe. Then, an inference is performed to detect the probe holder using a [YOLOv5-based](https://github.com/ultralytics/yolov5) trained model. The model is available for use and can be found at [HERE](https://github.com/b-it-bots/robothon2023/tree/main/models/probe_holder_horizontal). Once the probe holder is detected, the arm performs visual servoing to align with the holder before proceeding further.

<p float="left">
  <img src="images/probe_holder/probe_holder_output.jpg" width="350" />
</p>

## Task 6: Press stop trial button
This task is identical to Task 1, with the exception that the robot aligns with and presses the red button to stop the trial.


## Force monitoring
The `kortex_driver` publishes an estimated wrench (force + torque) at the end-effector based on the sensed torques at each joint. The estimated wrench is quite noisy, and inaccurate since it does not consider the dynamics of the arm for the estimation. This means, for example, that the wrench fluctuates a lot during arm motions (especially at high accelerations), and can be different in different positions of the arm. Therefore, we only monitor this wrench at low velocities, and only consider the relative change in wrench. To detect a contact with the board (for example to press a button), we monitor the difference between the latest estimated force along the Z axis to the mean Z force for a fixed history. When the difference exceeds a threshold, the arm is stopped.

## Visual servoing
For several tasks, the robot arm needs to be accurately aligned with parts of the board to complete the task successfully. Since we assume there are inaccuracies in the initially estimated position of the board, we cannot fully rely on fixed transformation from the origin of the board. Therefore, we use visual servoing to align more accurately.

Visual servoing is only performed in the X-Y plane, and depending on the task we have a target element which we align to (e.g. the red port, door knob, etc.). For the task, we define the target position for the selected element, and move the arm in velocity control mode until the selected element is at the target position.
