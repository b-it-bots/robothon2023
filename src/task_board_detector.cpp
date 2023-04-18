#include "task_board_detector.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

TaskBoardDetector::TaskBoardDetector(ros::NodeHandle &nh) : nh(nh), received_camera_info(false)
{
    nh.param<std::string>("target_frame", target_frame, "base_link");
    tf_listener.reset(new tf::TransformListener);

    image_publisher = nh.advertise<sensor_msgs::Image>("debug_image", 1);
    board_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("approximate_board_pose", 1);
    camera_info_sub = nh.subscribe<sensor_msgs::CameraInfo>("camera_info_topic", 1, &TaskBoardDetector::cameraInfoCallback, this);

    event_in_subscriber = nh.subscribe<std_msgs::String>("event_in", 1, &TaskBoardDetector::eventInCallback, this);
    event_out_publisher = nh.advertise<std_msgs::String>("event_out", 1);

}

TaskBoardDetector::~TaskBoardDetector()
{
}

void TaskBoardDetector::eventInCallback(const std_msgs::StringConstPtr &msg)
{
    if (msg->data == "e_start")
    {
        ROS_INFO_STREAM("Starting board detection");
        image_sub = new message_filters::Subscriber<sensor_msgs::Image> (nh, "input_image_topic", 1);
        cloud_sub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh, "input_pointcloud_topic", 1);
        msg_sync = new message_filters::Synchronizer<msgSyncPolicy> (msgSyncPolicy(10), *image_sub, *cloud_sub);
        msg_sync->registerCallback(boost::bind(&TaskBoardDetector::synchronizeCallback, this, _1, _2));
    }
    else if (msg->data == "e_stop")
    {
        ROS_INFO_STREAM("Stopped board detection");
        if (image_sub)
        {
            image_sub->unsubscribe();
            cloud_sub->unsubscribe();
        }
        std_msgs::String stop_msg;
        stop_msg.data = "e_stopped";
        event_out_publisher.publish(stop_msg);
    }
}

void TaskBoardDetector::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
    camera_info = msg;
    received_camera_info = true;
}

void TaskBoardDetector::synchronizeCallback(const sensor_msgs::ImageConstPtr &image,
                      const sensor_msgs::PointCloud2ConstPtr &cloud_in)
{
    if (!received_camera_info)
    {
        return;
    }
    sensor_msgs::PointCloud2 cloud_in_transformed;
    try
    {
        ros::Time common_time;
        tf_listener->getLatestCommonTime(target_frame, cloud_in->header.frame_id, common_time, NULL);
        // -fpermissive because we're changing a const
        cloud_in->header.stamp = common_time;
        tf_listener->waitForTransform(target_frame, cloud_in->header.frame_id, ros::Time::now(),
                                      ros::Duration(1.0));
        pcl_ros::transformPointCloud(target_frame, *cloud_in, cloud_in_transformed, *tf_listener);
        cloud_in_transformed.header.frame_id = target_frame;
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("PCL transform error: %s", ex.what());
        return;
    }


    PointCloud::Ptr full_cloud(new PointCloud);
    pcl::fromROSMsg(cloud_in_transformed, *full_cloud);

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat img = cv_ptr->image.clone();

    bool origin_success = findBoardOrigin(full_cloud, img, image->header.frame_id, cloud_in_transformed.header.frame_id);
    if (!origin_success)
    {
        ROS_ERROR_STREAM("Could not find board origin");
    }
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    img_msg->header.stamp = ros::Time::now();
    img_msg->header.frame_id = image->header.frame_id;
    image_publisher.publish(img_msg);
}


/**
 * find blue and red buttons in 2D and 3D, and determine orientation of the board
 */
bool TaskBoardDetector::findBoardOrigin(PointCloud::Ptr &full_cloud, cv::Mat &debug_image, const std::string &img_frame_id, const std::string &pc_frame_id)
{
    /**
     * 1. Mask blue and red using HSV filter
     * 2. Find circles on each mask
     * 3. filter out circles by only considering two roughly equal circles next to each other
     * 4. get corresponding 3D positions of the circles
     * 5. determine axis from the two points, and broadcast a transform for the origin of the board
     */

    cv::Mat hsv;
    cv::cvtColor(debug_image, hsv, cv::COLOR_BGR2HSV);

    cv::Mat red_mask1, red_mask2, red_mask, blue_mask;

    // we need two masks for red because
    // the hue range wraps around 180
    // i.e. 170-180 and 0-10 are all red
    // TODO: parameterize this for easy tuning
    cv::Scalar red_lower1(0, 100, 140);
    cv::Scalar red_lower2(175, 100, 140);
    cv::Scalar red_upper1(5, 255, 255);
    cv::Scalar red_upper2(180, 255, 255);
    cv::inRange(hsv, red_lower1, red_upper1, red_mask1);
    cv::inRange(hsv, red_lower2, red_upper2, red_mask2);
    cv::bitwise_or(red_mask1, red_mask2, red_mask);

    std::vector<cv::Point> largest_contour = getLargestContour(red_mask);
    if (largest_contour.empty())
    {
        ROS_ERROR_STREAM("Could not find blue and red buttons");
        return false;
    }
    std::vector<std::vector<cv::Point>> old_contours;
    old_contours.push_back(largest_contour);
    cv::drawContours(red_mask, old_contours, 0, cv::Scalar(0, 0, 0), cv::FILLED, 8);

    cv::Scalar blue_lower(85, 100, 140);
    cv::Scalar blue_upper(120, 255, 255);
    cv::inRange(hsv, blue_lower, blue_upper, blue_mask);

    cv::blur(red_mask, red_mask, cv::Size(3,3));
    std::vector<cv::Vec3f> red_circles;
    // param1=50, param2=10, minradius=10, maxradius=50
    cv::HoughCircles(red_mask, red_circles, cv::HOUGH_GRADIENT, 1, 20, 50, 10, 10, 50);

    // fill red contours
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(red_mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::drawContours(red_mask, contours, -1, cv::Scalar(255, 255, 255), cv::FILLED);

    /*
    cv::imshow("red_mask", red_mask);
    cv::imshow("blue_mask", blue_mask);
    cv::waitKey(10);

    */

    cv::blur(blue_mask, blue_mask, cv::Size(3,3));
    std::vector<cv::Vec3f> blue_circles;
    cv::HoughCircles(blue_mask, blue_circles, cv::HOUGH_GRADIENT, 1, 20, 50, 10, 10, 50);

    if (red_circles.empty() or blue_circles.empty())
    {
        ROS_ERROR_STREAM("No circles found");
        return false;
    }

    int min_red_idx = 0;
    int min_blue_idx = 0;
    double min_dist = 1000.0;
    bool found_button_pair = false;
    for (size_t i = 0; i < red_circles.size(); i++)
    {
        for (size_t j = 0; j < blue_circles.size(); j++)
        {
            double dist = cv::norm(cv::Point(red_circles[i][0], red_circles[i][1])- cv::Point(blue_circles[j][0], blue_circles[j][1]));
            double blue_r = blue_circles[j][2];
            double red_r = red_circles[i][2];
            // closest distance where: circles are not overlapping, and diff in radius is less than N pixels
            if (dist < min_dist and dist > (blue_r + red_r) and std::abs(blue_r - red_r) < 10.0)
            {
                min_dist = dist;
                min_red_idx = i;
                min_blue_idx = j;
                found_button_pair = true;
            }
        }
    }
    if (!found_button_pair)
    {
        ROS_ERROR_STREAM("Could not find blue and red buttons");
        return false;
    }

    cv::circle(debug_image, cv::Point(red_circles[min_red_idx][0], red_circles[min_red_idx][1]), red_circles[min_red_idx][2], cv::Scalar(0,255,0), 2, cv::LINE_AA);
    cv::circle(debug_image, cv::Point(blue_circles[min_blue_idx][0], blue_circles[min_blue_idx][1]), blue_circles[min_blue_idx][2], cv::Scalar(0,255,0), 2, cv::LINE_AA);

    cv::Vec3f selected_red_circle = red_circles[min_red_idx];
    cv::Vec3f selected_blue_circle = blue_circles[min_blue_idx];

    PointCloud::Ptr red_circle_3d = get3DPointsInCircle(full_cloud, selected_red_circle);
    PointCloud::Ptr blue_circle_3d = get3DPointsInCircle(full_cloud, selected_blue_circle);

    Eigen::Vector4f red_circle_center;
    pcl::compute3DCentroid(*red_circle_3d, red_circle_center);

    Eigen::Vector4f blue_circle_center;
    pcl::compute3DCentroid(*blue_circle_3d, blue_circle_center);

    double box_yaw = std::atan2((red_circle_center[1] - blue_circle_center[1]), (red_circle_center[0] - blue_circle_center[0]));

    tf2::Quaternion q;
    q.setRPY(0, 0, box_yaw);


    geometry_msgs::PoseStamped board_origin_pose;

    board_origin_pose.header.stamp = ros::Time::now();
    board_origin_pose.header.frame_id = pc_frame_id;
    board_origin_pose.pose.position.x = blue_circle_center[0];
    board_origin_pose.pose.position.y = blue_circle_center[1];
    board_origin_pose.pose.position.z = blue_circle_center[2];
    board_origin_pose.pose.orientation.x = q.x();
    board_origin_pose.pose.orientation.y = q.y();
    board_origin_pose.pose.orientation.z = q.z();
    board_origin_pose.pose.orientation.w = q.w();

    board_pose_publisher.publish(board_origin_pose);

    return true;

}

PointCloud::Ptr TaskBoardDetector::get3DPointsInCircle(const PointCloud::Ptr &full_cloud, const cv::Vec3f &circle)
{
    /**
     * loop through points in circumscribed square, find 3D positions if inside circle
     */
    int width = full_cloud->width;
    int height = full_cloud->height;
    int grid_x_start = std::max(int(circle[0] - circle[2]), 0);
    int grid_y_start = std::max(int(circle[1] - circle[2]), 0);
    int grid_x_end = std::min(int(circle[0] + circle[2]), width-1);
    int grid_y_end = std::min(int(circle[1] + circle[2]), height-1);
    PointCloud::Ptr circle_cloud(new PointCloud);
    for (int i = grid_x_start; i <= grid_x_end; i++)
    {
        for (int j = grid_y_start; j <= grid_y_end; j++)
        {
            double dist = cv::norm(cv::Point(i, j)- cv::Point(circle[0], circle[1]));
            if (dist <= circle[2])
            {
                PointT point_3d = full_cloud->at(i, j);
                circle_cloud->push_back(point_3d);
            }
        }
    }
    circle_cloud->is_dense = false;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*circle_cloud, *circle_cloud, indices);
    return circle_cloud;
}



std::vector<cv::Point> TaskBoardDetector::getLargestContour(const cv::Mat &img)
{
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(img, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty())
    {
        return std::vector<cv::Point>();
    }
    double max_area = 0.0;
    int largest_contour_idx = 0;
    for (int idx = 0; idx < contours.size(); idx++)
    {
        double area = cv::contourArea(contours[idx]);
        if (area > max_area)
        {
            max_area = area;
            largest_contour_idx = idx;
        }
    }
    return contours[largest_contour_idx];
}


void TaskBoardDetector::erode(cv::Mat &img)
{
    int erosion_size = 1;
    int erosion_shape = cv::MORPH_RECT;
    cv::Mat element = cv::getStructuringElement(erosion_shape, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1), cv::Point(erosion_size, erosion_size));
    cv::erode(img, img, element);
}

void TaskBoardDetector::dilate(cv::Mat &img)
{
    int dilation_size = 1;
    int dilation_shape = cv::MORPH_RECT;
    cv::Mat element = cv::getStructuringElement(dilation_shape, cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1), cv::Point(dilation_size, dilation_size));
    cv::dilate(img, img, element);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "task_board_detector");

    ros::NodeHandle n("~");

    TaskBoardDetector task_board_detector(n);

    ros::spin();

    return 0;
}
