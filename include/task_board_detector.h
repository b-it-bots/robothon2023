#ifndef TASK_BOARD_DETECTOR_H_
#define TASK_BOARD_DETECTOR_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PolygonStamped.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <string>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::Normal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudN;


class TaskBoardDetector
{
public:
    TaskBoardDetector(ros::NodeHandle &nh);
    virtual ~TaskBoardDetector();

private:
    ros::NodeHandle nh;

    ros::Publisher pointcloud_publisher;
    ros::Publisher plane_polygon_publisher;
    ros::Publisher image_publisher;
    ros::Publisher board_pose_publisher;
    ros::Subscriber camera_info_sub;

    ros::Publisher event_out_publisher;
    ros::Subscriber event_in_subscriber;

    boost::shared_ptr<tf::TransformListener> tf_listener;
    std::string target_frame;
    sensor_msgs::CameraInfoConstPtr camera_info;
    bool received_camera_info;

    geometry_msgs::TransformStamped circle_tf;

    message_filters::Subscriber<sensor_msgs::Image> *image_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_sub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> msgSyncPolicy;
    message_filters::Synchronizer<msgSyncPolicy> *msg_sync;
    void synchronizeCallback(const sensor_msgs::ImageConstPtr &image,
                 const sensor_msgs::PointCloud2ConstPtr &cloud);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);
    void eventInCallback(const std_msgs::StringConstPtr &msg);
    /**
     * find blue and red buttons in 2D and 3D, and determine orientation of the board
     */
    bool findBoardOrigin(PointCloud::Ptr &full_cloud, cv::Mat &debug_image, const std::string &img_frame_id, const std::string &pc_frame_id);
    /**
     * get 3D points of a circle defined in 2D
     */
    PointCloud::Ptr get3DPointsInCircle(const PointCloud::Ptr &full_cloud, const cv::Vec3f &circle);

    std::vector<cv::Point> getLargestContour(const cv::Mat &img);
    void erode(cv::Mat &img);
    void dilate(cv::Mat &img);
};


#endif
