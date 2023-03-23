#ifndef TASK_BOARD_DETECTOR_H_
#define TASK_BOARD_DETECTOR_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PolygonStamped.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/geometry/planar_polygon.h>
#include <pcl/filters/extract_indices.h>
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
    ros::Publisher pose_publisher;
    ros::Publisher image_publisher;
    boost::shared_ptr<tf::TransformListener> tf_listener;
    tf2_ros::TransformBroadcaster tf_broadcaster;
    std::string target_frame;

    double passthrough_x_min;
    double passthrough_x_max;
    double passthrough_y_min;
    double passthrough_y_max;
    double cluster_tolerance;
    double voxel_leaf_size;
    int min_cluster_size;
    int max_cluster_size;

    pcl::PassThrough<PointT> passthrough_filter;
    pcl::VoxelGrid<PointT> voxel_filter;
    pcl::SACSegmentationFromNormals<PointT, PointNT> sac_segmentation;
    pcl::ProjectInliers<PointT> project_inliers;
    pcl::EuclideanClusterExtraction<PointT> cluster_extraction;
    pcl::NormalEstimation<PointT, PointNT> normal_estimation;
    pcl::ConvexHull<PointT> convex_hull;
    pcl::ExtractPolygonalPrismData<PointT> extract_polygonal_prism;
    pcl::ExtractIndices<PointT> extract;

    PointCloud::Ptr getPlane(const PointCloud::Ptr &full_cloud,
                             pcl::ModelCoefficients::Ptr &coefficients,
                             pcl::PlanarPolygon<PointT>::Ptr &hull_polygon, PointCloud::Ptr &hull_pointcloud);

    message_filters::Subscriber<sensor_msgs::Image> *image_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> msgSyncPolicy;
    message_filters::Synchronizer<msgSyncPolicy> *msg_sync_;
    void synchronizeCallback(const sensor_msgs::ImageConstPtr &image,
                 const sensor_msgs::PointCloud2ConstPtr &cloud);
    /**
     * find blue and red buttons in 2D and 3D, and determine orientation of the board
     */
    bool findBoardOrigin(PointCloud::Ptr &full_cloud, const cv::Mat &image, const std::string &img_frame_id, const std::string &pc_frame_id);
    /**
     * find top plane of the task board in 3D
     */
    bool findBoardPlane(PointCloud::Ptr &full_cloud, const std::string &frame_id);

    /**
     * get 3D points of a circle defined in 2D
     */
    PointCloud::Ptr get3DPointsInCircle(const PointCloud::Ptr &full_cloud, const cv::Vec3f &circle);
};

#endif
