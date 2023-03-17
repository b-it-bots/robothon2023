#include "task_board_detector.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

TaskBoardDetector::TaskBoardDetector(ros::NodeHandle &nh) : nh(nh)
{
    nh.param<std::string>("target_frame", target_frame, "base_link");
    nh.param<double>("passthrough_x_min", passthrough_x_min, 0.0);
    nh.param<double>("passthrough_x_max", passthrough_x_max, 8.0);
    nh.param<double>("passthrough_y_min", passthrough_y_min, -0.7);
    nh.param<double>("passthrough_y_max", passthrough_y_max, 0.7);
    nh.param<double>("voxel_leaf_size", voxel_leaf_size, 0.01);
    nh.param<double>("cluster_tolerance", cluster_tolerance, 0.02);
    nh.param<int>("min_cluster_size", min_cluster_size, 1000);
    nh.param<int>("max_cluster_size", max_cluster_size, 50000);
    tf_listener.reset(new tf::TransformListener);

    cluster_extraction.setSearchMethod(boost::make_shared<pcl::search::KdTree<PointT> >());

    normal_estimation.setSearchMethod(boost::make_shared<pcl::search::KdTree<PointT> >());
    normal_estimation.setRadiusSearch(0.10);

    pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("debug_pointcloud", 1);
    image_publisher = nh.advertise<sensor_msgs::Image>("debug_image", 1);
    plane_polygon_publisher = nh.advertise<geometry_msgs::PolygonStamped>("output_plane_polygon", 1);
    pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("object_pose", 1);

    image_sub_ = new message_filters::Subscriber<sensor_msgs::Image> (nh, "input_image_topic", 1);
    cloud_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh, "input_pointcloud_topic", 1);
    msg_sync_ = new message_filters::Synchronizer<msgSyncPolicy> (msgSyncPolicy(10), *image_sub_, *cloud_sub_);
    msg_sync_->registerCallback(boost::bind(&TaskBoardDetector::synchronizeCallback, this, _1, _2));
}

TaskBoardDetector::~TaskBoardDetector()
{
}

void TaskBoardDetector::synchronizeCallback(const sensor_msgs::ImageConstPtr &image,
                      const sensor_msgs::PointCloud2ConstPtr &cloud_in)
{
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

    bool plane_success = findBoardPlane(full_cloud, cloud_in_transformed.header.frame_id);
    if (!plane_success)
    {
        ROS_WARN_STREAM("Could not find board plane");
    }
    else
    {
        ROS_DEBUG_STREAM("Found task board!");
    }

    findBoardOrigin(full_cloud, img, image->header.frame_id);

}

/**
 * find top plane of the task board in 3D
 */
bool TaskBoardDetector::findBoardPlane(PointCloud::Ptr &full_cloud, const std::string &frame_id)
{
    /**
     * 1. find largest plane
     * 2. extract points above plane
     * 3. Euclidean cluster, and select largest cluster
     * 4. find plane of largest cluster
     */
    pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
    pcl::PlanarPolygon<PointT>::Ptr hull_polygon(new pcl::PlanarPolygon<PointT>);
    PointCloud::Ptr hull_pointcloud(new PointCloud);

    ROS_DEBUG_STREAM("finding largest plane");
    PointCloud::Ptr plane = getPlane(full_cloud, plane_coefficients, hull_polygon, hull_pointcloud);

    if (hull_pointcloud->empty())
    {
        return false;
    }

    ROS_DEBUG_STREAM("extract polygon prism above largest plane");
    pcl::PointIndices::Ptr object_indices(new pcl::PointIndices);

    extract_polygonal_prism.setHeightLimits(0.05, 0.5); // height above plane
    extract_polygonal_prism.setInputPlanarHull(hull_pointcloud);
    extract_polygonal_prism.setInputCloud(full_cloud);
    extract_polygonal_prism.segment(*object_indices);

    PointCloud::Ptr objects_above_plane(new PointCloud);
    extract.setInputCloud(full_cloud);
    extract.setIndices(object_indices);
    extract.filter(*objects_above_plane);

    if (objects_above_plane->points.empty())
    {
        return false;
    }


    voxel_filter.setInputCloud(objects_above_plane);
    voxel_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    voxel_filter.filter(*objects_above_plane);


    ROS_DEBUG_STREAM("find clusters");
    std::vector<pcl::PointIndices> clusters_indices;
    std::vector<PointCloud::Ptr> clusters;
    cluster_extraction.setInputCloud(objects_above_plane);
    cluster_extraction.setClusterTolerance(cluster_tolerance);
    cluster_extraction.setMinClusterSize(min_cluster_size);
    cluster_extraction.setMaxClusterSize(max_cluster_size);
    cluster_extraction.extract(clusters_indices);


    ROS_DEBUG_STREAM("Num clusters " << clusters_indices.size());
    if (clusters_indices.empty())
    {
        return false;
    }


    int largest_cluster_id = 0;
    int num_points = 0;
    for (size_t i = 0; i < clusters_indices.size(); i++)
    {
        const pcl::PointIndices& cluster_indices = clusters_indices[i];
        PointCloud::Ptr cluster(new PointCloud);
        pcl::copyPointCloud(*objects_above_plane, cluster_indices, *cluster);
        clusters.push_back(cluster);

        if (cluster->points.size() > num_points)
        {
            num_points = cluster->points.size();
            largest_cluster_id = i;
        }
    }

    if (!clusters.empty())
    {
        PointCloud::Ptr cluster_plane = getPlane(clusters[largest_cluster_id], plane_coefficients, hull_polygon, hull_pointcloud);
        geometry_msgs::PolygonStamped poly;
        poly.header.stamp = ros::Time::now();
        poly.header.frame_id = frame_id;

        for (int i = 0; i < hull_pointcloud->size(); i++)
        {
            geometry_msgs::Point32 pt;
            pt.x = hull_pointcloud->points[i].x;
            pt.y = hull_pointcloud->points[i].y;
            pt.z = hull_pointcloud->points[i].z;
            poly.polygon.points.push_back(pt);
        }
        if (poly.polygon.points.size() > 0)
        {
            plane_polygon_publisher.publish(poly);
        }

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*(clusters[largest_cluster_id]), centroid);

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = frame_id;
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = centroid[0];
        pose.pose.position.y = centroid[1];
        pose.pose.position.z = centroid[2];
        pose.pose.orientation.w = 1.0;
        pose_publisher.publish(pose);
    }
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*(clusters[largest_cluster_id]), output);
    output.header.frame_id = frame_id;
    output.header.stamp = ros::Time::now();
    pointcloud_publisher.publish(output);
    return true;
}

/**
 * find blue and red buttons in 2D and 3D, and determine orientation of the board
 */
void TaskBoardDetector::findBoardOrigin(PointCloud::Ptr &full_cloud, const cv::Mat &image, const std::string &frame_id)
{
    /**
     * 1. Mask blue and red using HSV filter
     * 2. Find circles on each mask
     * 3. filter out circles by only considering two roughly equal circles next to each other [TODO]
     * 4. get corresponding 3D positions of the circles [TODO]
     * 5. determine axis from the two points, and broadcast a transform for the origin of the board [TODO]
     */
    cv::Mat debug_image = image.clone();

    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    cv::Mat red_mask1, red_mask2, red_mask, blue_mask;

    // we need two masks for red because
    // the hue range wraps around 180
    // i.e. 170-180 and 0-10 are all red
    // TODO: parameterize this for easy tuning
    cv::Scalar red_lower1(0, 35, 140);
    cv::Scalar red_lower2(175, 35, 140);
    cv::Scalar red_upper1(5, 255, 255);
    cv::Scalar red_upper2(180, 255, 255);
    cv::inRange(hsv, red_lower1, red_upper1, red_mask1);
    cv::inRange(hsv, red_lower2, red_upper2, red_mask2);
    cv::bitwise_or(red_mask1, red_mask2, red_mask);

    cv::Scalar blue_lower(90, 35, 140);
    cv::Scalar blue_upper(110, 255, 255);
    cv::inRange(hsv, blue_lower, blue_upper, blue_mask);

    cv::blur(red_mask, red_mask, cv::Size(3,3));
    std::vector<cv::Vec3f> red_circles;
    cv::HoughCircles(red_mask, red_circles, cv::HOUGH_GRADIENT, 1, 20, 50, 20, 1, 20);
    for (size_t i = 0; i < red_circles.size(); i++)
    {
        cv::Vec3i circle = red_circles[i];
        cv::circle(debug_image, cv::Point(circle[0], circle[1]), circle[2], cv::Scalar(0,255,0), 1, cv::LINE_AA);
    }

    cv::blur(blue_mask, blue_mask, cv::Size(3,3));
    std::vector<cv::Vec3f> blue_circles;
    cv::HoughCircles(blue_mask, blue_circles, cv::HOUGH_GRADIENT, 1, 20, 50, 20, 1, 20);
    for (size_t i = 0; i < blue_circles.size(); i++)
    {
        cv::Vec3i circle = blue_circles[i];
        cv::circle(debug_image, cv::Point(circle[0], circle[1]), circle[2], cv::Scalar(0,255,0), 1, cv::LINE_AA);
    }
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", debug_image).toImageMsg();
    img_msg->header.stamp = ros::Time::now();
    img_msg->header.frame_id = frame_id;
    image_publisher.publish(img_msg);

}

PointCloud::Ptr TaskBoardDetector::getPlane(const PointCloud::Ptr &full_cloud,
                    pcl::ModelCoefficients::Ptr &coefficients, pcl::PlanarPolygon<PointT>::Ptr &hull_polygon,
                    PointCloud::Ptr &hull_pointcloud)
{
    PointCloud::Ptr filtered_cloud(new PointCloud);
    passthrough_filter.setInputCloud(full_cloud);
    passthrough_filter.setFilterFieldName("x");
    passthrough_filter.setFilterLimits(passthrough_x_min, passthrough_x_max);
    passthrough_filter.filter(*filtered_cloud);

    passthrough_filter.setInputCloud(filtered_cloud);
    passthrough_filter.setFilterFieldName("y");
    passthrough_filter.setFilterLimits(passthrough_y_min, passthrough_y_max);
    passthrough_filter.filter(*filtered_cloud);

    voxel_filter.setInputCloud(filtered_cloud);
    voxel_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    voxel_filter.filter(*filtered_cloud);

    // indices of points which are part of the detected plane
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    PointCloudN::Ptr normals(new PointCloudN);
    normal_estimation.setInputCloud(filtered_cloud);
    normal_estimation.compute(*normals);

    sac_segmentation.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));
    sac_segmentation.setInputNormals(normals);
    sac_segmentation.setInputCloud(filtered_cloud);
    sac_segmentation.setOptimizeCoefficients(true);
    sac_segmentation.setModelType(pcl::SACMODEL_PLANE);
    sac_segmentation.setMethodType(pcl::SAC_RANSAC);
    sac_segmentation.setDistanceThreshold(0.05);
    sac_segmentation.setEpsAngle(0.3);

    sac_segmentation.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        std::cout << "could not find plane " << std::endl;
        return full_cloud;
    }

    PointCloud::Ptr plane_cloud(new PointCloud);

    project_inliers.setModelType(pcl::SACMODEL_PLANE);
    project_inliers.setInputCloud(filtered_cloud);
    project_inliers.setIndices(inliers);
    project_inliers.setModelCoefficients(coefficients);
    project_inliers.filter(*plane_cloud);

    convex_hull.setInputCloud(plane_cloud);
    convex_hull.reconstruct(*hull_pointcloud);
    hull_pointcloud->points.push_back(hull_pointcloud->points.front());
    hull_pointcloud->width += 1;
    hull_polygon->setContour(*hull_pointcloud);
    hull_polygon->setCoefficients(*coefficients);

    return plane_cloud;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "task_board_detector");

    ros::NodeHandle n("~");

    TaskBoardDetector task_board_detector(n);

    ros::spin();

    return 0;
}
