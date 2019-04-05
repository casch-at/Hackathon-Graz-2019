#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <box_detection/ConfigConfig.h>
#include <visualization_msgs/Marker.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/organized_edge_detection.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/impl/point_types.hpp>

ros::Publisher pub_cloud;
ros::Publisher pub_coeff;
ros::Publisher pub_edge;
ros::Publisher pub_target;
boost::recursive_mutex mutex;
box_detection::ConfigConfig config;

#if 0
void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Container for original & filtered data
  auto cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*input, *cloud);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);

  // Publish the data
  pub.publish (output);
}
#endif

#if 0
void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Container for original & filtered data
  auto cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*input, *cloud);

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  sor.filter(cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);

  // Publish the data
  pub.publish (output);
}
#endif

void create_marker(float x, float y, float z) {
    visualization_msgs::Marker sphere_list;
    sphere_list.header.frame_id = "/royale_camera_optical_frame";
    sphere_list.header.stamp = ros::Time::now();
    sphere_list.ns = "spheres";
    sphere_list.action = visualization_msgs::Marker::ADD;
    sphere_list.pose.orientation.w = 1.0;

    sphere_list.id = 0;

    sphere_list.type = visualization_msgs::Marker::SPHERE_LIST;

    // POINTS markers use x and y scale for width/height respectively
    sphere_list.scale.x = 0.1;
    sphere_list.scale.y = 0.1;
    sphere_list.scale.z = 0.1;

    // Points are green
    sphere_list.color.r = 1.0f;
    sphere_list.color.a = 1.0;

    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;

    sphere_list.points.push_back(p);

    pub_target.publish(sphere_list);
}

void
cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input) {
    // Container for original & filtered data
    auto cloud = new pcl::PCLPointCloud2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud <pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud <pcl::PointXYZ>);

    // Convert to PCL data type
    pcl_conversions::toPCL(*input, *cloud);
    pcl::fromPCLPointCloud2(*cloud, *cloud2);

    pcl::ModelCoefficients coefficients;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // Create the segmentation object
    pcl::SACSegmentation <pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(config.distance_threshold);

    seg.setInputCloud(cloud2);
    seg.segment(*inliers, coefficients);

    if (inliers->indices.empty()) {
        ROS_DEBUG("Could not estimate a planar model for the given dataset.");
    }

    // Create the filtering object
    pcl::ExtractIndices <pcl::PointXYZ> extract;

    // Extract the inliers
    extract.setInputCloud(cloud2);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_filtered);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cleaned(new pcl::PointCloud <pcl::PointXYZ>);
    // Create the filtering object
    pcl::StatisticalOutlierRemoval <pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_filtered);
    sor.setMeanK(config.mean_k);
    sor.setStddevMulThresh(config.stddev_mul_thresh);
    sor.filter(*cloud_cleaned);

    // Publish the model coefficients
    pcl_msgs::ModelCoefficients ros_coefficients;
    pcl_conversions::fromPCL(coefficients, ros_coefficients);
    pub_coeff.publish(ros_coefficients);

    // Publish point cloud
    sensor_msgs::PointCloud2 output;
    auto cloud_out = new pcl::PCLPointCloud2;
    pcl::toPCLPointCloud2(*cloud_cleaned, *cloud_out);
    pcl_conversions::fromPCL(*cloud_out, output);
    pub_cloud.publish(output);

    float min_x = 0.0f;
    float min_y = 0.0f;
    float min_z = 0.0f;
    bool found = false;
    for (const auto &point: cloud_cleaned->points) {
        if (std::isnan(point.z)) {
            continue;
        }
        if (found) {

            if (point.z < min_z) {
                min_x = point.x;
                min_y = point.y;
                min_z = point.z;
            }
        } else {
            min_x = point.x;
            min_y = point.y;
            min_z = point.z;
            found = true;
        }
    }

    if (found) {
        create_marker(min_x, min_y, min_z);
    }

    // ---------------
    // Edge detection
    // ---------------
    pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud <pcl::Normal>);
    pcl::IntegralImageNormalEstimation <pcl::PointXYZ, pcl::Normal> ne;
    ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
    ne.setNormalSmoothingSize(config.normal_smoothing_size);
    ne.setBorderPolicy(ne.BORDER_POLICY_MIRROR);
    ne.setInputCloud(cloud2);
    ne.compute(*normal);

    pcl::OrganizedEdgeFromNormals <pcl::PointXYZ, pcl::Normal, pcl::Label> oed;
    oed.setInputNormals(normal);
    oed.setInputCloud(cloud2);
    oed.setDepthDisconThreshold(config.depth_discon_threshold);
    oed.setMaxSearchNeighbors(config.max_search);
    oed.setEdgeType(oed.EDGELABEL_NAN_BOUNDARY | oed.EDGELABEL_OCCLUDING | oed.EDGELABEL_OCCLUDED |
                    oed.EDGELABEL_HIGH_CURVATURE | oed.EDGELABEL_RGB_CANNY);
    pcl::PointCloud <pcl::Label> labels;
    std::vector <pcl::PointIndices> label_indices;
    oed.compute(labels, label_indices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr occluding_edges(new pcl::PointCloud <pcl::PointXYZ>),
            occluded_edges(new pcl::PointCloud <pcl::PointXYZ>),
            nan_boundary_edges(new pcl::PointCloud <pcl::PointXYZ>),
            high_curvature_edges(new pcl::PointCloud <pcl::PointXYZ>),
            rgb_edges(new pcl::PointCloud <pcl::PointXYZ>);

    pcl::copyPointCloud(*cloud2, label_indices[0].indices, *nan_boundary_edges);
    pcl::copyPointCloud(*cloud2, label_indices[1].indices, *occluding_edges);
    pcl::copyPointCloud(*cloud2, label_indices[2].indices, *occluded_edges);
    pcl::copyPointCloud(*cloud2, label_indices[3].indices, *high_curvature_edges);
    pcl::copyPointCloud(*cloud2, label_indices[4].indices, *rgb_edges);

    sensor_msgs::PointCloud2 output_edge;
    auto cloud_edge_out = new pcl::PCLPointCloud2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr edge_source;
    switch (config.edge_type) {
        case 0:
            edge_source = nan_boundary_edges;
            break;
        case 1:
            edge_source = occluding_edges;
            break;
        case 2:
            edge_source = occluded_edges;
            break;
        case 3:
            edge_source = high_curvature_edges;
            break;
        case 4:
            edge_source = rgb_edges;
            break;
        default:
            break;
    }
    pcl::toPCLPointCloud2(*edge_source, *cloud_edge_out);
    pcl_conversions::fromPCL(*cloud_edge_out, output_edge);
    pub_edge.publish(output_edge);
}

void callback(box_detection::ConfigConfig &config_, uint32_t level) {
    config = config_;
}

int
main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "box_detection");
    ros::NodeHandle nh;

    dynamic_reconfigure::Server <box_detection::ConfigConfig> server(mutex);
    dynamic_reconfigure::Server<box_detection::ConfigConfig>::CallbackType f;
    server.getConfigDefault(config);
    server.updateConfig(config);

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("output", 1);
    pub_edge = nh.advertise<sensor_msgs::PointCloud2>("output_edge", 1);
    pub_coeff = nh.advertise<pcl_msgs::ModelCoefficients>("coefficients", 1);
    pub_target = nh.advertise<visualization_msgs::Marker>("target_marker", 1);

    // Spin
    ros::spin();
}
