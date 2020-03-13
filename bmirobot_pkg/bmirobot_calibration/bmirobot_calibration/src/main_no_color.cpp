#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
ros::Publisher pub;
ros::Publisher pub_coff;
void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*input, *cloud);

const float depth_limit = 1.5;
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud ((cloud));
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, depth_limit);
  pass.filter (*cloud);


  pcl::ModelCoefficients coefficients;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setModelType (pcl::SACMODEL_SPHERE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setRadiusLimits(0.03,0.04);
  seg.setMaxIterations(5000000);

  seg.setDistanceThreshold (0.01);


  seg.setInputCloud (cloud->makeShared ());
  seg.segment (*inliers, coefficients); 

/*
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);


  seg.setInputCloud (cloud->makeShared ());
  seg.segment (*inliers, coefficients); 
  
*/
  // Publish the model coefficients
  pcl_msgs::ModelCoefficients ros_coefficients;
  pcl_conversions::fromPCL(coefficients, ros_coefficients);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
// Extract the inliers
if(inliers->indices.size () >0)
{
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud (cloud->makeShared ());
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud_filtered, output);


  pub.publish (output);
pcl_msgs::ModelCoefficients ros_coefficients;
  pcl_conversions::fromPCL(coefficients, ros_coefficients);
  pub_coff.publish(ros_coefficients);
  //std::cout << "found the calibration ball";
}
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);
//ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, cloud_cb);

  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<sensor_msgs::PointCloud2> ("reco_sphere", 1);
  pub_coff = nh.advertise<pcl_msgs::ModelCoefficients> ("reco_sphere_coffi", 1);


  // Spin
  ros::spin ();
}
