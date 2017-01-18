#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/ransac.h>
#include <Eigen/Geometry>
#include <tf/tf.h>
typedef pcl::PointXYZ PointT;
typedef pcl::Normal NormalT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<NormalT> NormalCloudT;
//pcl::visualization::CloudViewer viewer("Cloud Viewer");

class CylinderSegmentationRansac
{
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;


    pcl::PointCloud<PointT>::Ptr cloud_filtered;// (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;// (new pcl::PointCloud<pcl::Normal>);
    pcl::PassThrough<PointT> pass;
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::search::KdTree<PointT>::Ptr tree; 
    pcl::PointIndices::Ptr inliers_cylinder;// (new pcl::PointIndices);
public:
    CylinderSegmentationRansac() : 
	//coefficients_cylinder(new pcl::ModelCoefficients),
	cloud_filtered(new pcl::PointCloud<PointT>),
	cloud_normals(new pcl::PointCloud<pcl::Normal>),
	tree(new pcl::search::KdTree<PointT> ()),
	inliers_cylinder(new pcl::PointIndices)
    {};

    pcl::ModelCoefficients::Ptr segment(const PointCloudT::ConstPtr & point_cloud_in_);
};



