#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/common/transforms.h>
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
#include <pcl/filters/project_inliers.h>
#include <Eigen/Geometry>
#include <tf/tf.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv/cv.h>
#include <opencv/highgui.h>

// DEFINE THE TYPES USED
typedef pcl::PointXYZ PointT;
typedef pcl::Normal NormalT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<NormalT> NormalCloudT;
//pcl::visualization::CloudViewer viewer("Cloud Viewer");


class Cylinder
{
	Cylinder(pcl::PointIndices::Ptr inliers_,pcl::ModelCoefficients::Ptr coefficients_)
		: inliers(inliers_),
		  coefficients(coefficients_)
 	{}
	pcl::PointIndices::Ptr inliers;
	pcl::ModelCoefficients::Ptr coefficients;
};

class CylinderSegmentationHough
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;



	// private attributes
	unsigned int gaussian_sphere_points_num;
	unsigned int angle_bins;
	unsigned int radius_bins;
	unsigned int position_bins;
	float min_radius;
	float max_radius;
	float angle_step;
	float r_step;
	std::vector<float> cyl_direction_accum;
	std::vector<Eigen::Vector3f> gaussian_sphere_points;
	std::vector<std::vector<std::vector<unsigned int> > > cyl_circ_accum;
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
	pcl::PointCloud<PointT>::Ptr cloud_filtered;// (new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;// (new pcl::PointCloud<pcl::Normal>);
	pcl::PassThrough<PointT> pass;
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	pcl::search::KdTree<PointT>::Ptr tree; 
	pcl::PointIndices::Ptr inliers_cylinder;// (new pcl::PointIndices);

	// private methods
	Eigen::Vector3f findCylinderDirection(const NormalCloudT::ConstPtr & cloud_normals);
	Eigen::Matrix<float,5,1> findCylinderPositionRadius(const PointCloudT::ConstPtr & point_cloud_in_);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (PointCloudT::ConstPtr cloud,  pcl::PointCloud<pcl::Normal>::ConstPtr normals, pcl::ModelCoefficients::Ptr coefficients_cylinder);
	public:
		CylinderSegmentationHough(unsigned int angle_bins_=30,unsigned int radius_bins_=10,unsigned int position_bins_=10,float min_radius_=0.01,float max_radius_=0.1, unsigned int gaussian_sphere_points_num_=900);
		pcl::ModelCoefficients::Ptr segment(const PointCloudT::ConstPtr & point_cloud_in_);
};

