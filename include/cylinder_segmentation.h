#ifndef CYLINDERSEGMENTATION_H
#define CYLINDERSEGMENTATION_H
#include <pcl_ros/point_cloud.h>

#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/registration/icp.h>

#include <Eigen/Geometry>
#include <tf/tf.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <pcl/features/principal_curvatures.h>
// DEFINE THE TYPES USED
typedef pcl::PointXYZ PointT;
typedef pcl::Normal NormalT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<NormalT> NormalCloudT;

class CylinderSegmentation
{
	protected:
	// params
	float min_radius;
	float max_radius;

	// refine estimation
	bool do_refine;

	//aux structures
	// Normal estimation
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;// (new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	pcl::search::KdTree<PointT>::Ptr tree; 

	PointCloudT::Ptr cloud_filtered;// (new pcl::PointCloud<PointT>);
	PointCloudT::Ptr transformed_cloud;// (new pcl::PointCloud<PointT> ());
	pcl::PointIndices::Ptr inliers_cylinder;// (new pcl::PointIndices);

	pcl::PassThrough<PointT> pass;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (PointCloudT::ConstPtr cloud,  pcl::PointCloud<pcl::Normal>::ConstPtr normals, pcl::ModelCoefficients::Ptr coefficients_cylinder);
	public:
		CylinderSegmentation(float min_radius_=0.01,float max_radius_=0.1, bool do_refine_=false) : 
			min_radius(min_radius_), 
			max_radius(max_radius_), 
			do_refine(do_refine_),
			cloud_normals(new pcl::PointCloud<pcl::Normal>),
			tree(new pcl::search::KdTree<PointT> ()),	
			cloud_filtered(new pcl::PointCloud<PointT>),
			transformed_cloud(new pcl::PointCloud<PointT> ()),
			inliers_cylinder(new pcl::PointIndices)
		{};

		Eigen::Matrix4f refine(const PointCloudT::ConstPtr & point_cloud_source_, const PointCloudT::ConstPtr & point_cloud_target_);
};

#endif // CYLINDERSEGMENTATION_H

