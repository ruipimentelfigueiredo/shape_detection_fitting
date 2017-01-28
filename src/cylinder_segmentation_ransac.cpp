#include "cylinder_segmentation_ransac.h"
CylinderSegmentationRansac::CylinderSegmentationRansac(float normal_distance_weight_, unsigned int max_iterations_, float distance_threshold_, float min_radius_,float max_radius_, bool do_refine_) :
	CylinderSegmentation(min_radius_,max_radius_,do_refine_),
	normal_distance_weight(normal_distance_weight_),
	max_iterations(max_iterations_),
	distance_threshold(distance_threshold_)
{};

pcl::ModelCoefficients::Ptr CylinderSegmentationRansac::segment(const PointCloudT::ConstPtr & point_cloud_in_)
{
	// Estimate point normals
	ne.setSearchMethod (tree);
	ne.setInputCloud (point_cloud_in_);
	ne.setKSearch (50);
	ne.compute (*cloud_normals);

	// Create the segmentation object for cylinder segmentation and set all the parameters
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_CYLINDER);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight (normal_distance_weight);
	seg.setMaxIterations (max_iterations);
	seg.setDistanceThreshold (distance_threshold);
	seg.setRadiusLimits (min_radius, max_radius);
	seg.setInputCloud (point_cloud_in_);
	seg.setInputNormals (cloud_normals);

	// Obtain the cylinder inliers and coefficients
	pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
	seg.segment (*inliers_cylinder, *coefficients_cylinder);
	std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

	return coefficients_cylinder;
}




