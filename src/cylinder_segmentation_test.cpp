#include "cylinder_segmentation_hough.h"
#include <ctime>



int main (int argc, char** argv)
{

	unsigned int angle_bins=50;
	unsigned int radius_bins=100;
 	unsigned int position_bins=50;
 	float max_radius=1000.0;

	pcl::PassThrough<PointT> pass;
	pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
	// Read in the cloud data
        pcl::PCDReader reader;
	reader.read (argv[1], *cloud);
	std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;


	// Build a passthrough filter to remove spurious NaNs
	ROS_INFO_STREAM(" 1. Filter");
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0, 100000000);
	pass.filter (*cloud);
	std::cerr << "PointCloud after filtering has: " << cloud->points.size () << " data points." << std::endl;

	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

	///// REMOVE PLANE
	// Estimate point normals
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

	ne.setSearchMethod (tree);
	ne.setInputCloud (cloud);
	ne.setKSearch (50);
	ne.compute (*cloud_normals);
	// Create the segmentation object for the planar model and set all the parameters

	pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
	pcl::ExtractIndices<PointT> extract;

	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
	seg.setNormalDistanceWeight (0.1);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (0.03);
	seg.setInputCloud (cloud);
	seg.setInputNormals (cloud_normals);
	// Obtain the plane inliers and coefficients
	seg.segment (*inliers_plane, *coefficients_plane);
	std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

	// Extract the planar inliers from the input cloud
	extract.setInputCloud (cloud);
	extract.setIndices (inliers_plane);
	extract.setNegative (false);


	// Remove the planar inliers, extract the rest
	extract.setNegative (true);
	extract.filter (*cloud);

	// Recompute normals
	ne.setSearchMethod (tree);
	ne.setInputCloud (cloud);
	ne.setKSearch (50);
	ne.compute (*cloud_normals);


	// Segment cylinder
	CylinderSegmentationHough segmentation_obj(angle_bins, radius_bins, position_bins, max_radius);
const clock_t begin_time = clock();

        pcl::ModelCoefficients::Ptr cyl_parameters=segmentation_obj.segment(cloud);
std::cout << float( clock () - begin_time ) /  CLOCKS_PER_SEC<< "seconds"<<std::endl;

	std::cout << "done" << std::endl;

	return (0);
}
