#include "cylinder_segmentation_hough.h"
#include <ctime>
#include <tf/transform_broadcaster.h>



int main (int argc, char** argv)
{
	ros::init(argc, argv, "cylinder_publisher");

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/

	ros::NodeHandle n;
	ros::Rate loop_rate(30);
	unsigned int angle_bins=50;
	unsigned int radius_bins=100;
 	unsigned int position_bins=50;

 	float mix_radius=10.0;
 	float max_radius=1000.0;

	ros::Publisher cloud_pub;
	cloud_pub=n.advertise<pcl::PointCloud<PointT> >( "cylinders_pcl", 0 );
	pcl::PassThrough<PointT> pass;
	pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
	// Read in the cloud data
        pcl::PCDReader reader;
	reader.read (argv[1], *cloud);
	cloud->header.frame_id="world";
	std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;

	Eigen::Vector3f rot_dir(0.5,0.5,0.5);
	rot_dir.normalize();
	Eigen::Matrix3f rot;
	rot=Eigen::AngleAxisf(M_PI*0.25,rot_dir);
	
	Eigen::Matrix4f transf;
	transf.block(0,0,3,3)=rot;
	pcl::transformPointCloud (*cloud, *cloud, transf);
  	/*static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(0, 0, 0.0) );
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));*/
	// Build a passthrough filter to remove spurious NaNs
	ROS_INFO_STREAM(" 1. Filter");
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0, 100000000);
	pass.filter (*cloud);
	std::cerr << "PointCloud after filtering has: " << cloud->points.size () << " data points." << std::endl;

	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
		cloud_pub.publish(cloud);
	}

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
	CylinderSegmentationHough segmentation_obj(angle_bins, radius_bins, position_bins, mix_radius, max_radius);
	const clock_t begin_time = clock();

        //pcl::ModelCoefficients::Ptr cyl_parameters=segmentation_obj.segment(cloud);
	std::cout << float( clock () - begin_time ) /  CLOCKS_PER_SEC<< "seconds"<<std::endl;

	std::cout << "done" << std::endl;

	return (0);
}
