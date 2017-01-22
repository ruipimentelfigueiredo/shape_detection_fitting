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

  	/*static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(0, 0, 0.0) );
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));*/
	// Build a passthrough filter to remove spurious NaNs
	/*ROS_INFO_STREAM(" 1. Filter");
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0, 100000000);
	pass.filter (*cloud);
	std::cerr << "PointCloud after filtering has: " << cloud->points.size () << " data points." << std::endl;

	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);*/

	Eigen::Vector4f min_pt,max_pt;
	pcl::getMinMax3D(*cloud,min_pt,max_pt);

	while (ros::ok())
	{
		Eigen::Vector3f rot_dir;
		cv::Mat aux(1, 1, CV_32F);

		// Generate random orientation
		cv::randn(aux, 0.0, 1.0);
		rot_dir(0,0)=aux.at<float>(0,0);

		cv::randn(aux, 0.0, 1.0);
		rot_dir(1,0)=aux.at<float>(0,0);

		cv::randn(aux, 0.0, 1.0);
		rot_dir(2,0)=aux.at<float>(0,0);

		rot_dir.normalize();

		cv::Mat angle(1, 1, CV_32F);
		cv::randn(angle, 0.0, 1.0);
		//Eigen::Vector3f rot_dir(1.0,0.0,0.0);
		rot_dir.normalize();
		Eigen::Matrix3f rot;
		rot=Eigen::AngleAxisf(M_PI*angle.at<float>(0,0),rot_dir);

		Eigen::Matrix4f transf;
		transf.block(0,0,3,3)=rot;
		transf.block(0,3,3,1)=Eigen::Vector3f(0,0,0);
		pcl::PointCloud<PointT>::Ptr cloud_transf (new pcl::PointCloud<PointT>);
		pcl::transformPointCloud (*cloud, *cloud_transf, transf);
	
		// Corrupt with noise

		ros::spinOnce();
		loop_rate.sleep();
		cloud_pub.publish(cloud_transf);
	}


	// Segment cylinder





	return (0);
}
