#include "cylinder_segmentation_hough.h"
#include <ctime>
#include <tf/transform_broadcaster.h>
#include <rosbag/view.h>
#include <active_semantic_mapping/Cylinders.h>
#include <rosbag/bag.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include "cylinder_segmentation_hough.h"
#include "cylinder_segmentation_ransac.h"
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

	// For visualization purposes
	ros::Publisher cloud_pub;
	cloud_pub=n.advertise<pcl::PointCloud<PointT> >( "cylinders_pcl", 0 );




	unsigned int angle_bins=50;
	unsigned int radius_bins=100;
 	unsigned int position_bins=50;

	std::ostringstream ss;
	ss << "/home/rui/rosbags/";
    	boost::filesystem::create_directories(ss.str());
	ss << std::fixed;
	ss << "angle_bins_";
	ss << angle_bins;
	ss << "_radius_bins_";
	ss << radius_bins;
	ss << "_position_bins_";
	ss << position_bins;
	ss << ".bag";

	std::string rosbag_file;
	rosbag_file = ss.str();




	rosbag::Bag bag;

	bag.open(rosbag_file, rosbag::bagmode::Read);



	// TESTING PARAMS
 	float min_radius=0.01;
 	float max_radius=0.1;

	int height_samples=30;
	int angle_samples=30;

	float height=0.3;
	float radius=0.05;

	std::vector<float> noise_levels; // percentage of object size (std_dev)
	noise_levels.push_back(0);
	noise_levels.push_back(0.1);
	noise_levels.push_back(0.2);
	noise_levels.push_back(0.3);
	noise_levels.push_back(0.4);
	noise_levels.push_back(0.5);
	


	// First, generate 200 point clouds with different radius, heights at random poses
	unsigned int iterations=200;
    	std::vector<std::string> topics;
    	topics.push_back(std::string("point_cloud"));
    	topics.push_back(std::string("ground_truth"));
    	rosbag::View view(bag, rosbag::TopicQuery(topics));

	std::vector<active_semantic_mapping::Cylinders> ground_truths;
    	foreach(rosbag::MessageInstance const m, view)
	{	

		active_semantic_mapping::Cylinders::ConstPtr s = m.instantiate<active_semantic_mapping::Cylinders>();
		if (s == NULL)
			continue;
		ROS_INFO("YOH");
		ground_truths.push_back(*s);
	}
	bag.close();


	bag.open(rosbag_file, rosbag::bagmode::Read);
    	rosbag::View view2(bag, rosbag::TopicQuery(topics));

	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > point_clouds;
    	foreach(rosbag::MessageInstance const m, view2)
	{

		pcl::PointCloud<pcl::PointXYZ>::ConstPtr s = m.instantiate<pcl::PointCloud<pcl::PointXYZ> >();
		if (s == NULL)
			continue;
		ROS_INFO("YAH");
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		*cloud=*s;
		point_clouds.push_back(cloud);
	}

	bag.close();

	// Segment cylinder
	boost::shared_ptr<CylinderSegmentationHough> cylinder_segmentation;
	for(int i=0;i<point_clouds.size();++i)
	{
			Eigen::VectorXf model_params=cylinder_segmentation->segment(clusters_point_clouds[i]);
	}




	return (0);
}
