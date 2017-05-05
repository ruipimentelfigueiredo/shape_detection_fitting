#include "cylinder_segmentation_ros.h"

int main (int argc, char** argv)
{
	ros::init(argc, argv, "talker");

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/

	ros::NodeHandle n;
	ros::NodeHandle n_priv("~");
	ros::Rate loop_rate(300);

	bool use_ransac;

	// Common params
 	double min_radius;
 	double max_radius;

    	n_priv.param("use_ransac",use_ransac,true);
	ROS_INFO_STREAM("use_ransac: "<< use_ransac);
	if (use_ransac)
		CylinderSegmentationROS<CylinderSegmentationRansac> cylinder_segmentation_ros(n, n_priv);
	else
		CylinderSegmentationROS<CylinderSegmentationHough> cylinder_segmentation_ros(n, n_priv);
	
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}


  
	return (0);
}

