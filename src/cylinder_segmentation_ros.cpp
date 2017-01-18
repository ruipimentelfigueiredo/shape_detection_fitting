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
	ros::Rate loop_rate(30);

	CylinderSegmentationROS cylinder_segmentation_ros(n);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
  
	return (0);
}

