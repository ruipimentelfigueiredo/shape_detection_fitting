#include "nodelets/cylinder_segmentation_ros_nodelet.h"

namespace active_semantic_mapping
{
    void CylinderSegmentationRosNodelet::onInit()
    {
	ros::NodeHandle n_priv=getPrivateNodeHandle();
	bool use_ransac;

    	n_priv.param("use_ransac",use_ransac,true);
	ROS_INFO_STREAM("use_ransac: "<< use_ransac);

	if (use_ransac)
	{
		inst_ransac_.reset(new CylinderSegmentationROS<CylinderSegmentationRansac>(getNodeHandle(),getPrivateNodeHandle()));
	}
	else
	{
		inst_hough_.reset(new  CylinderSegmentationROS<CylinderSegmentationHough>(getNodeHandle(),getPrivateNodeHandle()));
	}
    }
}

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(active_semantic_mapping::CylinderSegmentationRosNodelet,nodelet::Nodelet)
