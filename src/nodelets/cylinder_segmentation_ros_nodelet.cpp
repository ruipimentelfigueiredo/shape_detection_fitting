#include "nodelets/cylinder_segmentation_ros_nodelet.h"

namespace active_semantic_mapping
{
    void CylinderSegmentationRosNodelet::onInit()
    {
	boost::shared_ptr<CylinderSegmentationHough> cylinder_segmentation_;
        NODELET_INFO("Initializing nodelet");
        inst_.reset(new CylinderSegmentationROS<CylinderSegmentationHough>(getNodeHandle(),cylinder_segmentation_));
    }
}

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(active_semantic_mapping::CylinderSegmentationRosNodelet,nodelet::Nodelet)
