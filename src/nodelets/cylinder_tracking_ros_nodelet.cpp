#include "nodelets/cylinder_tracking_ros_nodelet.h"

namespace active_semantic_mapping
{
    void CylinderTrackingRosNodelet::onInit()
    {
        NODELET_INFO("Initializing nodelet");
        inst_.reset(new CylinderTrackingROS(getNodeHandle()));
    }
}

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(active_semantic_mapping::CylinderTrackingRosNodelet,nodelet::Nodelet)
