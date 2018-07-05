#include "nodelets/cylinder_tracking_ros_nodelet.h"

namespace shape_detection_fitting
{
    void CylinderTrackingRosNodelet::onInit()
    {
        NODELET_INFO("Initializing nodelet");
        inst_.reset(new CylinderTrackingROS(getNodeHandle()));
    }
}

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(shape_detection_fitting::CylinderTrackingRosNodelet,nodelet::Nodelet)
