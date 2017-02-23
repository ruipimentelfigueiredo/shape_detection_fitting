#ifndef CYLINDERSEGMENTATIONROSNODELET_H
#define CYLINDERSEGMENTATIONROSNODELET_H
#include <nodelet/nodelet.h>
#include "cylinder_segmentation_hough.h"
#include "cylinder_segmentation_ros.h"

namespace active_semantic_mapping
{
class CylinderSegmentationRosNodelet: public nodelet::Nodelet
{

public:
    CylinderSegmentationRosNodelet(){}
    ~CylinderSegmentationRosNodelet(){}
    virtual void onInit();
    boost::shared_ptr<CylinderSegmentationROS<CylinderSegmentationHough> > inst_;

};

}


#endif // CYLINDERSEGMENTATIONROSNODELET_H
