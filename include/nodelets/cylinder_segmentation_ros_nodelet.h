#ifndef CYLINDERSEGMENTATIONROSNODELET_H
#define CYLINDERSEGMENTATIONROSNODELET_H
#include <nodelet/nodelet.h>
#include "cylinder_segmentation_hough.h"
#include "cylinder_segmentation_ransac.h"
#include "cylinder_segmentation_ros.h"

namespace shape_detection_fitting
{
class CylinderSegmentationRosNodelet: public nodelet::Nodelet
{

public:
    CylinderSegmentationRosNodelet(){}
    ~CylinderSegmentationRosNodelet(){}
    virtual void onInit();
    boost::shared_ptr<CylinderSegmentationROS<CylinderSegmentationHough> > inst_hough_;
    boost::shared_ptr<CylinderSegmentationROS<CylinderSegmentationRansac> > inst_ransac_;
};

}


#endif // CYLINDERSEGMENTATIONROSNODELET_H
