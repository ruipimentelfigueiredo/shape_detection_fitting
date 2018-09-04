/*
Copyright 2018 Rui Miguel Horta Pimentel de Figueiredo

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*!    
    \author Rui Figueiredo : ruipimentelfigueiredo
*/

#ifndef SHAPEFITTINGROSNODELET_H
#define SHAPEFITTINGROSNODELET_H
#include <nodelet/nodelet.h>

#include "cylinder_fitting_hough.h"
#include "cylinder_fitting_ransac.h"

#include "sphere_fitting_hough.h"

#include "shape_fitting_ros.h"

namespace shape_detection_fitting
{
class ShapeDetectionFittingRosNodelet: public nodelet::Nodelet
{

public:
    ShapeDetectionFittingRosNodelet(){}
    ~ShapeDetectionFittingRosNodelet(){}
    virtual void onInit();


    //template <class cylinder_detector_type, class sphere_detector_type>
    //boost::shared_ptr<ShapeFittingROS<cylinder_detector_type,sphere_detector_type> > inst_fitting_;


    boost::shared_ptr<ShapeFittingROS<CylinderFittingRansac,SphereFittingHough> > inst_ransac_fitting_;
    boost::shared_ptr<ShapeFittingROS<CylinderFittingHough,SphereFittingHough> > inst_hough_fitting_;


};

}


#endif // SHAPEFITTINGROSNODELET_H
