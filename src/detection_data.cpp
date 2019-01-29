#include "detection_data.h"

DetectionData::CvColormap DetectionData::id_colors_map_bb = { 
    { std::pair<int,cv::Scalar>(0,cv::Scalar(0,  255, 0) ) },
    { std::pair<int,cv::Scalar>(0,cv::Scalar(0,  0, 255) ) }
}; 
