#include "cylinder_segmentation.h"
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/ransac.h>

typedef pcl::PointXYZ PointT;
typedef pcl::Normal NormalT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<NormalT> NormalCloudT;
//pcl::visualization::CloudViewer viewer("Cloud Viewer");

class CylinderSegmentationRansac : public CylinderSegmentation
{
	// Parameters
	float normal_distance_weight; 
	unsigned int max_iterations; 
	float distance_threshold;

	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;

	public:
		CylinderSegmentationRansac(float normal_distance_weight_=0.1, unsigned int max_iterations_=1000, float distance_threshold_=0.1, float min_radius_=0.01,float max_radius_=0.1, bool do_refine_=false);
		pcl::ModelCoefficients::Ptr segment(const PointCloudT::ConstPtr & point_cloud_in_);
};



