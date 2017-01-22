#include "cylinder_segmentation_hough.h"
class CylinderSegmentationROS {
	ros::NodeHandle n;
	ros::NodeHandle n_priv;
	ros::Subscriber point_cloud_sub;
	//ros::Subscriber cluster_sub;

	ros::Publisher vis_pub;
	ros::Publisher cloud_pub;
	void cloud_cb (const PointCloudT::ConstPtr& input);
	void clusters_cb (const visualization_msgs::MarkerArray::ConstPtr& input);
public:
	boost::shared_ptr<CylinderSegmentationHough> cylinder_segmentation;

	CylinderSegmentationROS(ros::NodeHandle & n_);

	visualization_msgs::Marker createMarker(const pcl::ModelCoefficients::Ptr & model_params, int model_type, const std::string & frame, int id=0);
};

