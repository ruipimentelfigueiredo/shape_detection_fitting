#include "cylinder_segmentation_hough.h"
#include "cylinder_segmentation_ransac.h"
#include "visualization_msgs/MarkerArray.h"
#include "multiple_tracker_manager.h"


class Color
{

	public:
	Color(double r_=0.0,double g_=0.0, double b_=0.0, double a_=1.0) : r(r_),g(g_),b(b_),a(a_)
	{}
	double r,g,b,a;
};

template <class detector_type>
class CylinderSegmentationROS {

	std::map<int, Color> id_colors_map;
	MultipleTrackerManager tracker_manager;
	

	ros::NodeHandle n;
	ros::NodeHandle n_priv;
	ros::Subscriber point_cloud_sub;
	ros::Subscriber cluster_sub;

	ros::Publisher vis_pub;
	ros::Publisher cloud_pub;
	void cloud_cb (const PointCloudT::ConstPtr& input);
	void clusters_cb (const visualization_msgs::MarkerArray::ConstPtr& input);

	boost::shared_ptr<detector_type> cylinder_segmentation;
public:

	CylinderSegmentationROS(ros::NodeHandle & n_, boost::shared_ptr<detector_type> & cylinder_segmentation_);
	visualization_msgs::Marker createMarker(const Eigen::VectorXf & model_params, int model_type, const std::string & frame, Color & color_, int id=0, std::string & marker_namespace_="det");
};

