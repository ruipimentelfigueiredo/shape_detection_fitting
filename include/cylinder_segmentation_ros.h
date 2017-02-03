#include "cylinder_segmentation_hough.h"
#include "cylinder_segmentation_ransac.h"
#include "visualization_msgs/MarkerArray.h"
#include "multiple_tracker_manager.h"
#include "sensor_msgs/CameraInfo.h"
#include "active_semantic_mapping/Clusters.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>


class Color
{

	public:
	Color(double r_=0.0,double g_=0.0, double b_=0.0, double a_=1.0) : r(r_),g(g_),b(b_),a(a_)
	{}
	double r,g,b,a;
};

template <class detector_type>
class CylinderSegmentationROS {

	const std::string marker_detections_namespace_ ="detections";
	const std::string marker_trackers_namespace_ ="trackers";

	 Eigen::Matrix4f cam_intrinsic;

	std::map<int, Color> id_colors_map;
	MultipleTrackerManager tracker_manager;
	
	
	ros::NodeHandle n;
	ros::NodeHandle n_priv;
	ros::Subscriber point_cloud_sub;



	ros::Publisher vis_pub;
	ros::Publisher cloud_pub;
	ros::Publisher image_pub;

	// Aux pub
	ros::Publisher cluster_pub;
	ros::Subscriber cluster_sub;


	void cloud_cb (const PointCloudT::ConstPtr& input);
	void clusters_cb (const visualization_msgs::MarkerArray::ConstPtr& input);

	boost::shared_ptr<detector_type> cylinder_segmentation;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, active_semantic_mapping::Clusters> MySyncPolicy;
	boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> > image_sub;
	boost::shared_ptr<message_filters::Subscriber<active_semantic_mapping::Clusters> > clusters_sub;
    	boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy> >sync;

  	void callback (const sensor_msgs::Image::ConstPtr& input_image, const active_semantic_mapping::Clusters::ConstPtr & input_clusters);

public:

	CylinderSegmentationROS(ros::NodeHandle & n_, boost::shared_ptr<detector_type> & cylinder_segmentation_);
	visualization_msgs::Marker createMarker(const Eigen::VectorXf & model_params, int model_type, const std::string & frame, Color & color_, int id, const std::string & marker_namespace_);
};

