#include "visualization_msgs/MarkerArray.h"
#include "multiple_tracker_manager.h"
#include "sensor_msgs/CameraInfo.h"
#include "active_semantic_mapping/Cylinders.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32MultiArray.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

class Color
{

	public:
	Color(double r_=0.0,double g_=0.0, double b_=0.0, double a_=1.0) : r(r_),g(g_),b(b_),a(a_)
	{}
	double r,g,b,a;
};

class CylinderTrackingROS {


	ros::Time odom_last_stamp;
	std::string odom_link;
	std::string tracking_frame;
	ros::Time filter_time;
	tf::StampedTransform meas_old_;
	bool vo_initialized_;
	tf::Transform filter_estimate_old_;
	Eigen::Matrix <double,6,1> filter_estimate_old_vec_;
	const std::string marker_detections_namespace_ ="detections";
	const std::string marker_trackers_namespace_ ="trackers";

	 Eigen::Matrix4d cam_intrinsic;

	std::map<int, Color> id_colors_map;
	MultipleTrackerManager tracker_manager;
	
	
	ros::NodeHandle n;
	ros::NodeHandle n_priv;
	boost::shared_ptr<tf::TransformListener> listener;

	ros::Subscriber odom_sub;
	ros::Publisher vis_pub;
	

	ros::Subscriber cylinders_sub;
	/*typedef message_filters::sync_policies::ApproximateTime<std_msgs::Int32MultiArray> MySyncPolicy;

	boost::shared_ptr<message_filters::Subscriber<std_msgs::Int32MultiArray> > cylinders_sub;
    	boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy> >sync;*/

  	void callback (const active_semantic_mapping::Cylinders::ConstPtr & input_clusters);
  	void odomCallback (const nav_msgs::Odometry::ConstPtr & odom);
	bool addMeasurement(const tf::StampedTransform& meas, const Eigen::Matrix<double, 6, 6> & covar);
	void decomposeTransform(const tf::Transform& trans, double& x, double& y, double&z, double&Rx, double& Ry, double& Rz);
	void angleOverflowCorrect(double& a, double ref);
	void initialize(const tf::Transform& prior, const ros::Time& time);
	//Eigen::Matrix4f  odom(const std::vector<Eigen::VectorXd> & detections_);
public:

	CylinderTrackingROS(ros::NodeHandle & n_);
	visualization_msgs::Marker createMarker(const Eigen::VectorXd & model_params, int model_type, const std::string & frame, Color & color_, int id, const std::string & marker_namespace_);
};

