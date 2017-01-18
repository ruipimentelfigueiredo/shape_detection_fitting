#include "cylinder_segmentation_hough.h"
class CylinderSegmentationROS {

	ros::Subscriber point_cloud_sub;
	ros::Subscriber cluster_sub;
	ros::NodeHandle n;
	ros::Publisher vis_pub;

	void cloud_cb (const PointCloudT::ConstPtr& input)
	{
		// Create a container for the data.
		sensor_msgs::PointCloud2 output;

		// Do data processing here...
		//output = *input;

		pcl::ModelCoefficients::Ptr model_params=cylinder_segmentation->segment(input);
		// Publish the data.
		visualization_msgs::Marker marker=createMarker(model_params,visualization_msgs::Marker::CYLINDER, input->header.frame_id);

	}

	void clusters_cb (const visualization_msgs::MarkerArray::ConstPtr& input)
	{
		// Create a container for the data.
		sensor_msgs::PointCloud2 output;

		visualization_msgs::MarkerArray markers_;
		//pcl::ModelCoefficients::Ptr model_params=cylinder_segmentation.segment(input);
		for(int i=0; i<input->markers.size();++i)
		{

    			PointCloudT::Ptr cloud_(new PointCloudT);

  			cloud_->header.frame_id = input->markers[i].header.frame_id;
  			//cloud_->height = input->markers[i].height;

			for (int p=0; p<input->markers[i].points.size();++p)
			{

				double x_=input->markers[i].points[p].x;
				double y_=input->markers[i].points[p].y;
				double z_=input->markers[i].points[p].z;
				cloud_->points.push_back (pcl::PointXYZ(x_, y_, z_));
						//pcl::PointCloudinput->markers[i].points
			}

			pcl::ModelCoefficients::Ptr model_params=cylinder_segmentation->segment(cloud_);
			visualization_msgs::Marker marker=createMarker(model_params,visualization_msgs::Marker::CYLINDER,input->markers[i].header.frame_id, i);

			markers_.markers.push_back(marker);
		}
		// Publish the data.
		vis_pub.publish( markers_ );

	}
public:
	boost::shared_ptr<CylinderSegmentationHough> cylinder_segmentation;

	CylinderSegmentationROS(ros::NodeHandle & n_) : n(n_)
	{
		unsigned int angle_bins=50;
		unsigned int radius_bins=100;
	 	unsigned int position_bins=50;
	 	float max_radius=1000.0;
		cylinder_segmentation=boost::shared_ptr<CylinderSegmentationHough>(new CylinderSegmentationHough(angle_bins,radius_bins,position_bins,max_radius));
		cluster_sub=n.subscribe<visualization_msgs::MarkerArray> ("clusters_in", 1, &CylinderSegmentationROS::clusters_cb, this);
		//point_cloud_sub=n.subscribe<pcl::PointCloud<PointT> > ("cloud_in", 1, &CylinderSegmentationROS::cloud_cb, this);
		vis_pub = n.advertise<visualization_msgs::MarkerArray>( "cylinders_markers", 0 );
	};


	visualization_msgs::Marker createMarker(const pcl::ModelCoefficients::Ptr & model_params, int model_type, const std::string & frame, int id=0)
	{

		tf::Vector3 axis_vector(model_params->values[3], model_params->values[4], model_params->values[5]);

		tf::Vector3 up_vector(0.0, 0.0, 1.0);
		tf::Vector3 right_vector = axis_vector.cross(up_vector);
		right_vector.normalized();
		tf::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
		q.normalize();
		geometry_msgs::Quaternion cylinder_orientation;
		tf::quaternionTFToMsg(q, cylinder_orientation);

		ROS_INFO_STREAM(frame);
		visualization_msgs::Marker marker;
		marker.header.frame_id =frame;
		marker.header.stamp = ros::Time();
		marker.ns = "model";
		marker.id = id;
		marker.type = model_type;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = model_params->values[0];
		marker.pose.position.y = model_params->values[1];
		marker.pose.position.z = model_params->values[2];
		marker.pose.orientation = cylinder_orientation;
/*		marker.pose.orientation.x = Q.x();
		marker.pose.orientation.y = Q.y();
		marker.pose.orientation.z = Q.z();
		marker.pose.orientation.w = Q.w();*/
		marker.scale.x = model_params->values[6];
		marker.scale.y = model_params->values[6];
		marker.scale.z = model_params->values[7];
		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
		marker.lifetime = ros::Duration(0.1);
		return marker;
	}
};

