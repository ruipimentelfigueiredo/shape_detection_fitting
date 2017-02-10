#include "cylinder_segmentation_hough.h"
#include <ctime>
#include <tf/transform_broadcaster.h>
#include <rosbag/view.h>
#include <active_semantic_mapping/Cylinders.h>
#include <rosbag/bag.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include "cylinder_segmentation_hough.h"
#include "cylinder_segmentation_ransac.h"

#include <visualization_msgs/MarkerArray.h>





visualization_msgs::Marker createMarker(const Eigen::VectorXf & model_params, int model_type, const std::string & frame, int id, const std::string & marker_namespace_)
{
	// Convert direction vector to quaternion
	tf::Vector3 axis_vector(model_params[3], model_params[4], model_params[5]);

	tf::Vector3 up_vector(0.0, 0.0, 1.0);
	tf::Quaternion q;
	if(axis_vector.dot(up_vector)>0.99)
	{
		q=tf::createIdentityQuaternion();
	}
	else
	{
		tf::Vector3 right_vector = axis_vector.cross(up_vector);
		right_vector.normalized();

		q=tf::Quaternion(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
	}

	q.normalize();
	geometry_msgs::Quaternion cylinder_orientation;
	tf::quaternionTFToMsg(q, cylinder_orientation);
	float height=model_params[7];

	visualization_msgs::Marker marker;
	marker.header.frame_id =frame;
	marker.header.stamp = ros::Time();
	marker.ns = marker_namespace_;
	marker.id = id;
	marker.type = model_type;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = model_params[0];
	marker.pose.position.y = model_params[1];
	marker.pose.position.z = model_params[2];
	marker.pose.orientation = cylinder_orientation;
/*		marker.pose.orientation.x = Q.x();
	marker.pose.orientation.y = Q.y();
	marker.pose.orientation.z = Q.z();
	marker.pose.orientation.w = Q.w();*/
	marker.scale.x = 2*model_params[6];
	marker.scale.y = 2*model_params[6];
	marker.scale.z = height;
	marker.color.a = 0.5;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	//marker.lifetime = ros::Duration(0.05);
	return marker;
}



int main (int argc, char** argv)
{
	ros::init(argc, argv, "cylinder_publisher");

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/

	ros::NodeHandle n;
	ros::Rate loop_rate(30);

	// For visualization purposes
	ros::Publisher cloud_pub;
	cloud_pub=n.advertise<pcl::PointCloud<PointT> >( "cylinders_pcl", 0 );


	ros::Publisher detection_pub;
	detection_pub=n.advertise<visualization_msgs::MarkerArray>( "detections", 0 );

	unsigned int angle_bins=30;
	unsigned int radius_bins=10;
 	unsigned int position_bins=10;

	int gaussian_sphere_points_num=600;
	std::ostringstream ss;
	ss << "/home/rui/rosbags/";
    	boost::filesystem::create_directories(ss.str());
	ss << std::fixed;
	ss << "angle_bins_";
	ss << angle_bins;
	ss << "_radius_bins_";
	ss << radius_bins;
	ss << "_position_bins_";
	ss << position_bins;


	std::string rosbag_file;
	rosbag_file = ss.str()+".bag";

	rosbag::Bag bag;

	bag.open(rosbag_file, rosbag::bagmode::Read);



	// TESTING PARAMS
 	float min_radius=0.1;
 	float max_radius=0.2;

	int height_samples=30;
	int angle_samples=30;

	float height=0.3;
	float radius=0.05;

	std::vector<float> noise_levels; // percentage of object size (std_dev)

	for(unsigned int i=0; i<=20; ++i)
	{
		float noise_level=(float)0.05*i;
		noise_levels.push_back(noise_level);
	}
	
	
position_bins=100;

	while(ros::ok())
	{
		// First, generate 200 point clouds with different radius, heights at random poses
		unsigned int iterations=200;
	    	std::vector<std::string> topics;
	    	topics.push_back(std::string("point_cloud"));
	    	topics.push_back(std::string("ground_truth"));
	    	rosbag::View view(bag, rosbag::TopicQuery(topics));

		std::vector<active_semantic_mapping::Cylinders> ground_truths;
	    	foreach(rosbag::MessageInstance const m, view)
		{	

			active_semantic_mapping::Cylinders::ConstPtr s = m.instantiate<active_semantic_mapping::Cylinders>();
			if (s == NULL)
				continue;

			ground_truths.push_back(*s);
		}
		bag.close();


		bag.open(rosbag_file, rosbag::bagmode::Read);
	    	rosbag::View view2(bag, rosbag::TopicQuery(topics));

		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > point_clouds;
	    	foreach(rosbag::MessageInstance const m, view2)
		{

			pcl::PointCloud<pcl::PointXYZ>::ConstPtr s = m.instantiate<pcl::PointCloud<pcl::PointXYZ> >();
			if (s == NULL)
				continue;

			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
			*cloud=*s;
			point_clouds.push_back(cloud);
		}

		bag.close();


		std::string detections_frame_id="world";
		std::string marker_detections_namespace_="detections";
		// Segment cylinder and store results
		std::vector<Eigen::VectorXf > detections;
		std::vector<float> position_errors;
		boost::shared_ptr<CylinderSegmentationHough> cylinder_segmentation(new CylinderSegmentationHough((unsigned int)angle_bins,(unsigned int)radius_bins,(unsigned int)position_bins,(float)min_radius, (float)max_radius,(unsigned int)gaussian_sphere_points_num));


		std::fstream fs_orientation;
		std::fstream fs_radius;
		std::fstream fs_position;
		fs_orientation.open (ss.str()+"_orientation.txt", std::fstream::in | std::fstream::out | std::fstream::app);
		fs_radius.open (ss.str()+"_radius.txt", std::fstream::in | std::fstream::out | std::fstream::app);
		fs_position.open (ss.str()+"_position.txt", std::fstream::in | std::fstream::out | std::fstream::app);


		for(int i=0;i<point_clouds.size();++i)
		{
			unsigned int ground_truth_index=i%iterations;

			if(ground_truth_index==0&&i>0)
			{
			 fs_orientation<< "\n";
			 fs_radius<< "\n";
			 fs_position<< "\n";
			
			}
			//ROS_INFO_STREAM("i:"<<i<<" index:"<<ground_truth_index);
			Eigen::VectorXf model_params=cylinder_segmentation->segment(point_clouds[i]);
			detections.push_back(model_params);

			// Compute errors
			//ROS_INFO_STREAM("model_params:" << model_params);
			//ROS_INFO_STREAM("ground_truth_params:" << ground_truths[ground_truth_index].cylinders.data[3] << " " << ground_truths[ground_truth_index].cylinders.data[4] << " " << ground_truths[ground_truth_index].cylinders.data[5]);

			float orientation_error=acos(model_params.segment(3,3).dot(Eigen::Vector3f(ground_truths[ground_truth_index].cylinders.data[3],ground_truths[ground_truth_index].cylinders.data[4],ground_truths[ground_truth_index].cylinders.data[5])));
			if(orientation_error>M_PI/2.0)
				orientation_error=M_PI-orientation_error;
			fs_orientation << orientation_error << " ";

			float radius_error=fabs(model_params[6]-ground_truths[ground_truth_index].cylinders.data[6]);
			fs_radius << radius_error << " ";


			float position_error=(model_params.head(2)-Eigen::Vector2f(ground_truths[ground_truth_index].cylinders.data[0],
										   ground_truths[ground_truth_index].cylinders.data[1])).norm();
			fs_position << position_error << " ";




			//ROS_INFO_STREAM("orientation_error:" << orientation_error*(180.0/M_PI));

			visualization_msgs::MarkerArray markers_;
			// First delete all markers
			visualization_msgs::Marker marker_;
			marker_.action = 3;
			markers_.markers.push_back(marker_);
			visualization_msgs::Marker marker=createMarker(model_params,visualization_msgs::Marker::CYLINDER,detections_frame_id,  0, marker_detections_namespace_);
			markers_.markers.push_back(marker);

			//if(i<0*200+200&&i>0*200)
			{
				detection_pub.publish( markers_ );
				cloud_pub.publish(point_clouds[i]);


			}

		}
		//fs << " more lorem ipsum";

		fs_orientation.close();
		fs_radius.close();
		fs_position.close();
		break;
	}

	return (0);
}
