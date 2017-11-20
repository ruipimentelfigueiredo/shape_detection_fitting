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
#include "pcl_ros/point_cloud.h"
#include <visualization_msgs/MarkerArray.h>
#include <helpers.h>

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

	std::vector<double> weights;
	std::vector<Eigen::Matrix<double, 3 ,1> > means;
	std::vector<Eigen::Matrix<double, 3 ,1> > std_devs;
	weights.push_back(1.0);
	Eigen::Matrix<double, 3 ,1> mean_eigen(0,0,0);
	means.push_back(mean_eigen);
	Eigen::Matrix<double, 3 ,1> std_dev_eigen(0.5,0.5,0.5);
	std_devs.push_back(std_dev_eigen);

	unsigned int angle_bins=30;
	unsigned int radius_bins=10;
 	unsigned int position_bins=100;
	unsigned int orientation_accumulators_num=1;
	int gaussian_sphere_points_num=450;
        float accumulator_peak_threshold=0.8;
	// TESTING PARAMS
 	float min_radius=0.45;
 	float max_radius=0.55;



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

	while(ros::ok())
	{
		// First, generate 200 point clouds with different radius, heights at random poses
		unsigned int iterations=6*200;
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
		ROS_ERROR_STREAM("POINT CLOUD SIZE:"<<ground_truths.size());

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
		ROS_ERROR_STREAM("POINT CLOUD SIZE:"<<point_clouds.size());
		//exit(1);
		std::string detections_frame_id="world";
		std::string marker_detections_namespace_="detections";
		// Segment cylinder and store results
		std::vector<Eigen::VectorXf > detections;
		std::vector<float> position_errors;

		std::vector<boost::shared_ptr<CylinderSegmentationHough> > cylinder_segmentators;

		GaussianMixtureModel gmm(weights, means, std_devs);
		GaussianSphere gaussian_sphere(gmm,gaussian_sphere_points_num,orientation_accumulators_num);

		cylinder_segmentators.push_back(boost::shared_ptr<CylinderSegmentationHough> (new CylinderSegmentationHough(gaussian_sphere,(unsigned int)angle_bins,(unsigned int)radius_bins,(unsigned int)position_bins,(float)min_radius, (float)max_radius,(float)accumulator_peak_threshold,CylinderSegmentationHough::NORMAL)));

		cylinder_segmentators.push_back(boost::shared_ptr<CylinderSegmentationHough> (new CylinderSegmentationHough(gaussian_sphere,(unsigned int)angle_bins,(unsigned int)radius_bins,(unsigned int)position_bins,(float)min_radius, (float)max_radius,(float)accumulator_peak_threshold,CylinderSegmentationHough::HYBRID)));


		for (unsigned int d=0;d < cylinder_segmentators.size();++d)
		{

			std::fstream fs_orientation;
			std::fstream fs_radius;
			std::fstream fs_position;
			fs_orientation.open (ss.str()+"_orientation_noise_biased2_"+ std::to_string(d)+".txt", std::fstream::in | std::fstream::out | std::fstream::app );
			fs_radius.open (ss.str()+"_radius_noise_biased2_"          + std::to_string(d)+".txt", std::fstream::in | std::fstream::out | std::fstream::app);
			fs_position.open (ss.str()+"_position_noise_biased2_"      + std::to_string(d)+".txt", std::fstream::in | std::fstream::out | std::fstream::app);

			ROS_ERROR_STREAM("OI:"<<ground_truths.size());
			for(unsigned int i=0;i<point_clouds.size();++i)
			{	//continue;
				unsigned int ground_truth_index=i%iterations;

				if(ground_truth_index==0&&i>0)
				{
					 fs_orientation<< "\n";
					 fs_radius<< "\n";
					 fs_position<< "\n";
				}

				active_semantic_mapping::Cylinders ground_truth=ground_truths[ground_truth_index];

				pcl::PointCloud<PointT>::Ptr point_cloud(new pcl::PointCloud<PointT>());
				*point_cloud=*point_clouds[i];
				//ROS_INFO_STREAM("i:"<<i<<" index:"<<ground_truth_index);
				CylinderFitting model_params=cylinder_segmentators[d]->segment(point_clouds[i]);
				//detections.push_back(model_params);

				// Compute errors
				//ROS_INFO_STREAM("model_params:" << model_params);
				//ROS_INFO_STREAM("ground_truth_params:" << ground_truth.cylinders.data[3] << " " << ground_truth.cylinders.data[4] << " " << ground_truth.cylinders.data[5]);

				float orientation_error=acos(model_params.parameters.segment(3,3).dot(Eigen::Vector3f(ground_truth.cylinders.data[3],ground_truth.cylinders.data[4],ground_truth.cylinders.data[5])));
				if(orientation_error>M_PI/2.0)
					orientation_error=M_PI-orientation_error;
				fs_orientation << orientation_error << " ";

				float radius_error=fabs(model_params.parameters[6]-ground_truth.cylinders.data[6]);
				fs_radius << radius_error << " ";


				float position_error=(model_params.parameters.head(3)-Eigen::Vector3f(ground_truth.cylinders.data[0],
											   ground_truth.cylinders.data[1],
											   ground_truth.cylinders.data[2])).norm();
				fs_position << position_error << " ";




				//ROS_INFO_STREAM("orientation_error:" << orientation_error*(180.0/M_PI));

				visualization_msgs::MarkerArray markers_;
				// First delete all markers
				visualization_msgs::Marker marker_;
				marker_.action = 3;
				markers_.markers.push_back(marker_);
				visualization_msgs::Marker marker=createMarker(model_params.parameters,visualization_msgs::Marker::CYLINDER,detections_frame_id,  0, marker_detections_namespace_);
				markers_.markers.push_back(marker);

				//if(i<0*200+200&&i>0*200)
				{
					detection_pub.publish( markers_ );
					cloud_pub.publish(point_cloud);


				}

			}

			fs_orientation.close();
			fs_radius.close();
			fs_position.close();
		}

		break;
	}

	return (0);
}
