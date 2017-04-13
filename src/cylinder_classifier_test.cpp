#include "cylinder_segmentation_hough.h"
#include <ctime>
#include <tf/transform_broadcaster.h>

#include <rosbag/bag.h>
#include <active_semantic_mapping/Cylinders.h>

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



	unsigned int angle_bins=30;
	unsigned int radius_bins=10;
 	unsigned int position_bins=10;

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
	ss << ".bag";

	std::string rosbag_file;
	rosbag_file = ss.str();


	rosbag::Bag bag;

	bag.open(rosbag_file, rosbag::bagmode::Write);

	// TESTING PARAMS
 	float min_radius=0.10;
 	float max_radius=0.20;

	int height_samples=30;
	int angle_samples=30;

	float radius=0.5;
	std::vector<float> heights;

	heights.push_back(1.00);
	heights.push_back(2.00);
	heights.push_back(3.00);
	heights.push_back(4.00);
	heights.push_back(5.00);
	heights.push_back(6.00);
	std::vector<float> noise_levels; // percentage of object size (std_dev)

	noise_levels.push_back(0.001);
	for(unsigned int i=1; i<=10; ++i)
	{
		float noise_level=(float)0.05*i;
		noise_levels.push_back(noise_level);
	}
	
	std::vector<pcl::PointCloud<pcl::PointXYZ> > point_clouds;
	std::vector<active_semantic_mapping::Cylinders> ground_truths;
	std::vector<Eigen::Matrix4f> transfs;

	// First, generate 200 point clouds with different radius, heights at random poses
	unsigned int iterations=200;

	for(unsigned int i=0; i<iterations; ++i)
	{
		for(unsigned int h_=0; h_<heights.size();++h_)
		{
			float height=heights[h_];
			// Generate cylinder according to parameters
			pcl::PointCloud<PointT> cloud;
			cloud.header.frame_id="world";

			float angle_step=2.0*M_PI/angle_samples;
			float height_step=fabs(height/height_samples);

			//std::cout << "height:" << height.at<float>(0,0) << " radius:" << radius.at<float>(0,0) << std::endl;

	
			for(int a=0; a < angle_samples; ++a)
			{

				float x,y,z;
				x=cos(angle_step*a)*fabs(radius);
				y=sin(angle_step*a)*fabs(radius);
				for(int h=0; h < height_samples; ++h)
				{
					z=(float)height_step*h;
					pcl::PointXYZ point(x,y,z);
					cloud.push_back(point);
				}
			}

			// TAMPAS
			/*unsigned int plane_samples=30;
			float plane_size=0.3;
			float plane_min=0.0-0.5*plane_size;
			float plane_step=(float)plane_size/plane_samples;
			float z=0.0;
			for(int x_=0; x_<plane_samples;++x_)
			{
				float x=plane_min+x_*plane_step;
				for(int y_=0; y_<plane_samples;++y_)
				{
				
					float y=plane_min+y_*plane_step;
					if(sqrt(x*x+y*y)>0.15) continue;

					pcl::PointXYZ point(x,y,z);
					//cloud.push_back(point);

					cloud.push_back(pcl::PointXYZ(x,y,height));
				}
			}*/
			point_clouds.push_back(cloud);
			// Generate random orientation
			Eigen::Vector3f rot_dir;
			cv::Mat aux(1, 1, CV_32F);


			cv::randn(aux, 0.0, 1.0);
			rot_dir(0,0)=aux.at<float>(0,0);

			cv::randn(aux, 0.0, 1.0);
			rot_dir(1,0)=aux.at<float>(0,0);

			cv::randn(aux, 0.0, 1.0);
			rot_dir(2,0)=aux.at<float>(0,0);

			rot_dir.normalize();

			cv::Mat angle(1, 1, CV_32F);
			cv::randn(angle, 0.0, 1.0);
			//Eigen::Vector3f rot_dir(1.0,0.0,0.0);
			rot_dir.normalize();
			rot_dir=Eigen::Vector3f::UnitZ();
			Eigen::Matrix3f rot;
			rot=Eigen::AngleAxisf(M_PI*angle.at<float>(0,0),rot_dir);

			//Eigen::Vector3f cyl_dir=rot*Eigen::Vector3f::UnitZ();
			Eigen::Vector3f cyl_dir=Eigen::Vector3f::UnitZ();
			Eigen::Matrix4f transf;
			transf.block(0,0,3,3)=rot;
			transf.block(0,3,3,1)=Eigen::Vector3f(0,0,0);
			transfs.push_back(transf);



			active_semantic_mapping::Cylinders ground_truth;


			ground_truth.cylinders.layout.dim.resize(2);
			ground_truth.cylinders.layout.dim[0].label  = "cylinders";
			ground_truth.cylinders.layout.dim[0].size   = 1;
			ground_truth.cylinders.layout.dim[0].stride = 8;
			ground_truth.cylinders.layout.dim[1].label  = "parameters";
			ground_truth.cylinders.layout.dim[1].size   = 8;
			ground_truth.cylinders.layout.dim[1].stride = 8;

			ground_truth.cylinders.data.push_back(0.5*cyl_dir[0]*height);
			ground_truth.cylinders.data.push_back(0.5*cyl_dir[1]*height);
			ground_truth.cylinders.data.push_back(0.5*cyl_dir[2]*height);
		
			ground_truth.cylinders.data.push_back(cyl_dir[0]);
			ground_truth.cylinders.data.push_back(cyl_dir[1]);
			ground_truth.cylinders.data.push_back(cyl_dir[2]);

			ground_truth.cylinders.data.push_back(radius);
			ground_truth.cylinders.data.push_back(height);

			bag.write("ground_truth",ros::Time::now(), ground_truth);
		}
	}

	// Then corrupt with diferent levels of noise
	for(unsigned int n=0; n<noise_levels.size(); ++n)
	{
		for(unsigned int i=0; i<iterations; ++i)
		{
			for(unsigned int h_=0; h_<heights.size();++h_)
			{
				int index=h_+i*heights.size();
				pcl::PointCloud<PointT> cloud=point_clouds[index];
		

				pcl::PointCloud<PointT> noisy_cloud;
				noisy_cloud=cloud;
				for(unsigned int p=0; p<cloud.size();++p)
				{
					cv::Mat aux(1, 1, CV_32F);
					cv::randn(aux, 0.0, noise_levels[n]*radius);
					noisy_cloud.points[p].x+=aux.at<float>(0,0);

					cv::randn(aux, 0.0, noise_levels[n]*radius);
					noisy_cloud.points[p].y+=aux.at<float>(0,0);

					cv::randn(aux, 0.0, noise_levels[n]*radius);
					noisy_cloud.points[p].z+=aux.at<float>(0,0);
				}


				//Transform point cloud
				pcl::PointCloud<PointT> cloud_transf;
				// transfs[index]
				Eigen::Matrix4f transf=Eigen::Matrix4f::Identity();
				pcl::transformPointCloud (noisy_cloud, cloud_transf,transf);


				ROS_ERROR_STREAM(" iteration:" << i << " noise_level: "<< n << " "  << noise_levels[n]); 
	    			bag.write("point_cloud",ros::Time::now(), cloud_transf);
			}
		}
	}

	bag.close();

	// Segment cylinder





	return (0);
}
