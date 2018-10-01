/*
Copyright 2018 Rui Miguel Horta Pimentel de Figueiredo

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*!   
    \author Rui Figueiredo : ruipimentelfigueiredo
*/

#include <ctime>
#include <chrono>
#include <boost/filesystem.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tinyxml.h>
#include "helpers.h"
#include "shape_detection_manager.h"
#include "planar_top_detector.h"




class Config 
{
	public:
	YAML::Node config;

	Config(){};

	// Common parameters
	double min_radius;
	double max_radius;

	// Classifier parameters
	double classification_threshold;
	std::string classifier_absolute_path_folder,model_file, weight_file, mean_file, device;
	int device_id;

	// Hough
	unsigned int angle_bins;
	unsigned int radius_bins;
	unsigned int position_bins;

	double distance_threshold;
	double cluster_tolerance;

	int min_cluster_size;
	int max_cluster_size;
	bool do_refine;

	double table_z_filter_min;
	double table_z_filter_max;
	double z_filter_min;
	double z_filter_max;
	double plane_detection_voxel_size;
	double cluster_voxel_size;
	int inlier_threshold;

	// projection padding
	double padding;

	int mode;
	double accumulator_peak_threshold;
	int gaussian_sphere_points_num;
	int orientation_accumulators_num;

	double fitting_distance_threshold;
	double fitting_acceptance_threshold;

	bool visualize;
	bool with_classifier;

	GaussianMixtureModel gmm;
	Eigen::Matrix3f cameraColorMatrix;
	Eigen::Matrix3f rotation;
	Eigen::Vector3f translation;
	Eigen::Matrix4f extrinsics;
	Eigen::Matrix4f cam_projection;
	void parseConfig(std::string file_name)
	{
		try
		{
			YAML::Node config = YAML::LoadFile(file_name);

			// Common parameters
			min_radius = config["min_radius"].as<double>();
			max_radius = config["max_radius"].as<double>();

			// Classifier parameters
			classifier_absolute_path_folder=config["classifier_absolute_path_folder"].as<std::string>();
			model_file=config["model_file"].as<std::string>();
			weight_file=config["weight_file"].as<std::string>();
			mean_file=config["mean_file"].as<std::string>();

			device=config["device"].as<std::string>();
			device_id=config["device_id"].as<int>();
			classification_threshold=config["classification_threshold"].as<double>();

			// hough
			angle_bins=config["angle_bins"].as<unsigned int>();
			radius_bins=config["radius_bins"].as<unsigned int>();
			position_bins=config["position_bins"].as<unsigned int>();

			distance_threshold=config["distance_threshold"].as<double>();
			cluster_tolerance=config["cluster_tolerance"].as<double>();

			min_cluster_size=config["min_cluster_size"].as<int>();
			max_cluster_size=config["max_cluster_size"].as<int>();
			do_refine=config["do_refine"].as<bool>();

			table_z_filter_min=config["table_z_filter_min"].as<double>();
			table_z_filter_max=config["table_z_filter_max"].as<double>();
			z_filter_min=config["z_filter_min"].as<double>();
			z_filter_max=config["z_filter_max"].as<double>();
			plane_detection_voxel_size=config["plane_detection_voxel_size"].as<double>();
			cluster_voxel_size=config["cluster_voxel_size"].as<double>();
			inlier_threshold=config["inlier_threshold"].as<int>();

			// projection padding
			padding=config["padding"].as<double>();

			mode=config["mode"].as<int>();
			accumulator_peak_threshold=config["accumulator_peak_threshold"].as<double>();
			gaussian_sphere_points_num=config["gaussian_sphere_points_num"].as<int>();
			orientation_accumulators_num=config["orientation_accumulators_num"].as<int>();

			std::vector<Eigen::Matrix<double, 3 ,1> > means;
    			std::vector<Eigen::Matrix<double, 3 ,1> > std_devs;
			std::vector<double> weights;
			YAML::Node orientation_hough_gmms = config["orientation_hough_gmm"];
			for (std::size_t i = 0; i < orientation_hough_gmms.size(); ++i) 
			{
				const YAML::Node& node_gmm = orientation_hough_gmms[i];

    				weights.push_back(node_gmm["weight"].as<double>());

				Eigen::Matrix<double, 3 ,1> mean_eigen;
				const YAML::Node& node_means = node_gmm["mean"];
				for (std::size_t j=0;j<node_means.size();++j)
				{
					mean_eigen(j)=node_means[j].as<double>();
				}
				means.push_back(mean_eigen);

				Eigen::Matrix<double, 3 ,1> std_dev_eigen;
				const YAML::Node& node_std_devs = node_gmm["standard_deviation"];
				for (std::size_t j=0;j<node_std_devs.size();++j)
				{
					std_dev_eigen(j)=node_std_devs[j].as<double>();

				}

				std_devs.push_back(std_dev_eigen);
			}
			gmm=GaussianMixtureModel(weights, means, std_devs);

			fitting_distance_threshold=config["fitting_distance_threshold"].as<double>();
			fitting_acceptance_threshold=config["fitting_acceptance_threshold"].as<double>();

			visualize=config["visualize"].as<bool>();
			with_classifier=config["with_classifier"].as<bool>();

			// camera rgb parameters
			YAML::Node cameraColorMatrix_ = config["camera_matrix"];

			for (std::size_t j=0;j<cameraColorMatrix_.size();++j)
			{
				cameraColorMatrix(j)=cameraColorMatrix_[j].as<float>();
			}
			cameraColorMatrix.transposeInPlace();

			// extrinsic parameters
			YAML::Node rotation_ = config["rotation"];
			for (std::size_t j=0;j<rotation_.size();++j)
			{
				rotation(j)=rotation_[j].as<float>();
			}
			rotation.transposeInPlace();


			YAML::Node translation_ = config["translation"];
			for (std::size_t j=0;j<translation_.size();++j)
			{
				translation(j)=translation_[j].as<float>();
			}

			Eigen::Matrix4f extrinsics = Eigen::Matrix<float,4,4>::Identity();
			extrinsics.block(0,0,3,3)=rotation;
			extrinsics.block(0,3,3,1)=translation;
			Eigen::Matrix<float,3,4> projection_matrix_3d_2d = Eigen::Matrix<float,3,4>::Zero();
			projection_matrix_3d_2d(0,0)=projection_matrix_3d_2d(1,1)=projection_matrix_3d_2d(2,2)=1.0;
    			cam_projection = Eigen::Matrix4f::Identity();
			cam_projection.block(0,0,3,4)=cameraColorMatrix*projection_matrix_3d_2d*extrinsics;

    			//undistort(InputArray src, OutputArray dst, InputArray cameraMatrix, InputArray distCoeffs, InputArray newCameraMatrix=noArray() )
		}
		catch (YAML::Exception& yamlException)
		{
			std::cout << "Unable to parse" << file_name <<std::endl;
		}

		std::cout << cam_projection << std::endl;
	}

    	friend ostream& operator<<(std::ostream& os, const Config& ft)  
	{  

		os << "min radius: " << ft.min_radius << " " << std::endl;  
		os << "max radius: " << ft.max_radius << " " << std::endl;  
		os << "classification_threshold: " << ft.classification_threshold << " " << std::endl;  
		os << "classifier_absolute_path_folder: " << ft.classifier_absolute_path_folder << " " << std::endl;  
		os << "model_file: " << ft.model_file << " " << std::endl;  
		os << "weight_file: " << ft.weight_file << " " << std::endl;   
		os << "mean_file: " << ft.mean_file << " " << std::endl;   
		os << "device: " << ft.device << " " << std::endl;   
		os << "device_id: " << ft.device_id << " " << std::endl;   

		os << "angle_bins: " << ft.angle_bins << " " << std::endl;   
		os << "radius_bins: " << ft.radius_bins << " " << std::endl;    
		os << "position_bins: " << ft.position_bins << " " << std::endl;  

		os << "distance_threshold: " << ft.distance_threshold << " " << std::endl;    
		os << "cluster_tolerance: " << ft.cluster_tolerance << " " << std::endl;    

		os << "min_cluster_size: " << ft.min_cluster_size << " " << std::endl;   
		os << "max_cluster_size: " << ft.max_cluster_size << " " << std::endl;    

		os << "do_refine: " << ft.do_refine << " " << std::endl;  
		os << "table_z_filter_min: " << ft.table_z_filter_min << " " << std::endl;   
		os << "table_z_filter_max: " << ft.table_z_filter_max << " " << std::endl;    
		os << "z_filter_min: " << ft.z_filter_min << " " << std::endl;  
		os << "z_filter_max: " << ft.z_filter_max << " " << std::endl;    

		os << "plane_detection_voxel_size: " << ft.plane_detection_voxel_size << " " << std::endl;    
		os << "cluster_voxel_size: " << ft.cluster_voxel_size << " " << std::endl;  
		os << "inlier_threshold: " << ft.inlier_threshold << " " << std::endl;   

		os << "padding: " << ft.padding << " " << std::endl;  

		os << "mode: " << ft.mode << " " << std::endl;   

		os << "accumulator_peak_threshold: " << ft.accumulator_peak_threshold << " " << std::endl;    
		os << "gaussian_sphere_points_num: " << ft.gaussian_sphere_points_num << " " << std::endl;    
		os << "orientation_accumulators_num: " << ft.orientation_accumulators_num << " " << std::endl;  

		os << "fitting_distance_threshold: " << ft.fitting_distance_threshold << " " << std::endl;    
		os << "fitting_acceptance_threshold: " << ft.fitting_acceptance_threshold << " " << std::endl;    
		os << "visualize: " << ft.visualize << " " << std::endl;    
		os << "with_classifier: " << ft.with_classifier << " " << std::endl;    
		return os;  
	} 
};


class GroundTruthDetection 
{
	public:
	cv::Rect bbox;
	cv::Scalar color;
	GroundTruthDetection(cv::Rect & bbox_, cv::Scalar & color_) : bbox(bbox_), color(color_)
	{};
};

std::vector<GroundTruthDetection> parseGroundTruth(std::string file, boost::shared_ptr<TiXmlDocument> doc)
{
	std::vector<GroundTruthDetection> ground_truth;

	// Load ground truth file
	doc->LoadFile(file);
	TiXmlElement *pRoot = doc->FirstChildElement("annotation");
	TiXmlElement *objParam;
	if(pRoot)
	{
		objParam = pRoot->FirstChildElement("object");

		while(objParam)
		{
			TiXmlElement* bndbox = objParam->FirstChildElement("bndbox");
			cv::Rect rect;
			if(bndbox)
			{
				int xmin(0),xmax(0),ymin(0),ymax(0);

				TiXmlElement *xminParam = bndbox->FirstChildElement("xmin");
				if(xminParam)
					xmin=atoi(xminParam->GetText());
				TiXmlElement *yminParam = bndbox->FirstChildElement("ymin");
				if(yminParam)
					ymin=atoi(yminParam->GetText());
				TiXmlElement *xmaxParam = bndbox->FirstChildElement("xmax");
				if(xmaxParam)
					xmax=atoi(xmaxParam->GetText());
				TiXmlElement *ymaxParam = bndbox->FirstChildElement("ymax");
				if(ymaxParam)
					ymax=atoi(ymaxParam->GetText());

				rect=cv::Rect(xmin,ymin,(xmax-xmin),(ymax-ymin));
			}
			else
				continue;

			TiXmlElement* typeParam = objParam->FirstChildElement("name");
			std::string type=std::string(typeParam->GetText());

			cv::Scalar color;
			if(type=="cylinder")
				color=cv::Scalar(0,0,255);
			else if(type=="sphere")
				color=cv::Scalar(0,255,0);
			else if(type=="box")
				color=cv::Scalar(255,0,0);

			objParam = objParam->NextSiblingElement("object");

			ground_truth.push_back(GroundTruthDetection(rect,color));
		}
	}
	return ground_truth;
};

int main (int argc, char** argv)
{
    // Arguments
    std::string dataset_path=std::string(argv[1]);
    std::cout << "dataset_path: " << dataset_path << std::endl;

    std::string config_file=std::string(argv[2]);

    Config config;
    config.parseConfig(config_file);
    std::cout << config << std::endl;

    std::vector<std::string> image_files,annotation_files,point_cloud_files;

    std::string images_dataset_path=dataset_path+"/images/";
    std::string point_clouds_dataset_path=dataset_path+"/point_clouds/";
    std::string annotations_dataset_path=dataset_path+"/annotations/";
    std::string results_path=dataset_path+"/results/";

    boost::shared_ptr<VisualizeFittingData> visualizer(new VisualizeFittingData());
    boost::shared_ptr<TiXmlDocument> doc(new TiXmlDocument());

    readDirectory(images_dataset_path,image_files);
    readDirectory(point_clouds_dataset_path,point_cloud_files);
    readDirectory(annotations_dataset_path,annotation_files);
    createDirectory(results_path);

    boost::shared_ptr<CylinderClassifier> cylinder_classifier;
    if(config.with_classifier)
      cylinder_classifier=boost::shared_ptr<CylinderClassifier> (new CylinderClassifier(config.classifier_absolute_path_folder,config.model_file,config.weight_file,config.mean_file,config.device,(unsigned int)config.device_id));
 
    GaussianSphere gaussian_sphere(config.gmm,config.gaussian_sphere_points_num,config.orientation_accumulators_num);

    boost::shared_ptr<SphereFittingHough> sphere_fitting(new SphereFittingHough(gaussian_sphere,config.position_bins,config.radius_bins,config.min_radius, config.max_radius,config.accumulator_peak_threshold));

    boost::shared_ptr<CylinderFittingHough> cylinder_fitting(new CylinderFittingHough(gaussian_sphere,config.angle_bins,config.radius_bins,config.position_bins,config.min_radius, config.max_radius,config.accumulator_peak_threshold,config.mode));

    boost::shared_ptr<PlaneFittingRansac> plane_fitting(new PlaneFittingRansac(config.distance_threshold,config.cluster_tolerance,config.min_cluster_size,config.max_cluster_size,config.do_refine,config.table_z_filter_min,config.table_z_filter_max,config.z_filter_min, config.z_filter_max,config.plane_detection_voxel_size, config.cluster_voxel_size,config.inlier_threshold));

    // Get point clouds
    PointClouds point_clouds(point_clouds_dataset_path);

    boost::shared_ptr<PlanarTopDetector<CylinderFittingHough, SphereFittingHough, PlaneFittingRansac> > planar_top_detector(new PlanarTopDetector<CylinderFittingHough, SphereFittingHough, PlaneFittingRansac>(plane_fitting, cylinder_classifier, cylinder_fitting, sphere_fitting, config.cam_projection, config.classification_threshold, config.padding, config.cluster_voxel_size, config.z_filter_min, config.z_filter_max, config.with_classifier, config.visualize));

    // Downsampler for faster visualization
    pcl::VoxelGrid<pcl::PointXYZRGB> grid_;
    grid_.setLeafSize (0.01, 0.01, 0.01);
    grid_.setFilterFieldName ("z");
    grid_.setFilterLimits (-10.0, 10.0);
    grid_.setDownsampleAllData (true);

    std::fstream plane_fitting_time;
    std::fstream cluster_extraction_time;
    std::fstream cluster_classification_fitting;
    plane_fitting_time.open (results_path+"plane_fitting_time.txt", std::fstream::in | std::fstream::out | std::fstream::trunc);
    cluster_extraction_time.open (results_path+"cluster_extraction_time.txt", std::fstream::in | std::fstream::out | std::fstream::trunc);
    cluster_classification_fitting.open (results_path+"cluster_classification_fitting.txt", std::fstream::in | std::fstream::out | std::fstream::trunc);
    
    for(unsigned int i=0;i<point_clouds.file_names.size();++i)
    {
	try
	{
		// Load RGB data in OpenCV format
		cv::Mat image = cv::imread(images_dataset_path+image_files[i], CV_LOAD_IMAGE_COLOR);    // Read the file
		if(! image.data )                              						// Check for invalid input
		{
		    std::cout <<  "Could not open or find the image " << image_files[i] << std::endl ;
		    return -1;
		}

		// Load 3D point cloud information in PCL format
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_color(new pcl::PointCloud<pcl::PointXYZRGB>());
		pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>());

		point_cloud_color=point_clouds.loadPointCloudRGB(point_clouds_dataset_path+point_clouds.file_names[i]);
		point_cloud->points.resize(point_cloud_color->points.size());
		for (size_t j = 0; j < point_cloud_color->points.size(); ++j) {
			point_cloud->points[j].x = point_cloud_color->points[j].x;
			point_cloud->points[j].y = point_cloud_color->points[j].y;
			point_cloud->points[j].z = point_cloud_color->points[j].z;
		}

		// Load ground truth file
		std::vector<GroundTruthDetection> bboxes=parseGroundTruth(annotations_dataset_path+annotation_files[i],doc);

		std::vector<long int> durations;
		/*std::cout << point_cloud->isOrganized() << std::endl;

	  	// Executing the transformation
	  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ> ());

	  	pcl::transformPointCloud(*point_cloud, *cloud_projected, config.cam_projection);
		cv::Mat depth_image(1920,1080,CV_32F,cv::Scalar(0));
		for (unsigned int p=0; p<cloud_projected->points.size(); ++p)
		{	
			cloud_projected->points[p].x/=cloud_projected->points[p].z;
			cloud_projected->points[p].y/=cloud_projected->points[p].z;
			unsigned int u=floor(cloud_projected->points[p].x);
			unsigned int v=floor(cloud_projected->points[p].y);

			if(u>=1920 || u<0 || v>=1080 || v<0)
			{
				std::cout << u << " " << cloud_projected->points[p].y << std::endl;
				continue;
			}

			depth_image.at<float>(v, u) = cloud_projected->points[p].z;
			cloud_projected->points[p].z = 1.0;
		}
		cv::Mat depth_image_view(1920,1080,CV_8U,cv::Scalar(0));
		cv::normalize(depth_image, depth_image_view, 255,0);
		std::cout << depth_image_view << std::endl;
		cv::namedWindow("Display window", cv::WINDOW_NORMAL); // Create a window for display.
		cv::imshow("Display window", depth_image);              // Show our image inside it.
		cv::resizeWindow("Display window", 0.5*image.cols,0.5*image.rows);
		cv::waitKey(1000);
		//*/

		/* VISUALIZE GROUND TRUTH */
		if(config.visualize)
		{
			visualizer->clear();
			/*for(unsigned int i = 0; i<bboxes.size(); ++i)
			{
				cv::rectangle(image, bboxes[i].bbox, bboxes[i].color, 2, 4, 0 );	
			}*/
		}

		DetectionData detections=planar_top_detector->detect(image,point_cloud,visualizer,durations);

		/* VISUALIZE DETECTIONS */
		if(config.visualize) 
		{
			grid_.setInputCloud (point_cloud_color);
			grid_.filter (*point_cloud_color);
			visualizer->addPointCloudRGB(point_cloud_color,point_clouds.file_names[i]);
			visualizer->viewer->setCameraPosition(0,0,-2, 0,-1,0);
 			visualizer->spin(1);
			detections.visualize(image);
		}
		/* END VISUALIZE */

		//cv::imwrite( "/home/rui/Desktop/results/"+image_files[i], image );
		plane_fitting_time << durations[0] << " " << "\n";
		cluster_extraction_time << durations[1] << " " << "\n";
		cluster_classification_fitting << durations[2] << " " << "\n";

		std::cout << "iteration " << (i+1) << " of " << image_files.size() << " plane fitting time: " << durations[0] << " ms " << " cluster extraction time: " << durations[1] << " ms" << " classification: "<< durations[2] << " ms  fitting: " << durations[3] << " ms" << std::endl;
	}
	catch (exception& e) {
		std::cout << e.what() << std::endl;
		continue;
	}
    }
    std::cout << "end" << std::endl;
    plane_fitting_time.close();
    cluster_extraction_time.close();
    cluster_classification_fitting.close();

    return 0;
}
