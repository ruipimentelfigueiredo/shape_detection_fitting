#ifndef CONFIG_FITTING_H
#define CONFIG_FITTING_H

#include "helpers.h"
#include "gaussian_mixture_model.h"
#include <tinyxml.h>
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
	}

    	friend std::ostream& operator<<(std::ostream& os, const Config& ft)  
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



#endif // config.h
