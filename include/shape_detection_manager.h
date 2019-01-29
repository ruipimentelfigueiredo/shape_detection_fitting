/*
Copyright 2018 Rui Miguel Horta Pimentel de Figueiredo

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*!    
    \author Rui Figueiredo : ruipimentelfigueiredo
*/
#ifndef SHAPEDETECTIONMANAGER_H
#define SHAPEDETECTIONMANAGER_H

<<<<<<< HEAD
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
=======
>>>>>>> master
#include <pcl/point_types.h>
#include "cylinder_classifier.h"
#include "cylinder_fitting_hough.h"
#include "cylinder_fitting_ransac.h"
#include "sphere_fitting_hough.h"
<<<<<<< HEAD
#include "plane_fitting.h"
#include "detection_data.h"
#include <chrono>
#include <pcl/conversions.h>

#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"
#include "helpers.h"
#include "annotations.h"
#include <future>
#include <thread>
#include <chrono>
=======
#include "plane_fitting_ransac.h"



class DetectionData
{
	public:
	FittingData plane_fitting_data;
	std::vector<FittingData> clusters_fitting_data;
	std::vector<ClassificationData> clusters_classification_data;
	std::vector<cv::Mat> bounding_boxes;
	DetectionData(FittingData & plane_fitting_data_, std::vector<FittingData> & clusters_fitting_data_,std::vector<cv::Mat> & bounding_boxes_) : plane_fitting_data(plane_fitting_data_), clusters_fitting_data(clusters_fitting_data_), bounding_boxes(bounding_boxes_)
	{};

};
>>>>>>> master

template <class cylinder_detector_type, class sphere_detector_type>
class ShapeDetectionManager
{
	boost::shared_ptr<CylinderClassifier> cylinder_classifier;
	boost::shared_ptr<cylinder_detector_type> cylinder_fitting;
	boost::shared_ptr<sphere_detector_type> sphere_fitting;

 	const Eigen::Matrix4f cam_projection_matrix;
	double classification_threshold;
	float padding;
<<<<<<< HEAD

	bool with_classifier, dataset_create;
	std::string dataset_path, object_type;
	unsigned int scene_number;
	unsigned int directory_number;
	std::string pcl_path;
	std::string pcl_clusters_path;
	std::string images_path;
	std::string images_clusters_path;
	std::string annotations_path;

	static std::string getRecord()
	{    
		std::string answer;
		std::cin >> answer;
		return answer;
	}
	std::future<std::string> future;
	bool recording;
=======
>>>>>>> master
	public:
		ShapeDetectionManager(
			boost::shared_ptr<CylinderClassifier> & cylinder_classifier_, 
			boost::shared_ptr<cylinder_detector_type> & cylinder_fitting_, 
			boost::shared_ptr<sphere_detector_type> & sphere_fitting_, 
			const Eigen::Matrix4f & cam_projection_matrix_, 
			const double & classification_threshold_,
<<<<<<< HEAD
			double padding_=0.1,
			bool with_classifier_=true,
			bool dataset_create_=false,
			std::string dataset_path_="path",
			std::string object_type_="cylinder") :
=======
			double padding_=0.1) :
>>>>>>> master
				cylinder_classifier(cylinder_classifier_), 
				cylinder_fitting(cylinder_fitting_),
				sphere_fitting(sphere_fitting_),
	 			cam_projection_matrix(cam_projection_matrix_),
				classification_threshold(classification_threshold_),
<<<<<<< HEAD
				padding(padding_),
				with_classifier(with_classifier_),
				dataset_create(dataset_create_),
				dataset_path(dataset_path_),
				object_type(object_type_),
				scene_number(0),
				recording(false)
		{
			if(dataset_create)
			{
				future=std::async(std::launch::async, [](){
					std::this_thread::sleep_for(std::chrono::seconds(1));
					return getRecord();
				});
				
				std::cout << "Dataset create mode" << std::endl;
				directory_number=sub_directory_count(dataset_path_+std::string("/annotations/")+object_type+std::string("/"));
				pcl_path=dataset_path_+std::string("/pointclouds/")+object_type+std::string("/")+std::to_string(directory_number)+std::string("/");
				pcl_clusters_path=dataset_path_+std::string("/pointclouds_clusters/")+object_type+std::string("/")+std::to_string(directory_number)+std::string("/");
				images_path=dataset_path_+std::string("/images/")+object_type+std::string("/")+std::to_string(directory_number)+std::string("/");
				images_clusters_path=dataset_path_+std::string("/images_clusters/")+object_type+std::string("/")+std::to_string(directory_number)+std::string("/");
				annotations_path=dataset_path_+std::string("/annotations/")+object_type+std::string("/")+std::to_string(directory_number)+std::string("/");

				boost::filesystem::path pcl_dir(pcl_path);
				boost::filesystem::path pcl_clusters_dir(pcl_clusters_path);
				boost::filesystem::path images_dir(images_path);
				boost::filesystem::path images_clusters_dir(images_clusters_path);
				boost::filesystem::path annotations_dir(annotations_path);
				try
				{
						boost::filesystem::create_directories(pcl_dir);           
						boost::filesystem::create_directories(pcl_clusters_dir); 
						boost::filesystem::create_directories(images_dir);
						boost::filesystem::create_directories(images_clusters_dir);
						boost::filesystem::create_directories(annotations_dir);
				}
				catch (const std::exception& ex)				
				{
    					std::cerr << ex.what() << '\n';
						exit(-1);
				}
				std::cout << "Successfully created dataset directories" << "\n";
			}
		};

		DetectionData detect(	const cv::Mat & image_cv, 
								std::vector<PointCloudT::Ptr> & pcl_clusters, 
								std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> & clusters_, 
								const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & point_cloud,//const PointCloudT::Ptr & point_cloud,
								FittingData & plane_fitting_data, 
								std::vector<long int> & durations=std::vector<long int>())
		{
			std::chrono::high_resolution_clock::time_point t1;
			std::chrono::high_resolution_clock::time_point t2;

			std::vector<FittingData> shape_fitting_data; 
			shape_fitting_data.resize(pcl_clusters.size());

			std::vector<cv::Rect> bounding_boxes;
			std::vector<ClassificationData> clusters_class_data;

			t1 = std::chrono::high_resolution_clock::now();
			for(unsigned int i=0; i<pcl_clusters.size();++i)
			{
				/* BACKPROJECT CLUSTERS AND CLASSIFY */
=======
				padding(padding_)
		{};
	
		DetectionData detect(cv::Mat & image_cv, std::vector<PointCloudT::Ptr> & pcl_clusters,FittingData & plane_fitting_data)
		{
			std::vector<FittingData> shape_fitting_data; shape_fitting_data.resize(pcl_clusters.size());

			std::chrono::high_resolution_clock::time_point t1;
			std::chrono::high_resolution_clock::time_point t2;

			std::vector<cv::Mat> bounding_boxes;
>>>>>>> master

			for(unsigned int i=0; i<pcl_clusters.size();++i)
			{
				///////////////////////////
				// Get 2d bounding boxes //
				///////////////////////////
				t1 = std::chrono::high_resolution_clock::now();
				// Get XY max and min and construct image
				PointCloudT::Ptr cloud_projected(new PointCloudT);
		  		pcl::transformPointCloud (*pcl_clusters[i], *cloud_projected, cam_projection_matrix);

				for (unsigned int p=0; p<cloud_projected->points.size();++p)
				{	
					cloud_projected->points[p].x/=cloud_projected->points[p].z;
					cloud_projected->points[p].y/=cloud_projected->points[p].z;
					cloud_projected->points[p].z=1.0;
				}

				// Get minmax
				Eigen::Vector4f min_pt,max_pt;
				pcl::getMinMax3D(*cloud_projected,min_pt,max_pt);

				float width=max_pt[0]-min_pt[0];
				float height=max_pt[1]-min_pt[1];

				// Begin pad
				float width_padding=0.5*padding*image_cv.cols;
				float height_padding=0.5*padding*image_cv.rows;

				(min_pt[0]-width_padding)  < 0 ? min_pt[0]=0 : min_pt[0]-=width_padding;
				(min_pt[1]-height_padding) < 0 ? min_pt[1]=0 : min_pt[1]-=height_padding;

<<<<<<< HEAD
				(max_pt[0]+width_padding)  > (image_cv.cols-1) ? max_pt[0]=(image_cv.cols-1)  : max_pt[0]+=width_padding;
				(max_pt[1]+height_padding) > (image_cv.rows-1) ? max_pt[1]=(image_cv.rows-1)  : max_pt[1]+=height_padding;
		
				width=fabs(max_pt[0]-min_pt[0]);
				height=fabs(max_pt[1]-min_pt[1]);
				// End pad

				// Classify
				cv::Rect rect;
				cv::Mat roi;
				try
				{
					rect=cv::Rect((int)min_pt[0],(int)min_pt[1],(int)width,(int)height);
					if(rect.area()==0)//||rect.area())//>=0.9*width*image_height)
					{
						continue;
					}
					roi=image_cv(rect);
					bounding_boxes.push_back(rect);
				}
				catch (cv::Exception& e) 
				{
					std::cout << e.what() << std::endl;
					continue;
				}
				
				// BINARY CLASSIFIER FOR NOW
				float confidence=0.0;
				if(with_classifier)
				{
					confidence=cylinder_classifier->classify(roi);
					if(confidence>classification_threshold)
					{
						clusters_class_data.push_back(ClassificationData(FittingData::CYLINDER,confidence));
						shape_fitting_data[i].type=FittingData::CYLINDER;
					}
					else
					{
						clusters_class_data.push_back(ClassificationData(FittingData::OTHER,confidence));
						shape_fitting_data[i].type=FittingData::OTHER;
					}
				}
				else
				{
					clusters_class_data.push_back(ClassificationData(FittingData::CYLINDER,-1.0));
					shape_fitting_data[i].type=FittingData::CYLINDER;
				}
			}
			t2 = std::chrono::high_resolution_clock::now();
			durations.push_back(std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count());
			/* END BACKPROJECT CLUSTERS AND CLASSIFY */

			if(dataset_create && bounding_boxes.size()>0)
			{
				std::future_status status = future.wait_for(std::chrono::milliseconds(1));
				if (status == std::future_status::ready) {
					std::string answer=future.get();

					if(answer.compare("r")==0)
					{
						std::cout << "RECORDING!"<< std::endl;
						recording=true;
					}
					else if(answer.compare("s")==0)
					{
						std::cout << "PAUSED!\n" << std::endl;
						recording=false;
					}
					future=std::async(std::launch::async, [](){
						std::this_thread::sleep_for(std::chrono::milliseconds(1));
						return getRecord();
					});
				}
=======

				// Classify
				try {
					cv::Rect rect(min_pt[0], min_pt[1], width, height);
					cv::Mat roi ;//= image_cv(rect);
					bounding_boxes.push_back(roi);
				}
				catch (cv::Exception& e) {
					std::cout << e.what() << std::endl;
					continue;
				}


				//std::cout << rect << std::endl;
				//float confidence=cylinder_classifier->classify(bounding_boxes.back());
				float confidence=1.0;
				//std::cout << "classification confidence: " << confidence << " threshold: " << classification_threshold << std::endl;


				if(confidence>classification_threshold)
				{
					shape_fitting_data[i].type=FittingData::CYLINDER;
					// Visualization
					//cv::rectangle(image_cv, cv::Point(min_pt[0],min_pt[1]), cv::Point(max_pt[0],max_pt[1]), cv::Scalar(255,0,0), 4);
				}
				else
				{
					shape_fitting_data[i].type=FittingData::SPHERE;
					// Visualization
					//cv::rectangle(image_cv, cv::Point(min_pt[0],min_pt[1]), cv::Point(max_pt[0],max_pt[1]), cv::Scalar(0,0,255), 4);
				}
				t2 = std::chrono::high_resolution_clock::now();

				// dataset creation
				//cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
				//cv::imshow( "Display window", roi );                   // Show our image inside it.
				//cv::waitKey();
>>>>>>> master

				if(recording)
				{
					std::cout << "RECORDING - Directory number: " << directory_number << "  Scene number: " << std::to_string(scene_number) << std::endl;
					annotations::storeTrainingData(
						images_path, 
						pcl_path, 
						pcl_clusters_path, 
						images_clusters_path,
						annotations_path,
						scene_number, 
						image_cv, 
						point_cloud, 
						bounding_boxes, 
						pcl_clusters, 
						object_type
					);

					++scene_number;
					if(scene_number%100==0)
						recording=false;
				}
				//return DetectionData(); 
			}

<<<<<<< HEAD
			/* CLUSTERS FITTING */
			t1 = std::chrono::high_resolution_clock::now();
			Eigen::Affine3d plane_transform=plane_fitting_data.computeReferenceFrame();
			for(unsigned int ind=0; ind<shape_fitting_data.size(); ++ind)
			{
				// If object type is cylinder
				if(shape_fitting_data[ind].type==FittingData::CYLINDER)
				{	
					pcl::PointCloud<pcl::PointNormal>::Ptr cluster_projected(new pcl::PointCloud<pcl::PointNormal>());

					pcl::transformPointCloudWithNormals (*clusters_[ind], *cluster_projected, plane_transform.inverse());

					pcl::PointCloud<pcl::Normal>::Ptr normal_cluster_projected(new pcl::PointCloud<pcl::Normal>);
					pcl::copyPointCloud(*cluster_projected, *normal_cluster_projected);

					pcl::PointCloud<pcl::PointXYZ>::Ptr point_cluster_projected(new pcl::PointCloud<pcl::PointXYZ>);
					pcl::copyPointCloud(*cluster_projected, *point_cluster_projected);

					try
					{
						shape_fitting_data[ind]=cylinder_fitting->fit(point_cluster_projected,normal_cluster_projected);
						if(shape_fitting_data[ind].confidence<FittingData::fitting_threshold&&!with_classifier)
							shape_fitting_data[ind].type=FittingData::OTHER;
					}
					catch (const std::exception& ex)
					{
						std::cout << ex.what() << std::endl;
						continue;
					}
					
					Eigen::Vector3f position=shape_fitting_data[ind].parameters.segment(0,3);
					Eigen::Vector3f direction=shape_fitting_data[ind].parameters.segment(3,3);

					shape_fitting_data[ind].parameters.segment(0,3)=(plane_transform.linear().cast<float>()*position).cast<float>();
					shape_fitting_data[ind].parameters.segment(3,3)=(plane_transform.rotation().cast<float>()*direction).cast<float>();

					// Convert point cloud back to original frame (for visualization purposes)
					pcl::transformPointCloud (*shape_fitting_data[ind].inliers, *shape_fitting_data[ind].inliers, plane_transform);
				}
				else 
				{
					shape_fitting_data[ind].type=FittingData::OTHER;
					continue;
				}
			}
			t2 = std::chrono::high_resolution_clock::now();
			durations.push_back(std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count());
			/* END CLUSTERS FITTING */
			return DetectionData(plane_fitting_data,shape_fitting_data,bounding_boxes,clusters_class_data,with_classifier);
=======
			Eigen::Affine3d plane_transform =plane_fitting_data.computeReferenceFrame();
			std::vector<FittingData> clusters_fitting_data;
			for(unsigned int ind=0; ind<shape_fitting_data.size();++ind)
			{

				t1 = std::chrono::high_resolution_clock::now();
				if(shape_fitting_data[ind].type==FittingData::CYLINDER)
				{
					PointCloudT::Ptr cloud_projected(new PointCloudT);
			  		pcl::transformPointCloud (*pcl_clusters[ind], *cloud_projected, plane_transform.inverse());
					clusters_fitting_data.push_back(cylinder_fitting->fit(cloud_projected));

					Eigen::Vector3f position=clusters_fitting_data[ind].parameters.segment(0,3);
					Eigen::Vector3f direction=clusters_fitting_data[ind].parameters.segment(3,3);
					clusters_fitting_data[ind].parameters.segment(0,3)=(plane_transform.linear().cast<float>()*position).cast<float>();
					clusters_fitting_data[ind].parameters.segment(3,3)=(plane_transform.rotation().cast<float>()*direction).cast<float>();

				}
				else if(shape_fitting_data[ind].type==FittingData::SPHERE)
				{
					clusters_fitting_data.push_back(sphere_fitting->fit(pcl_clusters[ind]));
				}
				else 
					continue;
				t2 = std::chrono::high_resolution_clock::now();
			}

			return DetectionData(plane_fitting_data,clusters_fitting_data,bounding_boxes);
>>>>>>> master
		}
};

#endif // SHAPEDETECTIONMANAGER_H
