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

#include <pcl/point_types.h>
#include "cylinder_classifier.h"
#include "cylinder_fitting_hough.h"
#include "cylinder_fitting_ransac.h"
#include "sphere_fitting_hough.h"
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

template <class cylinder_detector_type, class sphere_detector_type>
class ShapeDetectionManager
{
	boost::shared_ptr<CylinderClassifier> cylinder_classifier;
	boost::shared_ptr<cylinder_detector_type> cylinder_fitting;
	boost::shared_ptr<sphere_detector_type> sphere_fitting;

 	const Eigen::Matrix4f cam_projection_matrix;
	double classification_threshold;
	float padding;
	public:
		ShapeDetectionManager(
			boost::shared_ptr<CylinderClassifier> & cylinder_classifier_, 
			boost::shared_ptr<cylinder_detector_type> & cylinder_fitting_, 
			boost::shared_ptr<sphere_detector_type> & sphere_fitting_, 
			const Eigen::Matrix4f & cam_projection_matrix_, 
			const double & classification_threshold_,
			double padding_=0.1) :
				cylinder_classifier(cylinder_classifier_), 
				cylinder_fitting(cylinder_fitting_),
				sphere_fitting(sphere_fitting_),
	 			cam_projection_matrix(cam_projection_matrix_),
				classification_threshold(classification_threshold_),
				padding(padding_)
		{};
	
		DetectionData detect(cv::Mat & image_cv, std::vector<PointCloudT::Ptr> & pcl_clusters,FittingData & plane_fitting_data)
		{
			std::vector<FittingData> shape_fitting_data; shape_fitting_data.resize(pcl_clusters.size());

			std::chrono::high_resolution_clock::time_point t1;
			std::chrono::high_resolution_clock::time_point t2;

			std::vector<cv::Mat> bounding_boxes;

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
	
				// PAD
				float width_padding=0.5*padding*width;
				float height_padding=0.5*padding*height;

				(min_pt[0]-width_padding)  <0 ? min_pt[0]=0 : min_pt[0]-=width_padding;
				(min_pt[1]-height_padding) <0 ? min_pt[1]=0 : min_pt[1]-=height_padding;

				(max_pt[0]+width_padding)  >(image_cv.cols-1) ? max_pt[0]=(image_cv.cols-1)  : max_pt[0]+=width_padding;
				(max_pt[1]+height_padding) >(image_cv.rows-1) ? max_pt[1]=(image_cv.rows-1) : max_pt[1]+=height_padding;
		
				width=max_pt[0]-min_pt[0];
				height=max_pt[1]-min_pt[1];
				// end pad


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

				//if((image_number%5)==0)
				//imwrite("/home/rui/sphere/sphere.scene."+std::to_string(scene_number)+".cluster."+std::to_string(i)+".jpg", image_cv(rect) );

			}

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
		}
};

#endif // SHAPEDETECTIONMANAGER_H
