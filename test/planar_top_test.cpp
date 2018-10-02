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
#include "config.h"
#include "helpers.h"
#include "planar_top_detector.h"

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
