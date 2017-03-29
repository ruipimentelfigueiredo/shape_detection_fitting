#include "cylinder_classifier.h"
//#include "cylinder_segmentation_ros.h"
#include <ros/ros.h>
int main (int argc, char** argv)
{
	ros::init(argc, argv, "talker");

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle n_priv("~");
	std::string absolute_path_folder;
	std::string model_file;
	std::string weight_file;
	std::string mean_file;
	std::string device;
	int device_id;
	float classification_threshold;

	ROS_INFO("Getting classifier parameters");
	n_priv.param<std::string>("absolute_path_folder", absolute_path_folder, "absolute_path_folder");
	n_priv.param<std::string>("model_file", model_file, "model_file");
	n_priv.param<std::string>("weight_file", weight_file, "weight_file");
	n_priv.param<std::string>("mean_file", mean_file, "mean_file");
	n_priv.param<std::string>("device", device, "device");
	n_priv.param<int>("device_id", device_id, 0);

	n_priv.param<float>("classification_threshold", classification_threshold, 0.9);

	ROS_INFO_STREAM("absolute_path_folder:"<< absolute_path_folder);
	ROS_INFO_STREAM("model_file:"<< model_file);
	ROS_INFO_STREAM("weight_file:"<< weight_file);
	ROS_INFO_STREAM("mean_file:"<< mean_file);
	ROS_INFO_STREAM("device:"<< device);
	//ROS_INFO_STREAM("device_id:"<< device_id);
	//ROS_INFO_STREAM("classification_threshold:"<< classification_threshold);
	boost::shared_ptr<CylinderClassifier> cylinder_classifier(new CylinderClassifier(absolute_path_folder,model_file,weight_file,mean_file,device,(unsigned int)device_id));



	cv::String path("/home/rui/cylinder_dataset/train/other.1.jpg"); //select only other
	cv::String path1("/home/rui/cylinder/cylinder1/cylinder*.jpg"); //select only cylinder
	
	vector<cv::String> fn;
	vector<cv::String> fn1;

	cv::glob(path,fn,true); // recurse
	cv::glob(path1,fn1,true); // recurse

	int good=0;
	int total=0;

	total+=fn.size();
	total+=fn1.size();

	// Other first
	for (size_t k=0; k<fn.size(); ++k)
	{
	     cv::Mat im = cv::imread(fn[k]);
	     if (im.empty()) continue; //only proceed if sucsessful
	     // you probably want to do some preprocessing



	    int cyl=cylinder_classifier->classifyBest(im);
	    if(cyl==1)
		good++;

			/*if(confidence>classification_threshold)
			{
				cylinder_indices.push_back(i);

				// Visualization
				cv::rectangle(image_cv, cv::Point(min_pt[0],min_pt[1]), cv::Point(max_pt[0],max_pt[1]), cv::Scalar(0,255,0), 4);
			}*/
    /*cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
    cv::imshow( "Display window", im );                   // Show our image inside it.

    cv::waitKey(0);*/
	//	return 1;
	}







	for (size_t k=0; k<fn1.size(); ++k)
	{
	     cv::Mat im = cv::imread(fn1[k]);
	     if (im.empty()) continue; //only proceed if sucsessful
	     // you probably want to do some preprocessing


	    int cyl=cylinder_classifier->classifyBest(im);
	    if(cyl==0)
		good++;
	    else
	    	std::cout << fn1[k] << std::endl;

			/*if(confidence>classification_threshold)
			{
				cylinder_indices.push_back(i);

				// Visualization
				cv::rectangle(image_cv, cv::Point(min_pt[0],min_pt[1]), cv::Point(max_pt[0],max_pt[1]), cv::Scalar(0,255,0), 4);
			}*/

	}

	std::cout << "cylinder ratio:"<< (float)good/total<<std::endl;
	return (0);
}

