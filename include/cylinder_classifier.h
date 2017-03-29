#include <caffe/caffe.hpp>
#include <caffe/util/io.hpp>
#include <caffe/blob.hpp>
#include <caffe/layer.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <vector>
#include <memory>
#include <math.h>
#include <limits>
#include <boost/algorithm/string.hpp>

#include "network_classes.hpp"
//#include "laplacian_foveation.hpp"


using namespace caffe;
using namespace std;

using std::string;


/* Pair (label, confidence) representing a prediction. */
typedef std::pair<string, float> Prediction;



class CylinderClassifier
{

	std::string absolute_path_folder;
	std::string model_file;
	std::string weight_file;
	std::string mean_file;
	std::string device;
	unsigned int device_id;
	boost::shared_ptr<Network> network; 
public:
	CylinderClassifier(const std::string & absolute_path_folder_,
			   const std::string & model_file_,
			   const std::string & weight_file_,
			   const std::string & mean_file_,
		           const std::string & device_,
			   const unsigned int & device_id_=0);

	float classify(const cv::Mat& img);
	int classifyBest(const cv::Mat& img);
};





