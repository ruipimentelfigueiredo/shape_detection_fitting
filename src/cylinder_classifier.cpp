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



/*****************************************/
//		MAIN
/*****************************************/

int main(int argc, char** argv){

    // Init
    ::google::InitGoogleLogging(argv[0]);

    const string absolute_path_folder = string(argv[1]);
    const string model_file = absolute_path_folder + string(argv[2]);
    const string weight_file = absolute_path_folder + string(argv[3]);
    const string mean_file = absolute_path_folder + string(argv[4]);
    const string caffe_device = string(argv[5]);

    //const string ground_truth_labels = string(argv[14]);  // file with ground truth labels used to classification error
    //const string ground_bbox_dir = string(argv[15]);

    // Set mode
    if (strcmp(caffe_device.c_str(), "CPU") == 0){
	std::cout << "CPU MODE" << std::endl;
        Caffe::set_mode(Caffe::CPU);
	std::cout << "CPUasdasdE" << std::endl;
        //cout << "Using CPU\n" << endl;
    }
    else{
	std::cout << "GPU MODE" << std::endl;
        Caffe::set_mode(Caffe::CPU);
        //int device_id = atoi(argv[7]);
        //cout << "GPU "<< device_id << endl;
        //Caffe::SetDevice(device_id);
    }


    // Load network, pre-processment, set mean and load labels
    Network Network(model_file, weight_file, mean_file);

    std::cout << "yeah"<< std::endl;
    exit(0);



}


