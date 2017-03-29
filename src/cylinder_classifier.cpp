#include "cylinder_classifier.h"
using namespace caffe;
using namespace std;

using std::string;


/* Pair (label, confidence) representing a prediction. */
typedef std::pair<string, float> Prediction;


CylinderClassifier::CylinderClassifier(const std::string & absolute_path_folder_,
			   const std::string & model_file_,
			   const std::string & weight_file_,
			   const std::string & mean_file_,
		           const std::string & device_,
			   const unsigned int & device_id_) : absolute_path_folder(absolute_path_folder_), model_file(model_file_), weight_file(weight_file_), mean_file(mean_file_), device(device_), device_id(device_id_)
			   
{
	// Set mode
	if (strcmp(device.c_str(), "CPU") == 0){
		std::cout << "CPU MODE" << std::endl;
		Caffe::set_mode(Caffe::CPU);
	}
	else{
		std::cout << "GPU MODE" << std::endl;
		Caffe::set_mode(Caffe::GPU);
		//int device_id = atoi(argv[7]);
		//cout << "GPU "<< device_id << endl;
		//Caffe::SetDevice(device_id);
	}

	// Load network, pre-processment, set mean and load labels
	network=boost::shared_ptr<Network> (new Network(absolute_path_folder+model_file, absolute_path_folder+weight_file, absolute_path_folder+mean_file));
}		

int CylinderClassifier::classifyBest(const cv::Mat& img)
{

    return network->ClassifyBest(img);
}

float CylinderClassifier::classify(const cv::Mat& img)
{
	std::vector<ClassificationData> classifications=network->Classify(img);
	int best_index=0;
	float confidence=0;
	float sum=0.0;
	/*for(unsigned int i=0; i<classifications.size();++i)
	{
		sum+=classifications[i].confidence;
	}

	// Normalize
	for(unsigned int i=0; i<classifications.size();++i)
	{
		classifications[i].confidence/=sum;
	}*/
	
	return classifications[0].confidence;
}


	
