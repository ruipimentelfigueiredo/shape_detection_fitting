#include <caffe/caffe.hpp>
#include <caffe/util/io.hpp>
#include <caffe/blob.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <memory>
#include <math.h>
#include <limits>
#include<sstream>
//#include </usr/include/numpy/ndarrayobject.h>
//#include </usr/include/numpy/ndarraytypes.h>

#define CV_64FC3 CV_MAKETYPE(CV_64F,3)

//#include <boost/shared_ptr.hpp>
//#include <stdio>

using namespace caffe;
using namespace std;
using std::string;
using namespace cv;
//using cv::Mat;
//using namespace boost::numpy;

/* Pair (label, confidence) representing a prediction. */
typedef std::pair<string, float> Prediction;




/*****************************************/
/*		CLASSES
/*****************************************/

class ClassData{
public:
    ClassData(int N_): N(N_)    {  // construtor
      label.resize(N);
      score.resize(N);
      index.resize(N);
    }
    int N;
    std::vector<string> label;
    std::vector<float> score;
    std::vector<int> index;

    friend ostream &operator<<( ostream &output,const ClassData &D ) {
        for(int i=0; i<D.N;++i) {
            output << " Index: " << D.index[i] << "\n"
                   << " Label: " << D.label[i] << "\n"
                   << " Confidence: " << D.score[i] << "\n" << endl;
        }
        return output;
    }
};


/*****************************************/

class Network{
public:
    Network(const string& model_file,
            const string& weight_file,
            const string& mean_file); //construtor

    // Return Top 5 prediction of image in mydata
    ClassData Classify(const cv::Mat& img, int N);
    Rect CalcBBox(int N, int i, const cv::Mat &img, ClassData mydata, float thresh); // NEW
    void VisualizeBBox(std::vector<Rect> bboxes, int N, cv::Mat &img, int size_map, int ct);
    std::vector<String> GetDir(string dir, vector<String> &files);

    float* Limit_values(float* bottom_data); // NEW
    float find_max(Mat gradient_values);

private:
    void SetMean(const string& mean_file);
    void WrapInputLayer(std::vector<cv::Mat>* input_channels);
    void Preprocess(const cv::Mat& img, std::vector<cv::Mat>* input_channels);
    std::vector<float> Predict(const cv::Mat& img);

    int num_channels;
    std::shared_ptr<Net<float> > net;


    cv::Mat mean_;
    std::vector<string> labels;
    cv::Size input_geometry;		// size of network - width and height
};


/************************************************************************/
// Function Network
// Load network, mean file and labels
/************************************************************************/
Network::Network(const string& model_file,
                 const string& weight_file,
                 const string& mean_file)//, const string& label_file)
{

    // Load Network and set phase (TRAIN / TEST)
    net.reset(new Net<float>(model_file, TEST));

    // Load pre-trained net
    net->CopyTrainedLayersFrom(weight_file);

    // Set input layer and check number of channels
    Blob<float>* input_layer = net->input_blobs()[0];
    num_channels = input_layer->channels();
    CHECK(num_channels == 3 || num_channels == 1)
            << "Input layer should have 1 or 3 channels";

    input_geometry = cv::Size(input_layer->width(), input_layer->height());

    // Load mean file
    SetMean(mean_file);

    // Load labels
    /*std::ifstream labels2(label_file.c_str());   // vector with labels
    CHECK(labels2) << "Unable to open labels file " << label_file;
    std::string line;
    while (std::getline(labels2, line))
        labels.push_back(string(line));*/

    Blob<float>* output_layer = net->output_blobs()[0];
    /*CHECK_EQ(labels.size(), output_layer->channels())
                << "Number of labels is different from the output layer dimension.";*/
}

/************************************************************************/
// Function SetMean
// Create a mean image
/************************************************************************/
void Network::SetMean(const string& mean_file) {
    BlobProto blob_proto;
    ReadProtoFromBinaryFileOrDie(mean_file.c_str(), &blob_proto);
    // Convert from BlobProto to Blob<float>
    Blob<float> mean_blob;
    mean_blob.FromProto(blob_proto);			  // make copy
    CHECK_EQ(mean_blob.channels(), num_channels)
            << "Number of channels of mean file doesn't match input layer";


    // The format of the mean file is planar 32-bit float BGR or grayscale
    std::vector<cv::Mat> channels;
   // Mat* channels2;
    float* data = mean_blob.mutable_cpu_data();
    for (int i = 0; i < num_channels; ++i) {
        // Extract an individual channel
        cv::Mat channel(mean_blob.height(), mean_blob.width(), CV_32FC1, data);

        channels.push_back(channel);
        //channels2->push_back(channel);
        data += mean_blob.height() * mean_blob.width();
    }

    // Merge the separate channels into a single image
    cv::Mat mean;
    cv::merge(channels, mean);


    // Compute the global mean pixel value and create a mean image filled with this value
    cv::Scalar channel_mean = cv::mean(mean);
    mean_ = cv::Mat(input_geometry, mean.type(), channel_mean);
}


/************************************************************************/
// Function PairCompare
// Compare 2 pairs
/************************************************************************/
static bool PairCompare(const std::pair<float, int>& lhs,
                        const std::pair<float, int>& rhs) {
    return lhs.first > rhs.first;
}


/************************************************************************/
// Function Argmax
// Return the indices of the top N values of vector v
/************************************************************************/
static std::vector<int> Argmax(const std::vector<float>& v, int N) {
    std::vector<std::pair<float, int> > pairs;
    for (size_t i = 0; i < v.size(); ++i)
        pairs.push_back(std::make_pair(v[i], i));
    std::partial_sort(pairs.begin(), pairs.begin() + N, pairs.end(), PairCompare);

    std::vector<int> result;
    for (int i = 0; i < N; ++i)
        result.push_back(pairs[i].second);

    return result;
}


/************************************************************************/
// Function Classify
// Return the top N predictions
/************************************************************************/
ClassData Network::Classify(const cv::Mat& img, int N) {
    std::vector<float> output = Predict(img);  // output is a float vector

    ClassData mydata(N); // objecto

    N = std::min<int>(labels.size(), N);       // tem 5 top labels
    std::vector<int> maxN = Argmax(output, N); // tem o top

    for (int i = 0; i < N; ++i) {
        int idx = maxN[i];

        mydata.index[i] = idx;
        mydata.label[i] = labels[idx];
        mydata.score[i] = output[idx];
    }
//    cout << mydata << endl;  // imprime os dados do top 5

    return mydata;
}

/************************************************************************/
// Function Predict
// wrap input layers and make preprocessing
/************************************************************************/
std::vector<float> Network::Predict(const cv::Mat& img) {
    Blob<float>* input_layer = net->input_blobs()[0];

    input_layer->Reshape(1, num_channels, input_geometry.height, input_geometry.width);

    // Forward dimension change to all layers
    net->Reshape();

    std::vector<cv::Mat> input_channels;
    WrapInputLayer(&input_channels);

    // Convert the input image to the input image format of the network
    Preprocess(img, &input_channels);

    net->Forward();

    // Copy the output layer to a std::vector
    Blob<float>* output_layer = net->output_blobs()[0];
    //boost::shared_ptr<caffe::Blob<float> > output_layer = net->blob_by_name("fc8");
    const float* begin = output_layer->cpu_data();      // output of forward pass
    const float* end = begin + output_layer->channels();

    return std::vector<float>(begin, end);
}


/************************************************************************/
// Function WrapInputLayer
// Wrap the input layer of the network in separate cv::Mat objects (one per channel)
// The last preprocessing operation will write the separate channels directly to the input layer.
/************************************************************************/
void Network::WrapInputLayer(std::vector<cv::Mat>* input_channels){
    Blob<float>* input_layer = net->input_blobs()[0];

    int width = input_layer->width();
    int height = input_layer->height();
    float* input_data = input_layer->mutable_cpu_data();

    for (int i = 0; i < input_layer->channels(); ++i) {
        cv::Mat channel(height, width, CV_32FC1, input_data);
        input_channels->push_back(channel);
        input_data += width * height;
    }
}


/************************************************************************/
// Function Preprocess
// Subtract mean, swap channels, resize input
/************************************************************************/
void Network::Preprocess(const cv::Mat& img, std::vector<cv::Mat>* input_channels){

    // Convert the input image to the input image format of the network
    // swap channels from RGB to BGR
    cv::Mat sample;
    if (img.channels() == 3 && num_channels == 1)
        cv::cvtColor(img, sample, cv::COLOR_BGR2GRAY);
    else if (img.channels() == 4 && num_channels == 1)
        cv::cvtColor(img, sample, cv::COLOR_BGRA2GRAY);
    else if (img.channels() == 4 && num_channels == 3)
        cv::cvtColor(img, sample, cv::COLOR_BGRA2BGR);
    else if (img.channels() == 1 && num_channels == 3)
        cv::cvtColor(img, sample, cv::COLOR_GRAY2BGR);
    else
        sample = img;

    // Resize if geometry of image != input geometry of the network
    cv::Mat sample_resized;
    if (sample.size() != input_geometry)
        cv::resize(sample, sample_resized, input_geometry);
    else
        sample_resized = sample;

    cv::Mat sample_float;
    if (num_channels == 3)		// RGB
        sample_resized.convertTo(sample_float, CV_32FC3);
    else
        sample_resized.convertTo(sample_float, CV_32FC1);

    // Subtract the dataset-mean value in each channel
    cv::Mat sample_normalized;
    cv::subtract(sample_float, mean_, sample_normalized);

    /* This operation will write the separate BGR planes directly to the
       input layer of the network because it is wrapped by the cv::Mat
       objects in input_channels. */
    cv::split(sample_normalized, *input_channels);

    CHECK(reinterpret_cast<float*>(input_channels->at(0).data)
          == net->input_blobs()[0]->cpu_data())
            << "Input channels are not wrapping the input layer of the network.";
}



/************************************************************************/
// Function CalcRGBmax
// Get the highest value of R,G,B for each pixel
/************************************************************************/

Mat CalcRGBmax(Mat i_RGB) {

    std::vector<cv::Mat> planes(3);

    cv::split(i_RGB, planes);

    Mat maxRGB = max(planes[2], cv::max(planes[1], planes[0]));

    return maxRGB;
}



/************************************************************************/
// Function CalcBBox
/************************************************************************/
Rect Network::CalcBBox(int N, int i, const cv::Mat& img, ClassData mydata, float thresh ){

    //std::vector<Rect> bboxes;

    // For each predicted class (top 5)
    //for (int i = 0; i < N; ++i) {

    /*********************************************************/
    //                  Get Saliency Map                     //
    //              Backward and normalize                   //
    /*********************************************************/

    int label_index = mydata.index[i];  // tem o indice da classe

    // Dados do 1 forward
    Blob<float>* forward_output_layer = net->output_blobs()[0];
    float* fc8Data = forward_output_layer->mutable_cpu_data();
    float* fc8Diff = forward_output_layer->mutable_cpu_diff();

    // Backward of a specific class
    for (int i = 0;  i< forward_output_layer->num() * forward_output_layer->channels() * forward_output_layer->height() * forward_output_layer->width(); ++i)
        fc8Diff[i] = 0.0f;

    fc8Diff[label_index] = 1.0f; // Specific class

    // Backward
    net->Backward();

    // Get Data
    boost::shared_ptr<caffe::Blob<float> > out_data_layer = net->blob_by_name("data");  // get data from Data layer
    int dim = out_data_layer->num() * out_data_layer->channels() * out_data_layer->height() * out_data_layer->width();

    const float* begin_diff = out_data_layer->mutable_cpu_diff();

    Mat M2 = Mat(out_data_layer->height(),out_data_layer->width(),CV_32FC3);

    for (int i=0; i<out_data_layer->height(); ++i){
        for(int j=0; j< out_data_layer->width(); ++j){
            for (int c=0; c<3; ++c){

                int index =  j + i*out_data_layer->width() + c*out_data_layer->width()*out_data_layer->height();
                M2.at<Vec3f>(i,j)[c] = begin_diff[index];
            }
        }
    }

    cv::normalize(M2, M2, 0, 1, NORM_MINMAX);

    // Find max across RGB channels
    Mat saliency_map = CalcRGBmax(M2);

//    imshow("saliency", saliency_map);
//    waitKey(0);


    /*********************************************************/
    //                  Segmentation Mask                    //
    //       Set pixels > threshold to 1 and define box      //
    /*********************************************************/

    Mat foreground_mask;
    threshold(saliency_map, foreground_mask, thresh, 1, THRESH_BINARY);

//    imshow("Mask", foreground_mask);
//    waitKey(0);
    foreground_mask.convertTo(foreground_mask,CV_8UC1);

    Mat Points;
    findNonZero(foreground_mask,Points);
    Rect Min_Rect = boundingRect(Points);

    //bboxes.push_back(Min_Rect);

    //}

    return Min_Rect;

}

/************************************************************************/
// Function GetDir                                                      //
// Get list of image files on given directory                           //
/************************************************************************/

std::vector<String> Network::GetDir(string dir, vector<String> &files) {
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL) {
        cout << "Error(" << errno << ") opening " << dir << endl;
    }

    while ((dirp = readdir(dp)) != NULL) {
        files.push_back(string(dirp->d_name));
    }

    closedir(dp);

    return files;
}





/************************************************************************/
// Function VisualiseBbox
/************************************************************************/

void Network::VisualizeBBox(std::vector<Rect> bboxes, int N, cv::Mat& img, int size_map, int ct){

    // Transformation from (227*227) to (height*width) of input image
    for (int k =0; k< N; ++k){

        bboxes[k].x =  bboxes[k].x*img.size().width / size_map;
        bboxes[k].y =  bboxes[k].y*img.size().height / size_map;
        bboxes[k].width = bboxes[k].width*img.size().width / size_map;
        bboxes[k].height = bboxes[k].height*img.size().height / size_map;
    }

    for (int k =0; k< N; ++k)

        rectangle(img, bboxes[k], Scalar(0, 0, 255), 1, 8, 0 );

    stringstream ss;

    string name = "Figures/bbox_";
    string type = ".jpg";

    ss<<name<<(ct + 1)<<type;

    string filename = ss.str();
    ss.str("");

    imwrite(filename, img);
    imshow("box", img );
    waitKey(0);
}






/************************************************************************/
// Function Limit Values
// Find min and max value of vector
// Return bottom_data normalized
/************************************************************************/
float* Network::Limit_values(float* bottom_data){
    float smallest = bottom_data[0];
    float largest = bottom_data[0];
    for (int i=1; i<sizeof(bottom_data); i++) {
        if (bottom_data[i] < smallest)
            smallest = bottom_data[i];
        if (bottom_data[i] > largest)
            largest= bottom_data[i];
    }
    std::vector<float> result;
    result.push_back(smallest);
    result.push_back(largest);

    // Normalize
    for (int i=0; i< sizeof(bottom_data); ++i){
        bottom_data[i] = bottom_data[i]-result[0];
        bottom_data[i] = bottom_data[i]/result[1];
        //cout << bottom_data[i] << "\n" << endl;
    }

    return bottom_data;

}
