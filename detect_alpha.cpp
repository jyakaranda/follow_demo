#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include<iostream>
#include<fstream>
#include<unistd.h>
#include<opencv2/ml/ml.hpp>

using namespace cv;
using namespace std;

class DetectAlpha{
    public:
        DetectAlpha();
        init();

    private:
        ros::NodeHandle n_;
        ros::NodeHandle nh_;
        ros::Publisher pub_;
};

int main(int argc, char* argv[]){
    ros::init(argc, argv, "detect_alpha");

    DetectAlpha detectAlpha;

    ROS_INFO("detect_alpha ready");

    return 0;
}