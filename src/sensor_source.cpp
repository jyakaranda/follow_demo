#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "follow_demo/dist_deg.h"
#include "follow_demo/alpha.h"

#include <cmath>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ml/ml.hpp>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <sys/time.h>

using namespace std;
using namespace cv;

#define PI 3.1415926
#define DEG2RAD(x) ((x)*PI/180.0)

class SensorSource{
    public:
        SensorSource();
        ~SensorSource();
        void init();
        void cb_laser_scan(const sensor_msgs::LaserScan::ConstPtr &laserMsg);
        void cb_camera(const sensor_msgs::Image::ConstPtr &img);
        void cb_odom(const nav_msgs::Odometry::ConstPtr odomMsg);

    private:
        ros::NodeHandle n_;
        ros::NodeHandle nh_;
        ros::Subscriber sub_laser_scan;
        ros::Subscriber sub_cv_camera;
        ros::Publisher pub_follow_wall;
        ros::Publisher pub_follow_image;
        ros::Publisher pub_obstacle;

        // double param_min_angle;
        // double param_max_angle;
        double param_dist2wall;
        bool param_debug;
        double param_deg_sample1;
        double param_deg_sample2;
        double param_accuracy_deg;
        double param_accuracy_dist;
        double param_max_dist;
        double param_laser_update_interval;
        int param_same_count;

        ros::Duration laser_update_duration;
        ros::Time last_laser_update;
        time_t last_camera_update;
        Size imageSize;
        Ptr<ml::SVM> mySVM;
        const std::string OPENCV_WINDOW = "Image window";

        double cal_deg(double a, double b, double C);
        double getEstimateDist(const sensor_msgs::LaserScan::ConstPtr& laserMsg, int index, int size);
        double getEstimateDeg(const sensor_msgs::LaserScan::ConstPtr& laserMsg, int index, int size);
        void coumputeHog(const Mat& src, vector<float> &descriptors);
};

SensorSource::SensorSource():nh_("~"){
    init();
    cv::namedWindow(OPENCV_WINDOW);
}

SensorSource::~SensorSource(){
    cv::destroyWindow(OPENCV_WINDOW);
}

void SensorSource::init(){

    nh_.param<double>("dist2wall", param_dist2wall, 0.5);
    nh_.param<bool>("debug", param_debug, false);
    nh_.param<double>("deg_sample1", param_deg_sample1, 30.0);
    nh_.param<double>("deg_sample2", param_deg_sample2, 60.0);
    nh_.param<double>("accuracy_deg", param_accuracy_deg, 5.0);
    nh_.param<double>("accuracy_dist", param_accuracy_dist, 0.05);
    nh_.param<double>("max_dist", param_max_dist, 1.0);
    nh_.param<double>("laser_update_interval", param_laser_update_interval, 0.5);
    nh_.param<int>("same_count", param_same_count, 7);
    param_deg_sample1 = DEG2RAD(param_deg_sample1);
    param_deg_sample2 = DEG2RAD(param_deg_sample2);
    param_accuracy_deg = DEG2RAD(param_accuracy_deg);

    last_laser_update.setNow(ros::Time(0, 0));
    last_camera_update = time(&last_camera_update);
    laser_update_duration.fromSec(param_laser_update_interval);

    sub_laser_scan = n_.subscribe<sensor_msgs::LaserScan>("/scan", 100, boost::bind(&SensorSource::cb_laser_scan, this, _1));
    sub_cv_camera = n_.subscribe<sensor_msgs::Image>("/cv_camera/image_raw", 10, boost::bind(&SensorSource::cb_camera, this, _1));
    pub_follow_wall = n_.advertise<follow_demo::dist_deg>("follow_wall", 100);
    pub_follow_image = n_.advertise<follow_demo::alpha>("follow_image", 100);

    imageSize = Size(64, 64);
    mySVM = ml::SVM::load("/home/jyakaranda/catkin_ws/src/follow_demo/88_mysvm.xml");
}

void SensorSource::cb_laser_scan(const sensor_msgs::LaserScan::ConstPtr &laserMsg){
    static int count = 0;

    if(laserMsg == 0){
        return;
    }

    if((laserMsg->header.stamp - last_laser_update) > laser_update_duration){
        ROS_INFO("cb_laser_scan: %d", count++);
        // ROS_INFO("accuracy_deg: %f, accuracy_dist: %f", param_accuracy_deg, param_accuracy_dist);

        if(laserMsg->angle_min+PI > param_deg_sample1){
            ROS_INFO("can not scan point in deg_sample !!!");
            return;
        }

        int index[5];
        index[0] = (param_deg_sample1-laserMsg->angle_min-PI) / laserMsg->angle_increment;
        index[1] = (param_deg_sample2-laserMsg->angle_min-PI) / laserMsg->angle_increment;
        index[2] = (DEG2RAD(90)-laserMsg->angle_min-PI) / laserMsg->angle_increment;
        index[3] = (DEG2RAD(90)+param_deg_sample1-laserMsg->angle_min-PI) / laserMsg->angle_increment;
        index[4] = (DEG2RAD(90)+param_deg_sample2-laserMsg->angle_min-PI) / laserMsg->angle_increment;

        follow_demo::dist_deg distDeg;
        double dist[5], estDeg[5];
        int i = 0;

        for(i = 0; i < 5; i++){
            dist[i] = getEstimateDist(laserMsg, index[i], 5);
            estDeg[i] = getEstimateDeg(laserMsg, index[i], 5);
            // ROS_INFO("i %d estDist: %f, estDeg: %f", i, dist[i], estDeg[i]);
        }

        if(dist[2] > param_max_dist){
            ROS_INFO("too far from the wall: %d m", dist[2]);
            return;
        }

        double deg[4];
        
        deg[0] = cal_deg(dist[2], dist[0], PI/2-estDeg[0]);
        deg[1] = cal_deg(dist[2], dist[1], PI/2-estDeg[1]);
        deg[2] = cal_deg(dist[2], dist[3], estDeg[3] - PI/2);
        deg[3] = cal_deg(dist[2], dist[4], estDeg[4] - PI/2);

        distDeg.deg = 0;
        for(i = 0; i < 4; i++){
            // ROS_INFO("deg %d: %lf", i, deg[i]);
            if(deg[i] > 0){
                if(abs(deg[i]-PI/2) < param_accuracy_deg){
                    break;
                } else if(distDeg.deg == 0){
                    if(i < 2){
                        distDeg.deg = deg[i] - PI/2;
                    } else {
                        distDeg.deg = PI/2 - deg[i];
                    }
                }
            }
        }

        if(abs(dist[2]-param_dist2wall) < param_accuracy_dist){
            // ROS_INFO("abs dist: %f, accuracy_dist: %f", abs(dist[2]-param_dist2wall), param_accuracy_dist);
            distDeg.dist_y = 0;
        } else{
            distDeg.dist_y = param_dist2wall - dist[2];
        }

        ROS_INFO("deg: %f, dist_y: %f", distDeg.deg, distDeg.dist_y);

        pub_follow_wall.publish(distDeg);
        last_laser_update = laserMsg->header.stamp;
    }
}

double SensorSource::cal_deg(double a, double b, double C){
    if(b > param_max_dist){
        return -1;
    }
    double c = sqrt(a*a + b*b - 2*a*b*cos(C));
    double B = acos((a*a + c*c - b*b)/(2*a*c));
    return B;
}

double SensorSource::getEstimateDist(const sensor_msgs::LaserScan::ConstPtr& laserMsg, int index, int size){
    int count = 0;
    double dist = 0.0;
    for(int i = index-size/2; i < index+size/2; i++){
        if(laserMsg->ranges[i] < param_max_dist){
            dist += laserMsg->ranges[i];
            count++;
        }
    }
    if(count == 0){
        // 范围内的点都是inf
        return param_max_dist + 1;
    } else{
        return dist/count;
    }
}
double SensorSource::getEstimateDeg(const sensor_msgs::LaserScan::ConstPtr& laserMsg, int index, int size){
    int count = 0;
    double deg = 0.0;
    for(int i = index-size/2; i < index+size/2; i++){
        if(laserMsg->ranges[i] < 3){
            deg += laserMsg->angle_min + laserMsg->angle_increment*i;
            count++;
        }
    }
    if(count == 0){
        return laserMsg->angle_min + laserMsg->angle_increment*index + PI;
    } else{
        return deg/count + PI;
    }
}

void SensorSource::cb_camera(const sensor_msgs::Image::ConstPtr &msg){
    static int count = 0;
    static int cntAlpha[3];
    time_t now;
    now = time(&now);
    // if(now - last_camera_update < laser_update_duration.toSec()){
    //     return;
    // }

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(30);

    cv::cvCvtColor(cv_ptr->image, cv_ptr->image,cv::CV_BGR2GRAY);
    resize(cv_ptr->image,cv_ptr->image,imageSize);
    vector<float> imageDescriptor;
    coumputeHog(cv_ptr->image,imageDescriptor);
    Mat testDescriptor = Mat::zeros(1, imageDescriptor.size(), CV_32FC1);
    for (size_t i = 0; i < imageDescriptor.size(); i++)
    {
        testDescriptor.at<float>(0, i) = imageDescriptor[i];
    }
    float label = mySVM->predict(testDescriptor);
    follow_demo::alpha a;
    a.alpha = (int)label;
    ROS_INFO("cb_camera %d: label %d", count++, a.alpha);
    if(a.alpha == 4){
        return;
    }
    cntAlpha[a.alpha-1]++;
    if(cntAlpha[a.alpha-1]%param_same_count == 0){
        for(int i = 0; i < 3; i++){
            cntAlpha[i] = 0;
        }
        pub_follow_image.publish(a);
        ROS_INFO("cb_camera %d: publish label %d", count, a.alpha);
    }

    last_camera_update = now;

}

void SensorSource::coumputeHog(const Mat& src, vector<float> &descriptors)
{
	HOGDescriptor myHog = HOGDescriptor(imageSize, Size(16, 16), cvSize(8, 8), cvSize(8, 8), 9);
	myHog.compute(src.clone(),descriptors,Size(1,1),Size(0,0));
}

int main(int argc, char** argv){
    
    ros::init(argc, argv, "sensor_source");

    ROS_INFO("sensor_source");

    SensorSource sensors;

    ROS_INFO("sensor_source ready");

    ros::spin();

    return 0;
}
