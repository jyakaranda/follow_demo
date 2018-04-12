#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Header.h>
#include "follow_demo/dist_deg.h"
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <cstdlib>

using namespace std;

#define PI 3.1415926
#define DEG2RAD(x) ((x)*PI/180.0)

class SensorSource{
    public:
        SensorSource();
        void init();
        void cb_laser_scan(const sensor_msgs::LaserScan::ConstPtr &laserMsg);
        void cb_camera();
        void cb_odom(const nav_msgs::Odometry::ConstPtr odomMsg);

    private:
        ros::NodeHandle n_;
        ros::NodeHandle nh_;
        ros::Subscriber sub_laser_scan;
        ros::Subscriber sub_camera;
        ros::Publisher pub_follow_wall;
        ros::Publisher pub_obstacle;

        ros::Duration obstacle_update_interval;
        // double param_min_angle;
        // double param_max_angle;
        double param_dist2wall;
        bool param_debug;
        double param_deg_sample1;
        double param_deg_sample2;
        double param_accuracy_deg;
        double param_accuracy_dist;
        double param_laser_update_interval;

        ros::Duration laser_update_duration;
        ros::Time last_laser_update;

        double cal_deg(double a, double b, double C);
};

SensorSource::SensorSource():nh_("~"){
    init();
}

void SensorSource::init(){

    nh_.param<double>("dist2wall", param_dist2wall, 0.5);
    nh_.param<bool>("debug", param_debug, false);
    nh_.param<double>("deg_sample1", param_deg_sample1, 30.0);
    nh_.param<double>("deg_sample2", param_deg_sample2, 60.0);
    nh_.param<double>("accuracy_deg", param_accuracy_deg, 5.0);
    nh_.param<double>("accuracy_dist", param_accuracy_dist, 0.05);
    nh_.param<double>("laser_update_interval", param_laser_update_interval, 0.5);
    param_deg_sample1 = DEG2RAD(param_deg_sample1);
    param_deg_sample2 = DEG2RAD(param_deg_sample2);
    param_accuracy_deg = DEG2RAD(param_accuracy_deg);

    last_laser_update.setNow(ros::Time(0, 0));
    laser_update_duration.fromSec(param_laser_update_interval);

    sub_laser_scan = n_.subscribe<sensor_msgs::LaserScan>("scan", 100, boost::bind(&SensorSource::cb_laser_scan, this, _1));
    // sub_camera = n_.subscribe<>
    pub_follow_wall = n_.advertise<follow_demo::dist_deg>("follow_wall", 100);
}

void SensorSource::cb_laser_scan(const sensor_msgs::LaserScan::ConstPtr &laserMsg){
    static int count = 0;

    if(laserMsg == 0){
        return;
    }

    if((laserMsg->header.stamp - last_laser_update) > laser_update_duration){
        ROS_INFO("cb_laser_scan: %d", count++);
        ROS_INFO("accuracy_deg: %f, accuracy_dist: %f", param_accuracy_deg, param_accuracy_dist);

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
        double dist[5];
        int i = 0;

        for(i = 0; i < 5; i++){
            dist[i] = laserMsg->ranges[index[i]];
            ROS_INFO("dist %d: %f", i, dist[i]);
            ROS_INFO("index %d, dist: %f", index[i], laserMsg->ranges[index[i]]);
        }

        if(dist[2] > 3){
            ROS_INFO("too far from wall: %d m", dist[3]);
            return;
        }

        double deg[4];
        
        deg[0] = cal_deg(dist[2], dist[0], PI/2-param_deg_sample1);
        deg[1] = cal_deg(dist[2], dist[1], PI/2-param_deg_sample2);
        deg[2] = cal_deg(dist[2], dist[3], param_deg_sample1);
        deg[3] = cal_deg(dist[2], dist[4], param_deg_sample2);

        distDeg.deg = 0;
        for(i = 0; i < 4; i++){
            if(deg[i] > 0){
                if(abs(deg[i]-PI/2) < param_accuracy_deg){
                    ROS_INFO("deg %d: %f, abs: %f, accuracy: %f", i, deg[i], abs(deg[i]-PI/2), param_accuracy_deg);
                    break;
                } else if(distDeg.deg == 0){
                    if(i < 2){
                        distDeg.deg = PI/2 - deg[i];
                    } else {
                        distDeg.deg = deg[i] - PI/2;
                    }
                }
            }

        }

        if(abs(dist[2]-param_dist2wall) < param_accuracy_dist){
            ROS_INFO("abs dist: %f, accuracy_dist: %f", abs(dist[2]-param_dist2wall), param_accuracy_dist);
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
    if(b> 3){
        return -1;
    }
    double c = sqrt(a*a + b*b - 2*a*b*cos(C));
    double B = acos((a*a + c*c - b*b)/(2*a*c));
    return B;
}

int main(int argc, char** argv){
    
    ros::init(argc, argv, "sensor_source");

    SensorSource sensors;

    ROS_INFO("sensor_source ready");

    ros::spin();

    return 0;
}