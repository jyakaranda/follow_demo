#include "ros/ros.h"
#include "ros/time.h"
#include "follow_demo/alpha.h"
#include "follow_demo/dist_deg.h"
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <sys/time.h>
#include <cmath>
#include <cstdlib>

using namespace std;

#define PI 3.1415926

class robotBase{
    public:
        robotBase();
        void init(); 
        void cb_follow_wall(const follow_demo::dist_deg::ConstPtr& distDeg);
        void cb_follow_image(const follow_demo::alpha::ConstPtr& alpha);
    private:
        ros::NodeHandle n_;
        ros::NodeHandle nh_;
        ros::Publisher pub_cmd_vel;
        ros::Subscriber sub_follow_wall;
        ros::Subscriber sub_follow_image;

        geometry_msgs::Twist Stwist;

        ros::Duration duration;
        time_t last_update;
        follow_demo::alpha lastAlpha;
        double param_vel_update_interval;
        double param_max_linear_speed;
        double param_max_angular_speed;
};

robotBase::robotBase():nh_("~"){
    init();
}

void robotBase::init(){
    ROS_INFO("robot_base init");

    pub_cmd_vel = n_.advertise<geometry_msgs::Twist>("cmd_vel", 50);
    sub_follow_wall = n_.subscribe<follow_demo::dist_deg>("follow_wall", 100, boost::bind(&robotBase::cb_follow_wall, this, _1));
    sub_follow_image = n_.subscribe<follow_demo::alpha>("follow_image", 100, boost::bind(&robotBase::cb_follow_image, this, _1));

    nh_.param<double>("vel_update_interval", param_vel_update_interval, 1.0);
    nh_.param<double>("max_linear_speed", param_max_linear_speed, 0.2);
    nh_.param<double>("max_angular_speed", param_max_angular_speed, PI/2);
    duration.fromSec(param_vel_update_interval);
    last_update = time(&last_update);

}

void robotBase::cb_follow_wall(const follow_demo::dist_deg::ConstPtr& distDeg){
    time_t now;
    now = time(&now);
    if(now-last_update > duration.toSec()){
        ROS_INFO("cb_follow_wall %d, %d", now, last_update);
        last_update = now;
        Stwist.linear.y = abs(distDeg->dist_y)>param_max_linear_speed ? abs(distDeg->dist_y)/distDeg->dist_y*param_max_linear_speed : distDeg->dist_y;
        Stwist.angular.z = abs(distDeg->deg)>param_max_angular_speed ? abs(distDeg->deg)/distDeg->deg*param_max_angular_speed : distDeg->deg;
        pub_cmd_vel.publish(Stwist);
        ROS_INFO("cb_follow_wall: linear_y: %f, angular_z: %f", Stwist.linear.y, Stwist.angular.z);
    }
}
void robotBase::cb_follow_image(const follow_demo::alpha::ConstPtr& alpha){
    time_t now;
    now = time(&now);
    if(alpha->alpha == lastAlpha.alpha){
        ROS_INFO("same alpha %d", alpha->alpha);
        return;
    }
    if(now-last_update > duration.toSec()){
        ROS_INFO("cb_follow_image %d, %d", now, last_update);
        last_update = now;
        lastAlpha.alpha = alpha->alpha;
        switch(alpha->alpha){
            case 1:
                // 前进
                Stwist.linear.x = 0.2;
                Stwist.angular.z = 0;
                break;
            case 2:
                // 右转
                Stwist.linear.x = 0.2;
                Stwist.angular.z = -PI/1.9;
                break;
            case 3:
                // 停止
                Stwist.linear.x = 0;
                Stwist.linear.y = 0;
                Stwist.angular.z = 0;
                break;
            default:
                break;
        }
        pub_cmd_vel.publish(Stwist);
        if(alpha->alpha == 2){
            Stwist.angular.z = 0;
            ros::Rate rate(1);
            for(int i = 0; i < 7; i++){
                rate.sleep();
                pub_cmd_vel.publish(Stwist);
            }
        }
        ROS_INFO("cb_follow_image: alpha: %d,  linear_x: %f, linear_y: %f, angular_z: %f", alpha->alpha, Stwist.linear.x, Stwist.linear.y, Stwist.angular.z);
    }
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "robot_base");

    robotBase base;
    ROS_INFO("robot_base ready");
    ros::spin();
    return 0;
}