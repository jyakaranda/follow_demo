#include "ros/ros.h"
#include "follow_demo/alpha.h"
#include "follow_demo/dist_deg.msg"
#include <geometry_msgs/Twist.h>
#include <iostream>

using namespace std;

#define PI 3.1415926

class robotBase{
    public:
        robotBase();
        void init(); 
        void cb_follow_wall(const follow_demo::dist_des::ConstPtr& distDeg);
        void cb_follow_image(const follow_demo::alpha::ConstPtr& alpha);
    private:
        ros::NodeHandle n_;
        ros::NodeHandle nh_;
        ros::Publisher pub_cmd_vel;
        ros::Subscriber sub_follow_wall;
        ros::Subscriber sub_follow_image;

        geometry_msgs::Twist Stwist;

        ros::Duration duration;
        ros::Time last_update;
        double param_vel_update_interval;
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
    duration.fromSec(param_vel_update_interval);
    last_update.setNow(ros::Time(0, 0));

    // geometry_msgs::Twist twist;
    // twist.linear.x = 0.2;
    // ros::Rate rate(10);
    // pub_cmd_vel.publish(twist);
    // rate.sleep();
    // pub_cmd_vel.publish(twist);

}

void cb_follow_wall(const follow_demo::dist_des::ConstPtr& distDeg){
    ros::Time now;
    if(now-last_update > duration){
        last_update = now;
        Stwist.linear.y = distDeg->dist_y;
        Stwist.angular.z = distDeg->deg;
        pub_cmd_vel.publish(Stwist);
        ROS_INFO("cb_follow_wall: linear_y: %f, angular_z: %f", distDeg->dist_y, distDeg->deg);

    }
}
void cb_follow_image(const follow_demo::alpha::ConstPtr& alpha){
    ros::Time now;
    if(now-last_update > duration){
        last_update = now;
        switch(alpha->alpha){
            case 1:
                // 前进
                Stwist.linear.x = 0.2;
                Stwist.angular.z = 0;
                break;
            case 2:
                // 右转
                Stwist.angular.z = -PI/2;
                break;`
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
        ROS_INFO("cb_follow_image: linear_x: %f, linear_y: %f, angular_z: %f", Stwist.linear.x, Stwist.linear.y, Stwist.angular.z);
    }
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "robot_base");

    robotBase base;
    ROS_INFO("robot_base ready");
    ros::spin();
    return 0;
}