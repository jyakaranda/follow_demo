#include "ros/ros.h"
#include "follow_demo/alpha.h"
#include <geometry_msgs/Twist.h>
#include <iostream>

using namespace std;



class robotBase{
    public:
        robotBase();
        void init(); 
    private:
        ros::NodeHandle n_;
        ros::NodeHandle nh_;

        double param_robot_width;
        ros::Publisher pub_cmd_vel;
        ros::Subscriber sub_sensor_source;
        
};

robotBase::robotBase():nh_("~"){
    init();
}

void robotBase::init(){
    ROS_INFO("robot_base init");

    pub_cmd_vel = n_.advertise<geometry_msgs::Twist>("cmd_vel", 50);
    geometry_msgs::Twist twist;
    twist.linear.x = 0.2;
    ros::Rate rate(10);
        pub_cmd_vel.publish(twist);
        rate.sleep();       
pub_cmd_vel.publish(twist);

}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "robot_base");


    robotBase base;
    ROS_INFO("robot_base");
    ros::spin();
    return 0;
}