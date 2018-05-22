#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"



double target_vel = 0.0;
double curr_vel = 0.0;

double target_rot = 0.0;
double curr_rot = 0.0;


void setNewTargetCB(const geometry_msgs::TwistConstPtr &msg){
    target_vel = msg->linear.x;
    target_rot = msg->angular.z;
}



int main(int argc, char** argv){

    ros::init(argc, argv, "smooth_js_node");

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/navigation/main_js_cmd_vel", 1, &setNewTargetCB);
    ros::Publisher vel_pub = nh.advertise<std_msgs::Float64>("/hololens/display/smoothed_velocity", 1);
    ros::Publisher rot_pub = nh.advertise<std_msgs::Float64>("/hololens/display/smoothed_rotation", 1);

    ros::Rate rate(10);
    double gain = 0.75;

    while(ros::ok()){
        curr_vel += (target_vel - curr_vel) * gain;
        std_msgs::Float64 vel_msg;
        vel_msg.data = round(curr_vel * 1000.0) / 1000.0 ;
        vel_pub.publish(vel_msg);

        curr_rot += (target_rot - curr_rot) * gain;
        std_msgs::Float64 rot_msg;
        rot_msg.data = round(curr_rot * 1000.0) / 1000.0;
        rot_pub.publish(rot_msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;

}
