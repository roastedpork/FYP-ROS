#include "ros/ros.h"
#include <ros/package.h>
#include <string>
#include <list>
#include <iostream>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>


class Waypoints{
public:
  Waypoints() : newWPGoal(false), newGTGoal(false), goalTolerance(0.80)
  {
    stopPose.header.frame_id = "base_link";
    stopPose.pose.position.x = 0.0;
    stopPose.pose.position.x = 0.0;
    stopPose.pose.position.x = 0.0;

    stopPose.pose.orientation.x = 0.0;
    stopPose.pose.orientation.y = 0.0;
    stopPose.pose.orientation.z = 0.0;
    stopPose.pose.orientation.w = 1.0;



    wpSub_ = nh_.subscribe("/hololens/navigation/waypoints", 1, &Waypoints::wpCallback, this);
    ctSub_ = nh_.subscribe("/hololens/navigation/continuous_tracking", 1, &Waypoints::ctCallback, this);
    stopSub_ = nh_.subscribe("/hololens/navigation/stop", 1, &Waypoints::stopCallback, this);
    pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/navigation/move_base_simple/goal", 1);
  }

  void wpCallback(const geometry_msgs::PoseArrayConstPtr msg){

    wpGoals.clear();
    for(int i=0; i < msg->poses.size(); i++){
      try{
        geometry_msgs::PoseStamped in;
        in.header.stamp = ros::Time(0);
        in.header.frame_id = msg->header.frame_id;
        in.pose = msg->poses[i];        
  
        geometry_msgs::PoseStamped out;
  
        listener_.transformPose("/map", in, out);
        
        newWPGoal = true;
        wpGoals.push_back(out);

      } catch(const std::exception &e){
        std::cout << e.what() << std::endl;
      }
    
  
    }
  }


  void ctCallback(const geometry_msgs::PoseStampedConstPtr msg){
    
    wpGoals.clear();
    try{
      
      geometry_msgs::PoseStamped in;
      in.header.stamp = ros::Time(0);
      in.header.frame_id = msg->header.frame_id;
      in.pose = msg->pose;

  
      geometry_msgs::PoseStamped out;
  
      listener_.transformPose("/map", in, out);

      ctGoal = out;
  
      newGTGoal = true;

    } catch(const std::exception &e){
      std::cout << e.what() << std::endl;
    }
  }

  void stopCallback(const geometry_msgs::PoseStampedConstPtr msg){
    wpGoals.clear();
    newGTGoal = false;
    newWPGoal = false;
    stopGoal = true;

  }


  void run(){

    std::cout << "waypoint running" << std::endl;

    ros::Rate rate(10);
    while(ros::ok()){


      if (newGTGoal){
        newGTGoal = false;
        ctGoal.header.stamp = ros::Time(0);
        pub_.publish(ctGoal);

      } else if (newWPGoal){
        newWPGoal = false;
        pub_.publish(wpGoals.front());
        
      } else if (wpGoals.size() > 0){

        try{
          tf::StampedTransform transform;
          listener_.lookupTransform("base_link", "map", ros::Time(0), transform); 

          double dx = wpGoals.front().pose.position.x - transform.getOrigin().x();
          double dy = wpGoals.front().pose.position.y - transform.getOrigin().y();

          std::cout << "[waypoint] x=" << transform.getOrigin().x() << ", y=" << transform.getOrigin().y() << std::endl;
          std::cout << "[waypoint] dx=" << dx << ", dy=" << dy << std::endl;
          if(dx*dx + dy*dy < goalTolerance*goalTolerance) wpGoals.pop_front(); 

        } catch(const std::exception &e){
          std::cout << e.what() << std::endl;
        }
      } else if (stopGoal){
        stopGoal = false;
        stopPose.header.stamp = ros::Time(0);
        pub_.publish(stopPose);
      }

      ros::spinOnce();
      rate.sleep();

    }
  }

private:
  
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber wpSub_;
  ros::Subscriber ctSub_;
  ros::Subscriber stopSub_;

  bool newWPGoal;
  bool newGTGoal;
  bool stopGoal;

  geometry_msgs::PoseStamped stopPose;
  std::list<geometry_msgs::PoseStamped> wpGoals;
  geometry_msgs::PoseStamped ctGoal;
  
  double goalTolerance;



  tf::TransformListener listener_;

};



int main(int argc, char** argv){

  ros::init(argc, argv, "waypoint_node");
  
  std::cout << "main running" << std::endl;

  Waypoints wp;
  wp.run();
  return 0;

}
