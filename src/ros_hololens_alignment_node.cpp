#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <iostream>
#include <string>
#include <sys/time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PolygonStamped.h>


float findTransform(std::vector<std::pair<cv::Mat, cv::Mat>> commonPoints, cv::Mat &R, cv::Mat &t); 

// Reference points relative to ROS map
// cv::Mat ros_p1(cv::Vec3f(2,-1,0));
// cv::Mat ros_p2(cv::Vec3f(0,0,0));
// cv::Mat ros_p3(cv::Vec3f(0,-1,0));
cv::Mat ros_p1(cv::Vec3f(1.3101,-1.2779,0));
cv::Mat ros_p2(cv::Vec3f(1.3951,1.0407,0));
cv::Mat ros_p3(cv::Vec3f(-3.0027,1.1805,0));

std::vector<std::pair<cv::Mat, cv::Mat>> alignmentPoints;


bool updated = false;

// Callback function
void hololensPointsCB(geometry_msgs::PolygonStampedConstPtr msg){
  alignmentPoints.clear();

  cv::Mat holo_p1(cv::Vec3f(msg->polygon.points[0].x, msg->polygon.points[0].y, msg->polygon.points[0].z));
  cv::Mat holo_p2(cv::Vec3f(msg->polygon.points[1].x, msg->polygon.points[1].y, msg->polygon.points[1].z));
  cv::Mat holo_p3(cv::Vec3f(msg->polygon.points[2].x, msg->polygon.points[2].y, msg->polygon.points[2].z));

  alignmentPoints.push_back(std::make_pair(holo_p1, ros_p1));
  alignmentPoints.push_back(std::make_pair(holo_p2, ros_p2));
  alignmentPoints.push_back(std::make_pair(holo_p3, ros_p3));

  updated = true;
}


//node that subscribes to 2 triangles and returns the corresponding transform (least squares)
int main(int argc, char **argv) {

  ros::init(argc, argv, "world_alignment_node");
  ros::NodeHandle node;

  ros::Subscriber holoPointsSub = node.subscribe("/hololens/reference_points", 1, &hololensPointsCB);
  tf::TransformBroadcaster br;

  ros::Rate rate(50);

  try{
    while(ros::ok()){
      if(updated){
        cv::Mat R, t;

        // Find transformation between the reference points
        float err = findTransform(alignmentPoints, R, t);
        ROS_INFO("Mean alignment error: %f", err);
        

        // Publishing on TF broadcaster
        tf::Vector3 origin((double) t.at<float>(0), (double) t.at<float>(1), (double) t.at<float>(2));
        tf::Matrix3x3 rotation(
            R.at<float>(0, 0), R.at<float>(0, 1), R.at<float>(0, 2),
            R.at<float>(1, 0), R.at<float>(1, 1), R.at<float>(1, 2),
            R.at<float>(2, 0), R.at<float>(2, 1), R.at<float>(2, 2));
        
        tf::Transform T_RH(rotation, origin); // ROS-Hololens Transformation 
       tf::StampedTransform TStamped(T_RH, ros::Time::now(), "map", "Unity");

        br.sendTransform(TStamped);
        updated = false;
      }

      ros::spinOnce();
      rate.sleep();
    }
  } catch(const std::exception &e){
    std::cout << e.what() << std::endl;
  }

  return 0;
}



float findTransform(std::vector<std::pair<cv::Mat, cv::Mat>> commonPoints, cv::Mat &R,
                    cv::Mat &t) 
{
  // commonPoints consist of (hololens, ROS) points
 
  // Implementation of Kabsch algorithm
  

  // Get centroid of both set of points
  cv::Mat centroid_holo(cv::Vec3f(0,0,0));
  cv::Mat centroid_ros(cv::Vec3f(0,0,0));

  for(auto it = commonPoints.begin(); it != commonPoints.end(); ++it){
    centroid_holo += it->first;
    centroid_ros += it->second;
  }
  centroid_holo = centroid_holo * (1.0/commonPoints.size());
  centroid_ros  = centroid_ros  * (1.0/commonPoints.size());

  // Calculate covariance matrix
  cv::Mat H(3,3,CV_32F, cv::Scalar(0));
  for(auto it = commonPoints.begin(); it != commonPoints.end(); ++it){
    H += (it->first - centroid_holo) * (it->second - centroid_ros).t();
  }

  // Get SVD of covariance matrix
  cv::SVD svd(H);
  cv::Mat v = svd.vt.t();
  cv::Mat ut = svd.u.t();

  // Get rotation matrix R and translation vector t
  float d = (cv::determinant(v*ut) > 0) ? 1 : -1;
  cv::Mat I = cv::Mat::eye(3, 3, CV_32F);
  I.at<float>(2,2) = d;

  R = v * I * ut;
  t = -(R * centroid_holo) + centroid_ros;


  // Find error of computed estimate
  float err = 0;
  for(auto it = commonPoints.begin(); it != commonPoints.end(); ++it){
    cv::Mat ros_est = R * it->first + t;
    cv::Mat delta = ros_est - it->second;
    err += cv::norm(delta);
  }
   
  err /= commonPoints.size();

  return err;
}
