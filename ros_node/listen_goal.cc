/**
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: listen_goal.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-12-03 10:13:42
  * @last_modified_date: 2018-12-03 13:02:35
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <cmath>

double goal_x = 0.;
double goal_y = 0.;
double goal_theta = 0.;
//CODE
void goalCallback(const move_base_msgs::MoveBaseActionGoal& msg)
{
  goal_x = msg.goal.target_pose.pose.position.x;
  goal_y = msg.goal.target_pose.pose.position.y;
  auto orientation = msg.goal.target_pose.pose.orientation;
  tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
  double yaw, pitch, roll;
  mat.getEulerYPR(yaw, pitch, roll);
  goal_theta = yaw * 180 / M_PI;
}

void poseCallback(const nav_msgs::Odometry& msg)
{
  auto odom_x = msg.pose.pose.position.x;
  auto odom_y = msg.pose.pose.position.y;
  geometry_msgs::Quaternion orientation = msg.pose.pose.orientation;
  tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
  double yaw, pitch, roll;
  mat.getEulerYPR(yaw, pitch, roll);
  auto odom_theta = yaw * 180 / M_PI;

  ROS_INFO_STREAM("Goal: (" << goal_x << ","
                            << goal_y << "), dir: "
                            << goal_theta);
  ROS_INFO_STREAM("Odom: (" << odom_x << ","
                            << odom_y << "), dir: "
                            << odom_theta);
  ROS_INFO_STREAM("Residual Goal: " << "res_x: " << abs(odom_x - goal_x)
                                    << "res_y: " << abs(odom_y - goal_y)
                                    << "res_theta: " << abs(odom_theta - goal_theta));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Goal_Error");
  ros::NodeHandle nh;
  ros::Subscriber sub_goal = nh.subscribe("/move_base/goal", 2, goalCallback);
  ros::Subscriber sub_odom = nh.subscribe("/odom", 2, poseCallback);
  ros::spin();
  return 0;
}
