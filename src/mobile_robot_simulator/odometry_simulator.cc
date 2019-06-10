/**
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: odometry_sim_node.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-02-29 16:59:46
  * @last_modified_date: 2019-06-10 16:58:38
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <mobile_robot_simulator/odometry_simulator.hh>
#include <cassert>

//CODE
namespace ak
{
  float OdomSimNode::joint2_stat_last_ = 0.;
  void OdomSimNode::cmd_callback(const geometry_msgs::Twist::ConstPtr& cmd)
  {
    static ros::Time last_callback_time = ros::Time::now();
    ros::Time new_callback_time = ros::Time::now();

    double dt = (new_callback_time - last_callback_time).toSec();

    ROS_INFO("Receive: vel_x: %lf, vel_y: %lf, vel_theta: %lf", cmd->linear.x, cmd->linear.y, cmd->angular.z);

    float vel_x = cmd->linear.x;
    float vel_y = cmd->linear.y;
    float vel_z = cmd->linear.z;
    float vel_theta = cmd->angular.z;

    // TODO: Limit speed.

    vel_.reset();
    vel_.linear_.x_ = vel_x;
    vel_.angular_.z_ = vel_theta;
    last_callback_time = new_callback_time;
    last_callback_ = last_callback_time;
  }

  OdomSimNode::OdomSimNode(int argc, char** argv, const std::string& node_name)
  {
    ros::init(argc, argv, node_name);
    ptr_nh_ = new ros::NodeHandle();
    ptr_broadcaster_ = new tf::TransformBroadcaster();
    odom_pub_ = ptr_nh_->advertise<nav_msgs::Odometry>("odom", 10);
    cmd_vel_sub_ = ptr_nh_->subscribe("cmd_vel", 10, &OdomSimNode::cmd_callback, this);
    ros::param::get("~max_heading_accel", max_heading_accel_);
    ros::param::get("~max_rotation_accel", max_rotation_accel_);
    ros::param::get("~cmd_timeout", cmd_timeout_);
    ros::param::get("~frequency", frequency_);
    ros::param::get("~child_frame_id", child_frame_id_);
    ros::param::get("~frame_id", frame_id_);
    ros::param::get("~axis_radius", axis_radius_);
    ros::param::get("~noise", flag_noise_);
    //ROS_INFO_STREAM( frame_id_ << " " << child_frame_id_);
    ptr_rate_ = new ros::Rate(frequency_);
  }

  void OdomSimNode::updateOdom()
  {
    float dx = 0.;
    float dy = 0.;
    float dt = (current_time_ - last_odom_time_).toSec();
    auto pose = this->getPose();
    auto theta = pose.orientation_.z_;
    double w = vel_.angular_.z_;
    double v = vel_.linear_.x_;

    // Motion part
    double new_x;
    double new_y;
    double new_theta = theta;
    double dth = w * dt;
    new_x = pose.position_.x_ + v*cos(theta)*dt;
    new_y = pose.position_.y_ + v*sin(theta)*dt;
    new_theta += dth;
    Pose new_pose(new_x, new_y, 0., 0., 0., new_theta);
    this->updatePose(new_pose);
    pose = this->getPose();
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(new_theta);
    odom_msg_.header.frame_id = frame_id_;
    //odom_msg_.header.frame_id = child_frame_id_;
    odom_msg_.header.stamp = current_time_;

    if(flag_noise_ == false)
    {
      odom_msg_.pose.pose.position.x = pose.position_.x_;
      odom_msg_.pose.pose.position.y = pose.position_.y_;
    }
    else
    {
      auto noise = generateGaussianNoise(0, 0.001);
      sum_noise_ += noise;
      noise_ = noise;
      odom_msg_.pose.pose.position.x = pose.position_.x_ + noise;
      odom_msg_.pose.pose.position.y = pose.position_.y_ + noise;
    }
    odom_msg_.pose.pose.position.z = pose.position_.z_;
    odom_msg_.pose.pose.orientation = quat;

    auto vel = this->getVelocity();
    odom_msg_.twist.twist.linear.x = vel_.linear_.x_;
    odom_msg_.twist.twist.linear.y = vel_.linear_.y_;
    odom_msg_.twist.twist.linear.z = vel_.linear_.z_;
    odom_msg_.twist.twist.angular.x = vel_.angular_.x_;
    odom_msg_.twist.twist.angular.y = vel_.angular_.y_;
    odom_msg_.twist.twist.angular.z = vel_.angular_.z_;
    last_odom_time_ = current_time_;
  }

  void OdomSimNode::updateTF()
  {
    odom_tf_.header.stamp = current_time_;
    odom_tf_.header.frame_id = frame_id_;
    odom_tf_.child_frame_id = child_frame_id_;

    auto pose = this->getPose();
    auto theta = pose.orientation_.z_;
    odom_tf_.transform.translation.x = pose.position_.x_;
    odom_tf_.transform.translation.y = pose.position_.y_;
    odom_tf_.transform.translation.z = pose.position_.z_;
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(theta);
    odom_tf_.transform.rotation = quat;
  }

  void OdomSimNode::boardcastTF()
  {
    ptr_broadcaster_->sendTransform(odom_tf_);
  }

  void OdomSimNode::publishOdom()
  {
    odom_pub_.publish(odom_msg_);
  }

  void OdomSimNode::loop_run()
  {
    current_time_ = ros::Time::now();
    last_time_ = ros::Time::now();
    last_odom_time_ = ros::Time::now();
    static size_t count_output = 2000;
    while(ptr_nh_->ok())
    {
      current_time_ = ros::Time::now();
      this->updateOdom();
      this->updateTF();
      this->boardcastTF();
      this->publishOdom();
      ros::spinOnce();
      if((current_time_ - last_callback_).toSec() > cmd_timeout_ && count_output<=0)
      {
        ROS_WARN("cmd_vel timeout, reset all velocity.");
        count_output = 2000;
        vel_.reset();
      }
      //else
      //{
      //  ROS_INFO_STREAM("Odom Error: " << noise_ << "(m), " << "Accumulation Error: " << sum_noise_ << "(m)");
      //}
      last_time_ = current_time_;
      --count_output;
      ptr_rate_->sleep();
    }
  }
}   // END of namespace ak
