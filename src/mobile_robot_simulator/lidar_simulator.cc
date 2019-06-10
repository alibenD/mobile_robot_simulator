/**
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: laser_simulator.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-01-02 13:34:33
  * @last_modified_date: 2019-06-10 10:58:22
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <mobile_robot_simulator/lidar_simulator.hh>

//CODE
namespace ak
{
  LidarSimulator::LidarSimulator(int argc, char** argv, const std::string& node_name)
    : min_dist_(0.),
      max_dist_(30.0),
      min_angle_(-M_PI/2),
      max_angle_(M_PI/2),
      num_of_line_(720),
      increment_((max_angle_ - min_angle_)/(num_of_line_ - 1)),
      freq_(30),
      global_frame_("world"),
      lidar_frame_("lidar")
  {
    ros::init(argc, argv, node_name);
    ptr_nh_ = std::make_shared<ros::NodeHandle>();
    ptr_tf_listener_ = std::make_shared<tf::TransformListener>();
    pub_ = ptr_nh_->advertise<sensor_msgs::LaserScan>("/scan", 1);
    sub_ = ptr_nh_->subscribe("/world", 1, &LidarSimulator::map_callback, this);
    obstacle_service_ = ptr_nh_->advertiseService("/obstacle_handle", &LidarSimulator::obstacleHandleServer, this);
    ros::param::get("~min_dist", min_dist_);
    ros::param::get("~max_dist", max_dist_);
    ros::param::get("~min_angle", min_angle_);
    ros::param::get("~max_angle", max_angle_);
    ros::param::get("~num_of_line", num_of_line_);
    ros::param::get("~freq", freq_);
    ros::param::get("~global_frame", global_frame_);
    ros::param::get("~lidar_frame", lidar_frame_);
    min_angle_ = min_angle_ * M_PI/180;
    max_angle_ = max_angle_ * M_PI/180;
    increment_ = ((max_angle_ - min_angle_)/(num_of_line_ - 1));
    ptr_rate_ = std::make_shared<ros::Rate>(freq_);
  }

  void LidarSimulator::loop_run()
  {
    ptr_tf_listener_->waitForTransform(global_frame_, lidar_frame_, ros::Time(), ros::Duration(1.0));
    while(ptr_nh_->ok())
    {
      ros::spinOnce();
      this->detectEnv();
      this->pubFrame();
      //ROS_INFO_STREAM("In Loop");
      //ROS_INFO_STREAM("Lidar Error(m): " << noise_);
      this->ptr_rate_->sleep();
    }
  }

  int LidarSimulator::pubFrame()
  {
    laser_scan_.header.stamp = ros::Time::now();
    laser_scan_.header.frame_id = lidar_frame_;
    laser_scan_.angle_min = min_angle_;
    laser_scan_.angle_max = max_angle_;
    laser_scan_.angle_increment = increment_;
    laser_scan_.time_increment = 1 / (double)num_of_line_ / freq_;
    laser_scan_.range_min = min_dist_;
    laser_scan_.range_max = max_dist_;
    pub_.publish(laser_scan_);
    return 0;
  }

  int LidarSimulator::detectEnv()
  {
    //tf::StampedTransform transform;
    double theta_at_world, roll, pitch;
    double x_at_world, y_at_world, distance;
    ptr_tf_listener_->lookupTransform(global_frame_, lidar_frame_, ros::Time(0), tf_);
    tf_.getBasis().getRPY(roll, pitch, theta_at_world);
    x_at_world = (double)tf_.getOrigin().getX();
    y_at_world = (double)tf_.getOrigin().getY();
    
    auto start_angle = theta_at_world - std::abs(min_angle_);
    //ROS_ERROR_STREAM("(X,Y): (" << x_at_world << ", " << y_at_world << ")" << " " << start_angle);

    laser_scan_.ranges.clear();
    noise_ = 0.;
    for(int i=0; i<num_of_line_; i++)
    {
      double lidar_line_theta = start_angle + increment_ * i;
      //ROS_ERROR_STREAM("HERE: " << lidar_line_theta);
      auto noise = generateGaussianNoise(0, 0.001);
      distance = env_map_.calDistance(x_at_world, y_at_world, lidar_line_theta) + noise;
      noise_ += noise*noise;
      //ROS_ERROR_STREAM("Distance: " << distance);
      if(distance >= min_dist_ && distance <= max_dist_)
      {
        laser_scan_.ranges.push_back(distance);
      }
      else if(distance > max_dist_)
      {
        laser_scan_.ranges.push_back(std::numeric_limits<float>::infinity());
      }
      else
      {
        laser_scan_.ranges.push_back(0.);
      }
    }
    noise_ = sqrt(noise_/num_of_line_);
    return 0;
  }

  void LidarSimulator::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& ptr_grid_map)
  {
    env_map_.initMap(ptr_grid_map);
    ROS_INFO_STREAM("In callback");
  }


  bool LidarSimulator::obstacleHandleServer(mobile_robot_simulator::Obstacle::Request &req,
                                            mobile_robot_simulator::Obstacle::Response &res)
  {
    ROS_ERROR_STREAM("Obstacle received!!!!!");
    if (req.type == req.TRANSFORM)
    {
      env_map_.transformObstacle(req.obstacle_id, req.transform[0], req.transform[1], req.transform[2]);
    }
    else if (req.type == req.NEW)
    {
      env_map_.insertObstacle(req.obstacle_id, req.vertex, req.transform[0], req.transform[1], req.transform[2]);
    }
    else if (req.type == req.DELETE)
    {
      env_map_.deleteObstacle(req.obstacle_id);
    }
    else
    {
      return false;
    }
    env_map_.updateObstacleData();
    return true;
  }
}   // END of namespace ak
