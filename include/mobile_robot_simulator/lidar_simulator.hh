#ifndef __LASER_SIMULATOR_HH__
#define __LASER_SIMULATOR_HH__
/**-----------------------------------------------
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: lidar_simulator.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-01-02 13:36:02
  * @last_modified_date: 2019-06-10 10:58:24
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

// Header include
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <mobile_mobile_robot_simulator/environment_map.hh>
#include <mobile_mobile_robot_simulator/Obstacle.h>
#include <mobile_mobile_robot_simulator/function.hh>
#include <memory>

#include <cmath>
#include <limits>

// Declaration
namespace ak
{
  class LidarSimulator;
  class LidarSimulator
  {
    public:
      /**
       * @brief Default constructor
       */
      LidarSimulator() : LidarSimulator(1, nullptr, "lidar_simulator") {};

      /**
       * @brief Constructor with a string(node name)
       * @param[in] node_name
       */
      LidarSimulator(const std::string& node_name)
        : LidarSimulator(1, nullptr, node_name) {};

      /**
       * @brief Constructor with cmd params and node name
       * @param[in] argc
       * @param[in] argv
       * @param[in] node_name
       */
      LidarSimulator(int argc,
                     char** argv,
                     const std::string& node_name="lidar_simulator");
      ~LidarSimulator() = default;

      /**
       * @brief An entry of lidar simulator running
       */
      void loop_run();

    protected:
      /**
       * @brief Publish scan frame
       */
      int pubFrame();

      /**
       * @brief scan virtual environment
       */
      int detectEnv();

      /**
       * @brief callback for receiving virtual environment map
       * @param[in] ptr_grid_map const pointer of OccupancyGrid
       */
      void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& ptr_grid_map);

      bool obstacleHandleServer(mobile_mobile_robot_simulator::Obstacle::Request &req,
                                mobile_mobile_robot_simulator::Obstacle::Response &res);

    private:
      double min_dist_;   /*!< The minimum distance of lidar detectable*/
      double max_dist_;   /*!< The maximum distance of lidar detectable*/
      double min_angle_;  /*!< The minimum angle of lidar detectable*/
      double max_angle_;  /*!< The maximum angle of lidar detectable*/
      int num_of_line_;   /*!< The number of laser lines*/
      double increment_;  /*!< The resolution of laser sensor*/
      double freq_;       /*!< The frequency of this laser sensor working*/
      double noise_;
      std::string global_frame_;
      std::string lidar_frame_;

      std::shared_ptr<ros::NodeHandle> ptr_nh_;   
      ros::Publisher pub_;
      ros::Subscriber sub_;
      std::shared_ptr<tf::TransformListener> ptr_tf_listener_;
      tf::StampedTransform tf_;
      std::shared_ptr<ros::Rate> ptr_rate_;
      EnvironmentMap env_map_;          /*!< The maximum distance of lidar detectable*/
      sensor_msgs::LaserScan laser_scan_;
      ros::ServiceServer obstacle_service_;
  };
}   // END of namespace ak
#endif // __LASER_SIMULATOR_HH__
