#ifndef __ODOMETRY_SIM_NODE_HH__
#define __ODOMETRY_SIM_NODE_HH__
/**-----------------------------------------------
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: odometry_sim_node.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-02-29 16:22:59
  * @last_modified_date: 2019-06-10 10:52:02
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

// Header include
#include <mobile_mobile_robot_simulator/odometry.hh>
#include <mobile_mobile_robot_simulator/function.hh>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <cmath>
// Declaration

namespace ak
{
  class OdomSimNode;

  /**
   * @brief Odometry ROS Node
   */
  class OdomSimNode : Odometry
  {
    public:
      OdomSimNode() : OdomSimNode(1, nullptr, "odom_simulator") {};
      OdomSimNode(const std::string& node_name)
        : OdomSimNode(1, nullptr, node_name) {};
      OdomSimNode(int argc,
                  char** argv,
                  const std::string& node_name="odom_simulator");

      /**
       * @brief An entry of Odometry simulator
       */
      void loop_run();

    protected:
      /**
       * @brief Callback to listen control command
       * @param[in] cmd Velocity command
       */
      void cmd_callback(const geometry_msgs::Twist::ConstPtr& cmd);

      /**
       * @brief Update odometry info
       */
      void updateOdom();

      /**
       * @brief Publish odometry topic
       */
      void publishOdom();

      /**
       * @brief Refresh TF Boradcast
       */
      void updateTF();

      /**
       * @brief Publish TF
       */
      void boardcastTF();

      void integrateRungeKutta2(double linear, double angular);
      
    private:
      ros::NodeHandle* ptr_nh_;
      tf::TransformBroadcaster* ptr_broadcaster_;
      geometry_msgs::TransformStamped odom_tf_;
      nav_msgs::Odometry odom_msg_;
      ros::Subscriber cmd_vel_sub_;
      ros::Publisher odom_pub_;
      ros::Rate* ptr_rate_;
      std::string frame_id_;
      std::string child_frame_id_;

      static float joint2_stat_last_;

      Velocity vel_;
      double max_heading_accel_;
      double max_rotation_accel_;
      double cmd_timeout_;
      double frequency_;
      double axis_radius_;
      bool flag_noise_;
      double sum_noise_;
      double noise_;
      ros::Time last_callback_;
      ros::Time current_time_;
      ros::Time last_odom_time_;
      ros::Time last_time_;
  };
}   // END of namespace ak
#endif // __ODOMETRY_SIM_NODE_HH__
