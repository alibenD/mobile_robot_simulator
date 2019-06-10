/**
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: lidar_sim_node.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-01-05 14:57:04
  * @last_modified_date: 2019-06-10 11:11:46
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <mobile_robot_simulator/lidar_simulator.hh>

//CODE
int main(int argc, char** argv)
{
  using namespace ak;
  LidarSimulator lidar(argc, argv);
  //auto lidar = LidarSimulator();
  lidar.loop_run();
  return 0;
}
