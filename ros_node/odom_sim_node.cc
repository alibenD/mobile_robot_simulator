/**
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: odo_sim_node.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-10-29 23:14:40
  * @last_modified_date: 2018-10-29 23:42:38
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <mobile_robot_simulator/odometry_simulator.hh>

//CODE
int main(int argc, char** argv)
{
  using namespace ak;
  auto odom = OdomSimNode(argc, argv);
  odom.loop_run();
  return 0;
}
