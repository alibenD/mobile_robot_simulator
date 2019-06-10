/**
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: function.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-12-03 09:33:04
  * @last_modified_date: 2019-06-10 10:48:21
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <mobile_mobile_mobile_robot_simulator/function.hh>

//CODE
namespace ak
{
  double generateGaussianNoise(double mu, double sigma)
  {
    static double V1, V2, S;
    static int phase = 0;
    double X;
    double U1,U2;
    if ( phase == 0 )
    {
      do
      {
         U1 = (double)rand() / RAND_MAX;
         U2 = (double)rand() / RAND_MAX;
         V1 = 2 * U1 - 1;
         V2 = 2 * U2 - 1;
         S = V1 * V1 + V2 * V2;
      }while(S >= 1 || S == 0);
      X = V1 * sqrt(-2 * log(S) / S);
    }
    else
    {
      X = V2 * sqrt(-2 * log(S) / S);
    }
    phase = 1 - phase;
    return mu+sigma*X;
  }
}
