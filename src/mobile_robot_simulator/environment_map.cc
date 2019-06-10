/**
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: environment_map.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-01-02 13:41:20
  * @last_modified_date: 2019-06-10 10:46:05
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <mobile_mobile_mobile_robot_simulator/environment_map.hh>
#include <iostream>

//CODE
namespace ak
{
  int EnvironmentMap::initMap(const nav_msgs::OccupancyGrid::ConstPtr& ptr_grid_map)
  {
    size_x_ = ptr_grid_map->info.width;
    size_y_ = ptr_grid_map->info.height;
    origin_at_world_x_ = ptr_grid_map->info.origin.position.x;
    origin_at_world_y_ = ptr_grid_map->info.origin.position.y;
    resolution_ = ptr_grid_map->info.resolution;
    auto length = size_x_ * size_y_;
    meta_environment_data_ = new unsigned char[length];
    obstacle_data_ = new unsigned char[length];
    for(int i=0; i<length; i++)
    {
      meta_environment_data_[i] = ptr_grid_map->data.at(i);
      //std::cout << (int)meta_environment_data_[i] << " ";
    }
    //std::cout << std::endl;
    exist_obstacle_ = false;
    return 0;
  }

  int EnvironmentMap::mapToWorld(int map_x, int map_y,
                                 double& world_x, double& world_y)
  {
    world_x = (map_x + 0.5) * resolution_ + origin_at_world_x_;
    world_y = (map_y + 0.5) * resolution_ + origin_at_world_y_;
    return 0;
  }

  int EnvironmentMap::worldToMap(double world_x, double world_y,
                                 int& map_x, int& map_y)
  {
    map_x = (int)( (world_x - origin_at_world_x_) / resolution_ );
    map_y = (int)( (world_y - origin_at_world_y_) / resolution_ );
    return 0;
  }

  unsigned char EnvironmentMap::getGridCell(double world_x, double world_y)
  {
    int map_x, map_y;
    worldToMap(world_x, world_y, map_x, map_y);
    return getGridCell(map_x, map_y);
  }

  unsigned char EnvironmentMap::getGridCell(int map_x, int map_y)
  {
    auto position = map_x + map_y * size_x_;
    static auto size_map = size_x_ * size_y_;
    if(position > size_map || position < 0)
    {
      return ERROR_SPACE;
    }
    else
    {
      if(exist_obstacle_)
      {
        return obstacle_data_[position] == OCCUPIED_SPACE ? OCCUPIED_SPACE: meta_environment_data_[position];
      }
      else
      {
        return meta_environment_data_[position];
      }
    }
  }

  int EnvironmentMap::resetGridCell(double world_x, double world_y)
  {
    int map_x, map_y;
    worldToMap(world_x, world_y, map_x, map_y);
    return resetGridCell(map_x, map_y);
  }

  int EnvironmentMap::resetGridCell(int map_x, int map_y)
  {
    auto position = map_x + map_y * size_x_;
    meta_environment_data_[position] = -1;
    return 0;
  }

  int EnvironmentMap::setGridCell(double world_x, double world_y, unsigned char value)
  {
    int map_x, map_y;
    worldToMap(world_x, world_y, map_x, map_y);
    return setGridCell(map_x, map_y, value);
  }

  int EnvironmentMap::setGridCell(int map_x, int map_y, unsigned char value)
  {
    auto position = map_x + map_y * size_y_;
    meta_environment_data_[position] = value;
    return 0;
  }

  double EnvironmentMap::normalizeAngle(double theta)
  {
    if(theta > M_PI) 
    {
      theta = normalizeAngle(theta - 2 * M_PI);
    }
    else if(theta < -M_PI)
    {
      theta = normalizeAngle(theta + 2 * M_PI);
    }
    return theta;
  }

  double EnvironmentMap::noise(double u, double error)
  {

    std::default_random_engine e;
    std::normal_distribution<double> n(u, error);
    return n(e);
  }

  double EnvironmentMap::distance(int start_x, int start_y, int end_x, int end_y)
  {
    return sqrt(pow(end_x - start_x, 2) + pow(end_y - start_y, 2));
  }

  double EnvironmentMap::calDistance(double world_x, double world_y, double theta)
  {
    int map_x, map_y;
    worldToMap(world_x, world_y, map_x, map_y);
    calDistance(map_x, map_y, theta);
  }

  double EnvironmentMap::calDistance(int map_x, int map_y, double theta)
  {
    auto theta_normalized = normalizeAngle(theta);
    double k, e=-0.5;
    int major_step, minor_step;
    int end_x = map_x;
    int end_y = map_y;
    // Check if lidar pose exceed boundary
    if (outOfMap(end_x, end_y))
    {
      //return distance(map_x, map_y, end_x, end_y) * resolution_ + rand()%100/101.0 * resolution_ * 0.5;
      return distance(map_x, map_y, end_x, end_y) * resolution_;
    }
    else if (std::abs(theta_normalized) <= (0.25 * M_PI) || std::abs(theta_normalized) >= (0.75 * M_PI))
    {
      k = std::abs(std::tan(theta_normalized));
      major_step = std::abs(theta_normalized) <= (0.25 * M_PI) ? 1 : -1;
      minor_step = theta_normalized >= 0 ? 1 : -1;
      while (true){
        end_x += major_step;
        e += k;
        if (e >= 0){
          end_y += minor_step;
          e -= 1;
        }
        if (outOfMap(end_x, end_y) || getGridCell(end_x, end_y) == OCCUPIED_SPACE || getGridCell(end_x, end_y) == UNEXPLORED_SPACE){
          return distance(map_x, map_y, end_x, end_y) * resolution_ + rand()%100/101.0 * resolution_ * 0.5;
        }
      }
    }else{
      k = std::abs(tan(0.5 * M_PI - theta_normalized));
      major_step = theta_normalized > 0 ? 1 : -1;
      minor_step = std::abs(theta_normalized) <= (0.5 * M_PI) ? 1 : -1;
      while (true){
        end_y += major_step;
        e += k;
        if (e >= 0){
          end_x += minor_step;
          e -= 1;
        }
        if (outOfMap(end_x, end_y) || getGridCell(end_x, end_y) == OCCUPIED_SPACE|| getGridCell(end_x, end_y) == UNEXPLORED_SPACE){
          return distance(map_x, map_y, end_x, end_y) * resolution_ + rand()%100/101.0 * resolution_ * 0.5;
        }
      }
    }
  }

  bool EnvironmentMap::outOfMap(int& map_x, int& map_y)
  {
    bool x_exceed = (map_x >= size_x_) || (map_x <= 0);
    map_x = x_exceed? std::max(std::min(map_x, (int)size_x_), 0) : map_x;
    bool y_exceed = (map_y >= size_y_) || (map_y <= 0);
    map_y = y_exceed? std::max(std::min(map_y, (int)size_y_), 0) : map_y;
    return x_exceed || y_exceed;
  }

  bool EnvironmentMap::isOccupied(double world_x, double world_y)
  {
    int map_x, map_y;
    worldToMap(world_x, world_y, map_x, map_y);
    return isOccupied(map_x, map_y);
  }

  bool EnvironmentMap::isOccupied(int map_x, int map_y)
  {
    return (getGridCell(map_x, map_y) == OCCUPIED_SPACE ? true : false);
  }

  int EnvironmentMap::insertObstacle(std::string& obstacle_name,
                                     std::vector<geometry_msgs::Polygon>& polygon_list,
                                     float x,
                                     float y,
                                     float yaw)
  {
    deleteObstacle(obstacle_name);
    footprint_list obstacle_footprints;
    for (int i=0; i<polygon_list.size(); i++)
    {
      footprint f;
      for (int j=0; j<polygon_list.at(i).points.size(); j++)
      {
        Point p;
        p.x = polygon_list.at(i).points.at(j).x;
        p.y = polygon_list.at(i).points.at(j).y;
        f.push_back(p);
      }
      obstacle_footprints.push_back(f);
    }
    footprints_map_[obstacle_name] = obstacle_footprints;

    footprint obstacle_points;
    transformFootprint(obstacle_footprints, obstacle_points, x, y, yaw);
    obstacle_points_map_[obstacle_name] = obstacle_points;
    exist_obstacle_ = true;
    return 0;
  }

  int EnvironmentMap::transformObstacle(std::string& obstacle_name,
                        float x,
                        float y,
                        float yaw)
  {
    footprints_map::iterator iter = footprints_map_.find(obstacle_name);
    if (iter == footprints_map_.end())
    {
      throw std::runtime_error("No specified obstacle");
    }

    if (obstacle_points_map_.find(obstacle_name) != obstacle_points_map_.end())
    {
      obstacle_points_map_.erase(obstacle_name);
    }
    footprint obs_points;
    transformFootprint(iter->second, obs_points, x, y, yaw);
    obstacle_points_map_[obstacle_name] = obs_points;
    return 0;
  }

  int EnvironmentMap::deleteObstacle(std::string& obstacle_name)
  {
    if (footprints_map_.find(obstacle_name) != footprints_map_.end())
    {
      footprints_map_.erase(obstacle_name);
    }

    if (obstacle_points_map_.find(obstacle_name) != obstacle_points_map_.end())
    {
      obstacle_points_map_.erase(obstacle_name);
    }

    if(obstacle_points_map_.size() == 0)
    {
      exist_obstacle_ = false;
    }
    return 0;
  }

  int EnvironmentMap::updateObstacleData()
  {
    int mx, my, index;
    auto length = size_x_ * size_y_;
    memset(obstacle_data_, 0, length);
    for (auto iter = obstacle_points_map_.begin();
              iter!=obstacle_points_map_.end();
              iter++)
    {
      for (int i=0; i<(iter->second).size(); i++)
      {
        worldToMap((iter->second)[i].x, (iter->second)[i].y, mx, my);
        index = my * size_x_ + mx;
        if (index >=0 && index <= length)
        {
          obstacle_data_[index] = OCCUPIED_SPACE;
        }
      }
    }
    return 0;
  }

  int EnvironmentMap::getObstacleRange(footprint& footprint,
                                       float& min_x,
                                       float& max_x,
                                       float& min_y,
                                       float& max_y)
  {
    if (footprint.size() > 0)
    {
      min_x = max_x = footprint.at(0).x;
      min_y = max_y = footprint.at(0).y;
      for (int i=1; i<footprint.size(); i++)
      {
        min_x = footprint.at(i).x < min_x ? footprint.at(i).x : min_x;
        max_x = footprint.at(i).x > max_x ? footprint.at(i).x : max_x;
        min_y = footprint.at(i).y < min_y ? footprint.at(i).y : min_y;
        max_y = footprint.at(i).y > max_y ? footprint.at(i).y : max_y;
      }
      min_x = std::max(min_x, (float)origin_at_world_x_);
      max_x = std::min(max_x, (float)(origin_at_world_x_ + size_x_ * resolution_));
      min_y = std::max(min_y, (float)origin_at_world_y_);
      max_y = std::min(max_y, (float)(origin_at_world_y_ + size_y_ * resolution_));
    }
    return 0;
  }

  bool EnvironmentMap::pnpoly(footprint& footprint, float& x, float& y)
  {
    int i,j;
    bool c = false;
    for (i=0, j=footprint.size()-1; i<footprint.size(); j = i++)
    {
      if ( ( (footprint.at(i).y > y) != (footprint.at(j).y > y) ) &&
           (x < (footprint.at(j).x-footprint.at(i).x) * (y-footprint.at(i).y) / (footprint.at(j).y - footprint.at(i).y) + footprint.at(i).x) )
      {
        c = !c;
      }
    }
    return c;
  }

  int EnvironmentMap::getObstaclePoints(footprint& f, footprint& obs_points)
  {
    float min_x, max_x, min_y, max_y;
    getObstacleRange(f, min_x, max_x, min_y, max_y);
    for (float x = (std::ceil(min_x / resolution_) * resolution_); x <= max_x; x = x + resolution_/2)
    {
      for (float y = (std::ceil(min_y / resolution_) * resolution_); y <= max_y; y = y + resolution_/2)
      {
        if (pnpoly(f, x, y))
        {
          Point p; p.x = x; p.y = y;
          obs_points.push_back(p);
        }
      }
    }
    return 0;
  }

  int EnvironmentMap::transformFootprint(footprint_list& source,
                                         footprint& target,
                                         float x,
                                         float y,
                                         float yaw)
  {
    float cos_th = cos(yaw);
    float sin_th = sin(yaw);
    footprint f;
    for (int i=0; i<source.size(); i++)
    {
      f.clear();
      for (int j=0; j<source[i].size(); j++)
      {
        Point p;
        p.x =  x + (source[i][j].x * cos_th - source[i][j].y * sin_th);
        p.y =  y + (source[i][j].x * sin_th + source[i][j].y * cos_th);
        f.push_back(p);
      }
      getObstaclePoints(f, target);
    }
    return 0;
  }
}   // END of namespace ak
