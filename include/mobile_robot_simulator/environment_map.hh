#ifndef __ENVIRONMENT_MAP_HH__
#define __ENVIRONMENT_MAP_HH__
/**-----------------------------------------------
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: environment_map.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-01-02 13:41:20
  * @last_modified_date: 2019-06-10 10:45:59
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

// Header include
#include <vector>
#include <random>
#include <ctime>
#include <algorithm>
#include <unordered_map>

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Polygon.h>

#define FREE_SPACE 0
#define OCCUPIED_SPACE 100
#define UNEXPLORED_SPACE -1
#define ERROR_SPACE -2

// Declaration
namespace ak
{
  struct Point
  {
      float x;
      float y;
  };
  
  using footprint = std::vector<Point>;
  using footprint_list = std::vector<footprint>;
  using footprints_map = std::unordered_map<std::string, footprint_list>;
  using obstacle_points_map = std::unordered_map<std::string, footprint>;

  class EnvironmentMap;

  class EnvironmentMap
  {
    public:
      EnvironmentMap() = default;
      ~EnvironmentMap() = default;

      int initMap(const nav_msgs::OccupancyGrid::ConstPtr& grid_map);
      int mapToWorld(int map_x, int map_y,
                     double& world_x, double& world_y);

      int worldToMap(double world_x, double world_y,
                     int& map_x, int& map_y);

      bool isOccupied(double world_x, double world_y);
      bool isOccupied(int map_x, int map_y);

      int setGridCell(double world_x, double world_y, unsigned char value);
      int setGridCell(int map_x, int map_y, unsigned char value);
      unsigned char getGridCell(double world_x, double world_y);
      unsigned char getGridCell(int map_x, int map_y);
      int resetGridCell(double world_x, double world_y);
      int resetGridCell(int map_x, int map_y);
      double calDistance(double world_x, double world_y, double theta);
      double calDistance(int map_x, int map_y, double theta);
      unsigned char* returnMap()
      {
        return meta_environment_data_;
      };
      unsigned int getSizeX()
      {
        return size_x_;
      }
      unsigned int getSizeY()
      {
        return size_y_;
      }

      
      int transformObstacle(std::string& obstacle_name,
                            float x,
                            float y,
                            float yaw);
      int insertObstacle(std::string& obstacle_name,
                         std::vector<geometry_msgs::Polygon>& polygon_list,
                         float x,
                         float y,
                         float yaw);
      int deleteObstacle(std::string& obstacle_name);
      int updateObstacleData();

    protected:
      double normalizeAngle(double theta);
      double distance(int map_x, int map_y, int end_x, int end_y);
      bool outOfMap(int& map_x, int& map_y);
      double noise(double u, double error);


      int getObstacleRange(footprint& footprint,
                           float& min_x,
                           float& max_x,
                           float& min_y,
                           float& max_y);
      bool pnpoly(footprint& footprint, float& x, float& y);
      int getObstaclePoints(footprint& f, footprint& obs_points);
      int transformFootprint(footprint_list& source,
                             footprint& target,
                             float x,
                             float y,
                             float yaw);
    private:
      unsigned char* meta_environment_data_;
      //TODO: Add dynamic obstacle layer
      //unsigned char* meta_obstacle_data_;
      unsigned char* obstacle_data_;
      bool exist_obstacle_;
      footprints_map footprints_map_;
      obstacle_points_map obstacle_points_map_;

      unsigned int size_x_;
      unsigned int size_y_;
      double origin_at_world_x_;
      double origin_at_world_y_;
      double resolution_;
  };
}   // END of namespace ak
#endif // __ENVIRONMENT_MAP_HH__
