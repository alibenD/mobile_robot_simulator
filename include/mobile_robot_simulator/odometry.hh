#ifndef __ODOMETRY_HH__
#define __ODOMETRY_HH__
/**-----------------------------------------------
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: odometry.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-10-29 10:00:30
  * @last_modified_date: 2019-06-10 10:33:15
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

// Header include
#include <iostream>
#define M_PI 3.14159265358979323846  /* pi */

// Declaration
namespace ak
{
  class Odometry;
  class Pose;
  class Velocity;
  struct Point3D;
  struct Quternion;
  using Vector3D = Point3D;
  using Linear = Point3D;
  using Angular = Point3D;
  using RPY = Point3D;

  std::ostream& operator<<(std::ostream& os, const Point3D& point3d);
  std::ostream& operator<<(std::ostream& os, const Quternion& quaternion);
  std::ostream& operator<<(std::ostream& os, const Pose& pose);
  std::ostream& operator<<(std::ostream& os, const Velocity& velocity);
  std::ostream& operator<<(std::ostream& os, const Odometry& odom);

  struct Point3D
  {
    public:
      Point3D() : x_(0.0), y_(0.0), z_(0.0) {};
      explicit Point3D(double x)
        : Point3D(x, 0.0)
      {}
      explicit Point3D(double x, double y)
        : Point3D(x, y, 0.0)
      {}
      explicit Point3D(double x, double y, double z)
        : x_(x), y_(y), z_(z)
      {}

      ~Point3D() = default;
      Point3D& operator+(const Point3D& point);
      Point3D& operator+=(const Point3D& point);

      double x_;
      double y_;
      double z_;
  };

  struct Quternion
  {
    public:
      Quternion() : x_(0.0), y_(0.0), z_(0.0), w_(1.0) {};
      ~Quternion() = default;
      double x_;
      double y_;
      double z_;
      double w_;
  };

  class Velocity
  {
    public:
      Velocity() : linear_(), angular_() {};
      ~Velocity() = default;
      void reset();
      Linear linear_;
      Angular angular_;
  };


  class Pose
  {
    public:
      Pose()
        : Pose(Point3D(), RPY()) {};
      explicit Pose(double px, double py, double pz,
                    double ox, double oy, double oz)
        : Pose(Point3D(px, py, pz), RPY(ox, oy, oz)) {};
      explicit Pose(const Point3D& position, const RPY& orientation)
        : position_(position), orientation_(orientation) {};
      ~Pose() = default;

      Pose& operator+(const Point3D& position_increment);
      Pose& operator+=(const Point3D& position_increment);
      Pose& operator+(const Pose& pose_increment);
      Pose& operator+=(const Pose& pose_increment);

      Point3D position_;
      //Quternion orientation_;
      RPY orientation_;
  };

  /**
   * @brief Odometry simulator
   */
  class Odometry
  {
    public:
      Odometry() : pose_(0, 0, 0, 0, 0, 0), velocity_(), timestamp_(0) {};
      ~Odometry() = default;

      /**
       * @brief Given a new pose to update position info
       */
      void updatePose(const Pose& new_pose);

      /**
       * @brief Given a new velocity to update velocity info
       */
      void updateVelocity(const Velocity& new_velocity);

      /**
       * @brief Get the current pose
       */
      const Pose& getPose() const;

      /**
       * @brief Get the current velocity
       */
      const Velocity& getVelocity() const;

      //void setPose(const Pose& pose_new);
      //void setVelocity(const Velocity& velocity_new);

      // Overload functions
      friend std::ostream& operator<<(std::ostream& os, const Odometry& odom);
      Odometry& operator+(const Point3D& position_increment);
      Odometry& operator+=(const Point3D& position_increment);
      Odometry& operator+(const Pose& pose_increment);
      Odometry& operator+=(const Pose& pose_increment);

    private:
      Pose pose_;
      Velocity velocity_;
      int timestamp_;
  };
}   // END of namespace ak
#endif // __ODOMETRY_HH__
