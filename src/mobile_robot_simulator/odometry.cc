/**
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: odometry.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-10-29 10:51:25
  * @last_modified_date: 2018-10-29 19:11:49
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <mobile_mobile_mobile_robot_simulator/odometry.hh>

//CODE
namespace ak
{
  // Overload
  std::ostream& operator<<(std::ostream& os, const Odometry& odom)
  {
    os << odom.getVelocity() << std::endl
       << odom.getPose()
       << std::endl;
    return os;
  }

  std::ostream& operator<<(std::ostream& os, const Point3D& point3d)
  {
    os << "[VNLOG] Point3D: [x, y, z] = ["
       << point3d.x_ << ", "
       << point3d.y_ << ", "
       << point3d.z_ << "]";
    return os;
  }

  std::ostream& operator<<(std::ostream& os, const Quternion& quaternion)
  {
    os << "[VNLOG] Quaternion: [x, y, z, w] = ["
       << quaternion.x_ << ", "
       << quaternion.y_ << ", "
       << quaternion.z_ << ", "
       << quaternion.w_ << "]";
  }

  std::ostream& operator<<(std::ostream& os, const Pose& pose)
  {
    os << "[VNLOG] Position: \t\t[x, y, z] = ["
       << pose.position_.x_ << ", "
       << pose.position_.y_ << ", "
       << pose.position_.z_ << "]"
       << std::endl;

    os << "        Orientation(RPY): [x, y, z] = ["
       << pose.orientation_.x_ << ", "
       << pose.orientation_.y_ << ", "
       << pose.orientation_.z_ << "]";
    return os;
  }

  std::ostream& operator<<(std::ostream& os, const Velocity& velocity)
  {
    os << "[VNLOG] Velocity Linear: \t[x, y, z] = ["
       << velocity.linear_.x_ << ", "
       << velocity.linear_.y_ << ", "
       << velocity.linear_.z_ << "]"
       << std::endl;

    os << "        Velocity Angular: \t[x, y, z] = ["
       << velocity.angular_.x_ << ", "
       << velocity.angular_.y_ << ", "
       << velocity.angular_.z_ << "]";
    return os;
  }


  void Odometry::updatePose(const Pose& new_pose)
  {
    pose_ = new_pose;
  }

  void Odometry::updateVelocity(const Velocity& new_velocity)
  {
    velocity_ = new_velocity;
  }

  const Pose& Odometry::getPose() const
  {
    return pose_;
  }

  const Velocity& Odometry::getVelocity() const
  {
    return velocity_;
  }

//==================Odometry=================
  Odometry& Odometry::operator+(const Point3D& position_increment)
  {
    *this += position_increment;
    return *this;
  }

  Odometry& Odometry::operator+=(const Point3D& position_increment)
  {
    this->pose_ += position_increment;
    return *this;
  }

  Odometry& Odometry::operator+(const Pose& pose_increment)
  {
    *this += pose_increment;
    return *this;
  }

  Odometry& Odometry::operator+=(const Pose& pose_increment)
  {
    this->pose_ += pose_increment;
    return *this;
  }

//================Pose=================
  Pose& Pose::operator+(const Point3D& position_increment)
  {
    *this += position_increment;
    return *this;
  }

  Pose& Pose::operator+=(const Point3D& position_increment)
  {
    this->position_ += position_increment;
    return *this;
  }

  Pose& Pose::operator+(const Pose& pose_increment)
  {
    *this += pose_increment;
    return *this;
  }

  Pose& Pose::operator+=(const Pose& pose_increment)
  {
    this->position_ += pose_increment.position_;
    this->orientation_ += pose_increment.orientation_;
    return *this;
  }

//================Point3D=============
  Point3D& Point3D::operator+(const Point3D& point)
  {
    *this += point;
    return *this;
  }

  Point3D& Point3D::operator+=(const Point3D& point)
  {
    this->x_ += point.x_;
    this->y_ += point.y_;
    this->z_ += point.z_;
    return *this;
  }

//================Velocity============
  void Velocity::reset()
  {
    linear_.x_ = 0;
    linear_.y_ = 0;
    linear_.z_ = 0;
    angular_.x_ = 0;
    angular_.y_ = 0;
    angular_.z_ = 0;
  }
}   // END of namespace ak
