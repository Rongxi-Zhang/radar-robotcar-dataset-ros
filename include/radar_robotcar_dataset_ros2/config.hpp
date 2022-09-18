/*==================================================================
  Software License Agreement (GNU General Public License v3.0)
  http://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2022, Rongxi Zhang
  All rights reserved.

  E-mail: rongxizhangcar@gmail.com
 ===================================================================*/

#ifndef CONFIG_HPP_
#define CONFIG_HPP_

#ifdef __cplusplus
extern "C" {
#endif
// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
// https://gcc.gnu.org/wiki/Visibility
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define COMPOSITION_EXPORT __attribute__((dllexport))
#define COMPOSITION_IMPORT __attribute__((dllimport))
#else
#define COMPOSITION_EXPORT __declspec(dllexport)
#define COMPOSITION_IMPORT __declspec(dllimport)
#endif
#ifdef COMPOSITION_BUILDING_DLL
#define COMPOSITION_PUBLIC COMPOSITION_EXPORT
#else
#define COMPOSITION_PUBLIC COMPOSITION_IMPORT
#endif
#define COMPOSITION_PUBLIC_TYPE COMPOSITION_PUBLIC
#define COMPOSITION_LOCAL
#else
#define COMPOSITION_EXPORT __attribute__((visibility("default")))
#define COMPOSITION_IMPORT
#if __GNUC__ >= 4
#define COMPOSITION_PUBLIC __attribute__((visibility("default")))
#define COMPOSITION_LOCAL __attribute__((visibility("hidden")))
#else
#define COMPOSITION_PUBLIC
#define COMPOSITION_LOCAL
#endif
#define COMPOSITION_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

// std
#include <string.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <thread>
// ROS2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/time.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
// Eigen
#include <Eigen/Dense>
// QT
#include <QDebug>
#include <QMutex>
#include <QThread>

// namespace
using namespace std;

// Names of all frames
const string departure_frame = "world";
const string mono_left_frame = "Grasshopper2_left",
             mono_right_frame = "Grasshopper2_right",
             mono_rear_frame = "Grasshopper2_rear";
const string stereo_frame = "Bumblebee",
             stereo_centre_frame = "Bumblebee_centre",
             stereo_right_frame = "Bumblebee_right";
const string radar_frame = "CTS350-X", lidar_left_frame = "HDL-32E_left",
             lidar_right_frame = "HDL-32E_right",
             lidar_front3d_frame = "LD-MRS";
const string gps_frame = "SPAN-CPT";
const string lidar_front_frame = "LMS-151_front",
             lidar_rear_frame = "LMS-151_rear";

// enum types
enum sensor_type {
  sensor_stereo,
  sensor_mono_left,
  sensor_mono_right,
  sensor_mono_rear,
  sensor_radar,
  sensor_lidar_left,
  sensor_lidar_right,
  sensor_lidar_front,
  sensor_lidar_rear,
  sensor_gps_ins,
  sensor_lidar_old
};

class extrinsic {
 public:
  string name;
  string path;  // SDK path
  geometry_msgs::msg::TransformStamped trans, trans_inverse;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_publisher;

 public:
  void make_transforms(const rclcpp::Time& timestamp, const string& frame,
                       const string& child) {
    trans.header.stamp = timestamp;
    trans.header.frame_id = frame;
    trans.child_frame_id = child;

    tf_publisher->sendTransform(trans);
  }

  void make_inverse_transforms(const rclcpp::Time& timestamp,
                               const string& frame, const string& child) {
    trans_inverse.header.stamp = timestamp;
    trans_inverse.header.frame_id = frame;
    trans_inverse.child_frame_id = child;

    tf_publisher->sendTransform(trans_inverse);
  }

  void inital(string n, string p) {
    name = n;
    path = p;
    size_t index;
    ifstream file;
    string strline;
    file.open(path + "/extrinsics/" + name + ".txt", ios::in);
    if (!file.is_open()) {
      cout << "No such a file named " + name +
                  ".txt. pLease check your SDK path."
           << endl;
      return;
    } else
      cout << "Reading the extrinsic parameter of the " + name + "..." << endl;

    getline(file, strline);
    file.close();

    // translation
    index = strline.find_first_of(' ');
    trans.transform.translation.x = stod(strline.substr(0, index));
    strline = strline.substr(++index);

    index = strline.find_first_of(' ');
    trans.transform.translation.y = stod(strline.substr(0, index));
    strline = strline.substr(++index);

    index = strline.find_first_of(' ');
    trans.transform.translation.z = stod(strline.substr(0, index));
    strline = strline.substr(++index);
    // rotation
    double roll, pitch, yaw;

    index = strline.find_first_of(' ');
    roll = stod(strline.substr(0, index));
    strline = strline.substr(++index);

    index = strline.find_first_of(' ');
    pitch = stod(strline.substr(0, index));
    strline = strline.substr(++index);

    yaw = stod(strline);

    tf2::Quaternion q;
    q.setEuler(roll, pitch, yaw);

    trans.transform.rotation.x = q.x();
    trans.transform.rotation.y = q.y();
    trans.transform.rotation.z = q.z();
    trans.transform.rotation.w = q.w();

    // inverse(for tf2 tree and the monos)
    Eigen::Vector3d et;
    et(0) = trans.transform.translation.x;
    et(1) = trans.transform.translation.y;
    et(2) = trans.transform.translation.z;

    Eigen::Quaterniond eq;
    eq.x() = q.x();
    eq.y() = q.y();
    eq.z() = q.z();
    eq.w() = q.w();
    Eigen::Matrix3d eR = eq.toRotationMatrix();
    eR = eR.inverse().eval();
    // new translation
    et = -eR * et;
    trans_inverse.transform.translation.x = et(0);
    trans_inverse.transform.translation.y = et(1);
    trans_inverse.transform.translation.z = et(2);
    // new quaternion
    Eigen::Quaterniond eq2(eR);
    trans_inverse.transform.rotation.x = eq2.x();
    trans_inverse.transform.rotation.y = eq2.y();
    trans_inverse.transform.rotation.z = eq2.z();
    trans_inverse.transform.rotation.w = eq2.w();
  }

 public:
  explicit extrinsic() {}
  explicit extrinsic(string n, string p) : name(n), path(p) { inital(n, p); }
};  // class extrinsic

class timestamp {
 public:
  string name;
  string path;  // dataset path
  vector<string> timestamplist;

 public:
  chrono::milliseconds make_duration(int index) {
    double nanosec =
        stod(timestamplist[index + 1]) - stod(timestamplist[index]);
    return chrono::milliseconds(static_cast<int>(nanosec / 1e6));
  }

  double make_frequency(int index) {
    if (index < size() - 1) {
      double nanosec =
          stod(timestamplist[index + 1]) - stod(timestamplist[index]);
      return 1e6 / nanosec;
    } else
      return 4.0;
  }

  void inital(string n, string p) {
    name = n;
    path = p;
    timestamplist.clear();

    ifstream file;
    string strline, subline;
    file.open(path + "/" + name + ".timestamps", ios::in);

    if (file.is_open()) {
      cout << "Reading the timestamps of the " + name + ".timestamps..."
           << endl;
      while (getline(file, strline)) {
        subline = strline.substr(0, strline.find_last_of(' '));
        timestamplist.push_back(subline);
      }
      file.close();
    } else
      cout << "No such a file named " + name +
                  ".timestamps. Please check your dataset path ."
           << endl;
  }
  // basic info
  int size() { return this->timestamplist.size(); }
  string start_time() { return this->timestamplist.front(); }
  string end_time() { return this->timestamplist.back(); }
  double duration() {
    return (stod(this->end_time()) - stod(this->start_time())) / 1e6;
  }

  rclcpp::Time Timestamp2Rostime(string timestamp) {
    int32_t sec = stoi(timestamp.substr(0, 10));
    uint32_t nsec = stoi(timestamp.substr(10));
    return rclcpp::Time(sec, nsec);
  }

 public:
  explicit timestamp() {}
  explicit timestamp(string n, string p) : name(n), path(p) { inital(n, p); }

};  // class timestamp


#endif