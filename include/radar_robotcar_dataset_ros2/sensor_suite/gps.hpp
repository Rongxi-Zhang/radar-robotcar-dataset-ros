/*==================================================================
  Software License Agreement (GNU General Public License v3.0)
  http://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2022, Rongxi Zhang
  All rights reserved.

  E-mail: rongxizhangcar@gmail.com
 ===================================================================*/

#ifndef GPS_HPP  // Note: Only use the data from the ins.cv
#define GPS_HPP

// ROS2
#include <tf2_ros/transform_broadcaster.h>

#include <gps_msgs/msg/gps_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
// mine
#include "radar_robotcar_dataset_ros2/config.hpp"

// declare
class extrinsic;
class timestamp;

class gps {
 private:
  struct InsData {
    string timestamp, ins_status;
    double latitude, longitude, altitude;
    double northing, easting, down;
    string utm_zone;
    double velocity_north, velocity_east, velocity_down;
    float roll, pitch, yaw;
  };

 public:
  // basic parameters
  string sdk_path;
  string dataset_path;
  extrinsic e;
  timestamp t;  // don't initial
  vector<InsData> ins_v;
  double theta = 0.0f;

  // rclcpp
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_publisher;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_publisher;
  // data caches
  // mutex cache_mutex;
  const int cache_size = 1000;
  deque<sensor_msgs::msg::NavSatFix> gps_cache;
  deque<nav_msgs::msg::Odometry> odom_cache;

 public:
  void inital(string sp, string dp) {
    sdk_path = sp;
    dataset_path = dp;
    e.inital("ins", sp);
    data_read();
    cout << "The GPS/INS initialization completed." << endl;
  }

  void data_read() {
    ins_v.clear();
    t.timestamplist.clear();
    string path = dataset_path + "/gps/ins.csv";
    ifstream file(path, ios::in);
    string strline;
    size_t index;

    getline(file, strline);  // jump the first line
    while (getline(file, strline)) {
      InsData insline;

      index = strline.find_first_of(',');
      insline.timestamp = strline.substr(strline.find_first_not_of('0'), index);
      t.timestamplist.push_back(insline.timestamp);  // push to the t.list
      strline = strline.substr(++index);

      index = strline.find_first_of(',');
      insline.ins_status = strline.substr(0, index);
      strline = strline.substr(++index);

      index = strline.find_first_of(',');
      insline.latitude = stod(strline.substr(0, index));
      strline = strline.substr(++index);

      index = strline.find_first_of(',');
      insline.longitude = stod(strline.substr(0, index));
      strline = strline.substr(++index);

      index = strline.find_first_of(',');
      insline.altitude = stod(strline.substr(0, index));
      strline = strline.substr(++index);

      index = strline.find_first_of(',');
      insline.northing = stod(strline.substr(0, index));
      strline = strline.substr(++index);

      index = strline.find_first_of(',');
      insline.easting = stod(strline.substr(0, index));
      strline = strline.substr(++index);

      index = strline.find_first_of(',');
      insline.down = stod(strline.substr(0, index));
      strline = strline.substr(++index);

      index = strline.find_first_of(',');
      insline.utm_zone = strline.substr(0, index);
      strline = strline.substr(++index);

      index = strline.find_first_of(',');
      insline.velocity_north = stod(strline.substr(0, index));
      strline = strline.substr(++index);

      index = strline.find_first_of(',');
      insline.velocity_east = stod(strline.substr(0, index));
      strline = strline.substr(++index);

      index = strline.find_first_of(',');
      insline.velocity_down = stod(strline.substr(0, index));
      strline = strline.substr(++index);

      index = strline.find_first_of(',');
      insline.roll = stod(strline.substr(0, index));
      strline = strline.substr(++index);

      index = strline.find_first_of(',');
      insline.pitch = stod(strline.substr(0, index));
      strline = strline.substr(++index);

      insline.yaw = stod(strline);

      ins_v.push_back(insline);
    }
    // adjust the euler of INS
    theta = 90.0f;
  }

  void create_publisher(rclcpp::Node::SharedPtr n) {
    node = n;
    gps_publisher = node->create_publisher<sensor_msgs::msg::NavSatFix>(
        "/robotcar/ins/gps", 20);
    odom_publisher = node->create_publisher<nav_msgs::msg::Odometry>(
        "/robotcar/ins/odom", 20);
    tf_publisher = std::make_unique<tf2_ros::TransformBroadcaster>(*node);
    e.tf_publisher =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(*node);
    cout << "Created publishers for the GPS/INS." << endl;
  }

  void index_callback(int index, bool flag, bool is_radar) {
    static Eigen::Vector3f start_t;
    static Eigen::Matrix3f start_R, adjust_R;
    // compute
    string time_stamp = t.timestamplist[index];

    double northing, easting;
    northing = ins_v[index].northing;
    easting = ins_v[index].easting;
    tf2::Quaternion q;
    q.setEuler(ins_v[index].roll, ins_v[index].pitch, ins_v[index].yaw);

    if (!index) {
      start_t(0) = easting;
      start_t(1) = northing;
      start_t(2) = ins_v[index].altitude;

      Eigen::Quaternionf start_q;
      start_q.x() = q.x();
      start_q.y() = q.y();
      start_q.z() = q.z();
      start_q.w() = q.w();
      start_R = start_q.toRotationMatrix();

      Eigen::Quaternionf adjust_q;
      adjust_q.x() = 1.0;
      adjust_q.y() = 0.0;
      adjust_q.z() = 0.0;
      adjust_q.w() = 0.0;
      adjust_R = adjust_q.toRotationMatrix();
    }
    Eigen::Quaternionf curr_q;
    Eigen::Matrix3f curr_R;
    curr_q.x() = q.x();
    curr_q.y() = q.y();
    curr_q.z() = q.z();
    curr_q.w() = q.w();
    curr_R = curr_q.toRotationMatrix();
    if (!is_radar) {  // adjust
      Eigen::Quaternionf adjust_q;
      adjust_q.x() = 0.0;
      adjust_q.y() = 0.0;
      adjust_q.z() = 1.0;
      adjust_q.w() = 0.0;
      curr_R *= adjust_q.toRotationMatrix();
    }
    curr_R = start_R * curr_R.inverse() * adjust_R;  // adjust
    Eigen::Quaternionf new_q(curr_R);

    // adjust
    double x = easting - start_t(0);
    double y = northing - start_t(1);
    double x_ = -(x * cos(theta) + y * sin(theta));
    double y_ = x * sin(theta) - y * cos(theta);

    // NavSatFix
    sensor_msgs::msg::NavSatFix navsat;
    navsat.header.frame_id = departure_frame;
    navsat.header.stamp = t.Timestamp2Rostime(time_stamp);
    navsat.status.status = 0;
    navsat.status.service = 1;
    navsat.latitude = ins_v[index].latitude;
    navsat.longitude = ins_v[index].longitude;
    navsat.altitude = ins_v[index].altitude;

    // Odometry
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = navsat.header.stamp;
    odom.header.frame_id = departure_frame;
    odom.child_frame_id = gps_frame;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    // odom.pose.pose.position.z = ins_v[index].altitude - start_t(2);
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = new_q.x();
    odom.pose.pose.orientation.y = new_q.y();
    odom.pose.pose.orientation.z = new_q.z();
    odom.pose.pose.orientation.w = new_q.w();
    odom.twist.twist.linear.x = -ins_v[index].velocity_east;
    odom.twist.twist.linear.y = -ins_v[index].velocity_north;
    // odom.twist.twist.linear.z = ins_v[index].velocity_down;
    odom.twist.twist.linear.z = 0.0;

    // TF2 Brodcaster for the INS odometry
    geometry_msgs::msg::TransformStamped trans;
    trans.header = odom.header;
    trans.child_frame_id = gps_frame;
    trans.transform.translation.x = odom.pose.pose.position.x;
    trans.transform.translation.y = odom.pose.pose.position.y;
    // trans.transform.translation.z = odom.pose.pose.position.z;
    trans.transform.translation.z = 0.0;
    trans.transform.rotation.x = new_q.x();
    trans.transform.rotation.y = new_q.y();
    trans.transform.rotation.z = new_q.z();
    trans.transform.rotation.w = new_q.w();

    if (flag) {
      gps_publisher->publish(navsat);
      odom_publisher->publish(odom);
      tf_publisher->sendTransform(trans);
      e.make_inverse_transforms(navsat.header.stamp, gps_frame, stereo_frame);

    } else {
      // save bag
    }
  }

 public:
  explicit gps() {}
  explicit gps(string sp, string dp)
      : sdk_path(sp), dataset_path(dp), e("ins", sp) {
    cout << "The GPS/INS initialization completed." << endl;
  }

};  // class gps

#endif  // GPS_HPP