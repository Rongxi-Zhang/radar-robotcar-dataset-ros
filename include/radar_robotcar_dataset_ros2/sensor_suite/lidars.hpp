/*==================================================================
  Software License Agreement (GNU General Public License v3.0)
  http://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2022, Rongxi Zhang
  All rights reserved.

  E-mail: rongxizhangcar@gmail.com
 ===================================================================*/

#ifndef LIDARS_HPP
#define LIDARS_HPP

// PCL (mine version: 1.10)
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
// ROS2
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
// mine
#include "radar_robotcar_dataset_ros2/config.hpp"

// declare
class extrinsic;
class timestamp;

class lidar3d {
 public:
  // basic parameters
  string sdk_path;
  string dataset_path;
  extrinsic e_l, e_r;
  timestamp t_l, t_r;

  // rclcpp
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr left_publisher,
      right_publisher;

  // data caches
  // mutex cache_mutex;
  const int cache_size = 100;
  deque<sensor_msgs::msg::PointCloud2> left_cache, right_cache;

 public:
  void inital(string sp, string dp) {
    sdk_path = sp;
    dataset_path = dp;
    e_l.inital("velodyne_left", sp);
    t_l.inital("velodyne_left", dp);
    e_r.inital("velodyne_right", sp);
    t_r.inital("velodyne_right", dp);
    cout << "The 3D LiDARs initialization completed." << endl;
  }

  void cache_read(int index) {
    left_cache.clear();
    right_cache.clear();
    for (int i = index; i < cache_size; ++i) {
      index_callback(i, false, sensor_lidar_left);
      index_callback(i, false, sensor_lidar_right);
    }
    cout << "3D LiDAR data loading completed." << endl;
  }

  void create_publisher(rclcpp::Node::SharedPtr n) {
    node = n;
    left_publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/robotcar/lidar/left", 1);
    right_publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/robotcar/lidar/right", 1);
    e_l.tf_publisher =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(*node);
    e_r.tf_publisher =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(*node);
    cout << "Created publishers for the 3D LiDARs." << endl;
  }

  void index_callback(int index, bool flag, sensor_type type) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_ptr(
        new pcl::PointCloud<pcl::PointXYZI>());
    pcl_ptr->clear();
    string path, time_stamp, filename;
    size_t start, end, cnt = 0;

    switch (type) {
      case sensor_lidar_left: {
        time_stamp = t_l.timestamplist[index];
        filename = time_stamp + ".bin";
        path = dataset_path + "/velodyne_left/" + filename;
        // calc its size first
        ifstream filesize(path, ios::in | ios::binary);
        start = filesize.tellg();
        filesize.seekg(0, ios::end);
        end = filesize.tellg();
        filesize.close();  // closed
        cnt = (end - start) / (4 * sizeof(float));

        // then convert
        ifstream input(path, ios::in | ios::binary);
        // N*4, not 4*N!!
        for (size_t i = 0; i < cnt; ++i) {
          if (input.good() && !input.eof()) {
            pcl::PointXYZI pcl;
            input.read((char *)&pcl.x, sizeof(float));
            pcl_ptr->push_back(pcl);
          }
        }
        for (size_t i = 0; i < cnt; ++i)
          if (input.good() && !input.eof())
            input.read((char *)&(pcl_ptr->at(i).y), sizeof(float));
        for (size_t i = 0; i < cnt; ++i)
          if (input.good() && !input.eof())
            input.read((char *)&(pcl_ptr->at(i).z), sizeof(float));
        for (size_t i = 0; i < cnt; ++i)
          if (input.good() && !input.eof())
            input.read((char *)&(pcl_ptr->at(i).intensity), sizeof(float));
        input.close();  // closed
                        // to ROS2 pcl2
        sensor_msgs::msg::PointCloud2::SharedPtr left_msg_ptr(
            new sensor_msgs::msg::PointCloud2);
        pcl::toROSMsg(*pcl_ptr, *left_msg_ptr);
        left_msg_ptr->header.stamp = t_l.Timestamp2Rostime(time_stamp);
        left_msg_ptr->header.frame_id = lidar_left_frame;

        if (flag) {
          left_publisher->publish(*left_msg_ptr);
          e_l.make_transforms(left_msg_ptr->header.stamp, stereo_frame,
                              lidar_left_frame);
        } else
          left_cache.push_back(*left_msg_ptr);

        break;
      }
      case sensor_lidar_right: {
        time_stamp = t_r.timestamplist[index];
        filename = time_stamp + ".bin";
        path = dataset_path + "/velodyne_right/" + filename;
        // calc its size first
        ifstream filesize(path, ios::in | ios::binary);
        start = filesize.tellg();
        filesize.seekg(0, ios::end);
        end = filesize.tellg();
        filesize.close();  // closed
        cnt = (end - start) / (4 * sizeof(float));

        // then convert
        ifstream input(path, ios::in | ios::binary);
        // N*4, not 4*N!!
        for (size_t i = 0; i < cnt; ++i) {
          if (input.good() && !input.eof()) {
            pcl::PointXYZI pcl;
            input.read((char *)&pcl.x, sizeof(float));
            pcl_ptr->push_back(pcl);
          }
        }
        for (size_t i = 0; i < cnt; ++i)
          if (input.good() && !input.eof())
            input.read((char *)&(pcl_ptr->at(i).y), sizeof(float));
        for (size_t i = 0; i < cnt; ++i)
          if (input.good() && !input.eof())
            input.read((char *)&(pcl_ptr->at(i).z), sizeof(float));
        for (size_t i = 0; i < cnt; ++i)
          if (input.good() && !input.eof())
            input.read((char *)&(pcl_ptr->at(i).intensity), sizeof(float));
        input.close();  // closed

        // to ROS2 pcl2
        sensor_msgs::msg::PointCloud2::SharedPtr right_msg_ptr(
            new sensor_msgs::msg::PointCloud2);
        pcl::toROSMsg(*pcl_ptr, *right_msg_ptr);
        right_msg_ptr->header.stamp = t_r.Timestamp2Rostime(time_stamp);
        right_msg_ptr->header.frame_id = lidar_right_frame;

        if (flag) {
          right_publisher->publish(*right_msg_ptr);
          e_r.make_transforms(right_msg_ptr->header.stamp, stereo_frame,
                              lidar_right_frame);
        } else
          right_cache.push_back(*right_msg_ptr);

        break;
      }
      default:
        break;
    }
  }

  void index_publish(int index) {
    left_publisher->publish(left_cache.at(index));
    right_publisher->publish(right_cache.at(index));
  }

 public:
  explicit lidar3d() {}
  explicit lidar3d(string sp, string dp)
      : sdk_path(sp),
        dataset_path(dp),
        e_l("velodyne_left", sp),
        e_r("velodyne_right", sp),
        t_l("velodyne_left", dp),
        t_r("velodyne_right", dp) {
    cout << "The 3D LiDARs initialization completed." << endl;
  }
};  // class lidar3d

class lidar2d {
 public:
  // basic parameters
  string sdk_path;
  string dataset_path;
  extrinsic e_f, e_r;
  timestamp t_f, t_r;

  // rclcpp
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr front_publisher,
      rear_publisher;

  // data caches
  // mutex cache_mutex;
  const int cache_size = 200;
  deque<sensor_msgs::msg::LaserScan> front_cache, rear_cache;

 public:
  void inital(string sp, string dp) {
    sdk_path = sp;
    dataset_path = dp;
    e_f.inital("lms_front", sp);
    e_r.inital("lms_rear", sp);
    t_f.inital("lms_front", dp);
    t_r.inital("lms_rear", dp);
    cout << "The 2D LiDARs initialization completed." << endl;
  }

  void create_publisher(rclcpp::Node::SharedPtr n) {
    node = n;
    front_publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/robotcar/lidar/front", 1);
    rear_publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/robotcar/lidar/rear", 1);
    e_f.tf_publisher =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(*node);
    e_r.tf_publisher =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(*node);
    cout << "Created publishers for the 2D LiDARs." << endl;
  }

  void index_callback(int index, bool flag, sensor_type type) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_ptr(
        new pcl::PointCloud<pcl::PointXYZI>());
    pcl_ptr->clear();

    string path, time_stamp, filename;
    size_t start, end, cnt = 0;

    switch (type) {
      case sensor_lidar_front: {
        time_stamp = t_f.timestamplist[index];
        filename = time_stamp + ".bin";
        path = dataset_path + "/lms_front/" + filename;
        ifstream filesize(path, ios::in | ios::binary);
        start = filesize.tellg();
        filesize.seekg(0, ios::end);
        end = filesize.tellg();
        filesize.close();  // closed
        cnt = (end - start) / (3 * sizeof(double));

        // then convert
        ifstream input(path, ios::in | ios::binary);
        for (size_t i = 0; i < cnt; ++i) {
          if (input.good() && !input.eof()) {
            pcl::PointXYZI pcl;
            input.read((char *)&pcl.x, sizeof(double));
            pcl_ptr->push_back(pcl);
            input.read((char *)&(pcl_ptr->at(i).y), sizeof(double));
            input.read((char *)&(pcl_ptr->at(i).intensity), sizeof(double));
          }
        }
        input.close();  // closed
        // to ROS2 pcl2
        sensor_msgs::msg::PointCloud2::SharedPtr front_msg_ptr(
            new sensor_msgs::msg::PointCloud2);
        pcl::toROSMsg(*pcl_ptr, *front_msg_ptr);
        front_msg_ptr->header.stamp = t_f.Timestamp2Rostime(time_stamp);
        front_msg_ptr->header.frame_id = lidar_front_frame;

        if (flag) {
          front_publisher->publish(*front_msg_ptr);
          e_f.make_transforms(front_msg_ptr->header.stamp, stereo_frame,
                              lidar_front_frame);
        }
        break;
      }
      case sensor_lidar_rear: {
        time_stamp = t_r.timestamplist[index];
        filename = time_stamp + ".bin";
        path = dataset_path + "/lms_rear/" + filename;
        ifstream filesize(path, ios::in | ios::binary);
        start = filesize.tellg();
        filesize.seekg(0, ios::end);
        end = filesize.tellg();
        filesize.close();  // closed
        cnt = (end - start) / (3 * sizeof(double));

        // then convert
        ifstream input(path, ios::in | ios::binary);
        for (size_t i = 0; i < cnt; ++i) {
          if (input.good() && !input.eof()) {
            pcl::PointXYZI pcl;
            input.read((char *)&pcl.x, sizeof(double));
            pcl_ptr->push_back(pcl);
            input.read((char *)&(pcl_ptr->at(i).y), sizeof(double));
            input.read((char *)&(pcl_ptr->at(i).intensity), sizeof(double));
          }
        }
        input.close();  // closed
        sensor_msgs::msg::PointCloud2::SharedPtr rear_msg_ptr(
            new sensor_msgs::msg::PointCloud2);
        pcl::toROSMsg(*pcl_ptr, *rear_msg_ptr);
        rear_msg_ptr->header.stamp = t_r.Timestamp2Rostime(time_stamp);
        rear_msg_ptr->header.frame_id = lidar_rear_frame;
        if (flag) {
          rear_publisher->publish(*rear_msg_ptr);
          e_r.make_transforms(rear_msg_ptr->header.stamp, stereo_frame,
                              lidar_rear_frame);
        }
        break;
      }
      default:
        break;
    };
  }

 public:
  explicit lidar2d() {}
  explicit lidar2d(string sp, string dp)
      : sdk_path(sp),
        dataset_path(dp),
        e_f("lms_front", sp),
        e_r("lms_rear", sp),
        t_f("lms_front", dp),
        t_r("lms_rear", dp) {
    cout << "The 2D LiDARs initialization completed." << endl;
  }
};  // class lidar2d

class lidarold {
 public:
  // basic parameters
  string sdk_path;
  string dataset_path;
  extrinsic e;
  timestamp t;

  // rclcpp
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;

  // data caches
  // mutex cache_mutex;
  const int cache_size = 200;
  deque<sensor_msgs::msg::PointCloud2> cache;

 public:
  void inital(string sp, string dp) {
    sdk_path = sp;
    dataset_path = dp;
    e.inital("ldmrs", sp);
    t.inital("ldmrs", dp);
    cout << "The 3D LiDAR initialization completed." << endl;
  }

  void create_publisher(rclcpp::Node::SharedPtr n) {
    node = n;
    publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/robotcar/lidar/front3d", 1);
    e.tf_publisher =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(*node);
    cout << "Created publishers for the 3D LiDAR." << endl;
  }

  void index_callback(int index, bool flag) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl_ptr->clear();

    string path, time_stamp, filename;
    size_t start, end, cnt = 0;

    time_stamp = t.timestamplist[index];
    filename = time_stamp + ".bin";
    path = dataset_path + "/ldmrs/" + filename;
    ifstream filesize(path, ios::in | ios::binary);
    start = filesize.tellg();
    filesize.seekg(0, ios::end);
    end = filesize.tellg();
    filesize.close();  // closed
    cnt = (end - start) / (3 * sizeof(double));

    // then convert
    ifstream input(path, ios::in | ios::binary);
    for (size_t i = 0; i < cnt; ++i) {
      if (input.good() && !input.eof()) {
        pcl::PointXYZ pcl;
        input.read((char *)&pcl.x, sizeof(double));
        input.read((char *)&pcl.y, sizeof(double));
        input.read((char *)&pcl.z, sizeof(double));
        pcl_ptr->push_back(pcl);
      }
    }
    input.close();  // closed
    // to ROS2 pcl2
    sensor_msgs::msg::PointCloud2::SharedPtr pcl_msg_ptr(
        new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*pcl_ptr, *pcl_msg_ptr);
    pcl_msg_ptr->header.stamp = t.Timestamp2Rostime(time_stamp);
    pcl_msg_ptr->header.frame_id = lidar_front3d_frame;

    if (flag) {
      publisher->publish(*pcl_msg_ptr);
      e.make_transforms(pcl_msg_ptr->header.stamp, stereo_frame,
                        lidar_front3d_frame);
    }
  }

 public:
  explicit lidarold() {}
  explicit lidarold(string sp, string dp)
      : sdk_path(sp), dataset_path(dp), e("ldmrs", sp), t("ldmrs", dp) {
    cout << "The 3D LiDAR initialization completed." << endl;
  }
};  // class lidarold

#endif