/*==================================================================
  Software License Agreement (GNU General Public License v3.0)
  http://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2022, Rongxi Zhang
  All rights reserved.

  E-mail: rongxizhangcar@gmail.com
 ===================================================================*/

#ifndef RCL_THREAD_HPP
#define RCL_THREAD_HPP

// QT
#include <QDebug>
#include <QMutex>
#include <QThread>

// mine
#include "radar_robotcar_dataset_ros2/sensor_suite/cameras.hpp"
#include "radar_robotcar_dataset_ros2/sensor_suite/gps.hpp"
#include "radar_robotcar_dataset_ros2/sensor_suite/lidars.hpp"
#include "radar_robotcar_dataset_ros2/sensor_suite/radar.hpp"

// first: timestamp, second: pair<index, sensor_type>
typedef multimap<size_t, pair<int, sensor_type>> timestamp_map;
typedef vector<pair<size_t, pair<int, sensor_type>>> timestamp_vector;

class RCLThread : public QThread {
  Q_OBJECT

 public:
  explicit RCLThread(QObject* parent = nullptr, QMutex* mutex_ptr = nullptr);
  ~RCLThread();

  void inital(rclcpp::Node::SharedPtr);
  double frequency(int index);
  void play_sensors();
  void bag_inital();

 signals:
  void index_signal(int index);
  void range_signal(int maximum);

 public slots:
  void map_slot(sensor_type type);

 protected:
  void run() override;

 public:
  QMutex* Qmutex_ptr;
  rclcpp::Node::SharedPtr node;
  rclcpp::executors::MultiThreadedExecutor executor;
  // playback settings
  bool press_flag = false;
  bool run_flag = true;
  int timestamp_index_pre = -1;
  int timestamp_index = 0;
  double play_speed = 1.0;
  timestamp_map timestamps;
  timestamp_vector timestamps_v;
  // sensor suite
  stereo stereo_camera;
  radar spin_radar;
  mono mono_cameras;
  lidar3d lidar_3ds;
  lidarold lidar_old;
  gps gps_ins;
  lidar2d lidar_2ds;
  // add flags
  bool stereo_added = false;
  bool radar_added = false;
  bool mono_added = false;
  bool lidar3d_added = false;
  bool gps_added = false;
  bool lidar2d_added = false;
  bool lidarold_added = false;
  // checked flags
  bool is_radar_dataset = true;
  bool width = false;
  bool is_single = false;
  bool is_undistort = false;
  bool stereo_checked = false;
  bool radar_checked = false;
  bool mono_checked = false;
  bool lidar3d_checked = false;
  bool gps_checked = false;
  bool lidar2d_checked = false;
  bool lidarold_checked = false;
  // save to rosbag2 (coming soon)
  string bag_name;

};  // class RCLThread

#endif