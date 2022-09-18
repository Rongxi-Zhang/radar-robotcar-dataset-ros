/*==================================================================
  Software License Agreement (GNU General Public License v3.0)
  http://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2022, Rongxi Zhang
  All rights reserved.

  E-mail: rongxizhangcar@gmail.com
 ===================================================================*/

#include "radar_robotcar_dataset_ros2/rclthread.hpp"

RCLThread::RCLThread(QObject* parent, QMutex* mutex_ptr)
    : QThread(parent),
      Qmutex_ptr(mutex_ptr),
      executor(rclcpp::executor::ExecutorArgs(), 6, true) {
  qDebug() << "rclcpp thread start";
}

RCLThread::~RCLThread() {
  qDebug() << "rclcpp thread quit";
  rclcpp::shutdown();
}
// coming soon
void RCLThread::bag_inital() {}

void RCLThread::inital(rclcpp::Node::SharedPtr n) {
  node = n;
  executor.add_node(node);
  // create publishers of all sensors
  stereo_camera.create_publisher(n);
  spin_radar.create_publisher(n);
  mono_cameras.create_publisher(n);
  lidar_3ds.create_publisher(n);
  gps_ins.create_publisher(n);
  lidar_2ds.create_publisher(n);
  lidar_old.create_publisher(n);
}

double RCLThread::frequency(int index) {
  if (index) {
    double nsec = timestamps_v[index].first - timestamps_v[index - 1].first;
    return 1e6 / nsec;
  } else
    return 100.0;
}

void RCLThread::play_sensors() {
  rclcpp::Rate wait(100);
  while (rclcpp::ok()) {
    int size = static_cast<int>(timestamps_v.size());
    if (press_flag && timestamp_index != timestamp_index_pre) {
      sensor_type type = timestamps_v[timestamp_index].second.second;
      int index = timestamps_v[timestamp_index].second.first;
      switch (type) {
        case sensor_stereo: {
          if (stereo_added)
            stereo_camera.index_callback(index, true, width, is_undistort);
          break;
        }
        case sensor_mono_left: {
          if (mono_added)
            mono_cameras.index_callback(index, true, type, is_undistort);
          break;
        }
        case sensor_mono_right: {
          if (mono_added)
            mono_cameras.index_callback(index, true, type, is_undistort);
          break;
        }
        case sensor_mono_rear: {
          if (mono_added)
            mono_cameras.index_callback(index, true, type, is_undistort);
          break;
        }
        case sensor_radar: {
          if (radar_added) spin_radar.index_callback(index, true);
          break;
        }
        case sensor_lidar_left: {
          if (lidar3d_added) lidar_3ds.index_callback(index, true, type);
          break;
        }
        case sensor_lidar_right: {
          if (lidar3d_added) lidar_3ds.index_callback(index, true, type);
          break;
        }
        case sensor_gps_ins: {
          if (gps_added) gps_ins.index_callback(index, true, is_radar_dataset);
          break;
        }
        case sensor_lidar_front: {
          if (lidar2d_added) lidar_2ds.index_callback(index, true, type);
          break;
        }
        case sensor_lidar_rear: {
          if (lidar2d_added) lidar_2ds.index_callback(index, true, type);
          break;
        }
        case sensor_lidar_old: {
          if (lidarold_added) lidar_old.index_callback(index, true);
        }
      };
      timestamp_index_pre = timestamp_index;
    } else if (timestamp_index != size - 1) {
      for (int i = timestamp_index; i < size && run_flag; ++i) {
        emit index_signal(i);
        sensor_type type = timestamps_v[i].second.second;
        int index = timestamps_v[i].second.first;
        switch (type) {
          case sensor_stereo: {
            if (stereo_checked) {
              stereo_camera.index_callback(index, true, width, is_undistort);
              if (is_single) {
                double freq = stereo_camera.t.make_frequency(index);
                rclcpp::Rate loop(freq * play_speed);
                loop.sleep();
              }
            }
            break;
          }
          case sensor_mono_left: {
            if (mono_checked) {
              mono_cameras.index_callback(index, true, type, is_undistort);
              if (is_single) {
                double freq = mono_cameras.t_l.make_frequency(index);
                rclcpp::Rate loop(freq * play_speed);
                loop.sleep();
              }
            }
            break;
          }
          case sensor_mono_right: {
            if (mono_checked) {
              mono_cameras.index_callback(index, true, type, is_undistort);
              if (is_single) {
                double freq = mono_cameras.t_ri.make_frequency(index);
                rclcpp::Rate loop(freq * play_speed);
                loop.sleep();
              }
            }
            break;
          }
          case sensor_mono_rear: {
            if (mono_checked) {
              mono_cameras.index_callback(index, true, type, is_undistort);
              if (is_single) {
                double freq = mono_cameras.t_ri.make_frequency(index);
                rclcpp::Rate loop(freq * play_speed);
                loop.sleep();
              }
            }
            break;
          }
          case sensor_radar: {
            if (radar_checked) {
              spin_radar.index_callback(index, true);
              if (is_single) {
                double freq = spin_radar.t.make_frequency(index);
                rclcpp::Rate loop(freq * play_speed);
                loop.sleep();
              }
            }
            break;
          }
          case sensor_lidar_left: {
            if (lidar3d_checked) {
              lidar_3ds.index_callback(index, true, type);
              if (is_single) {
                double freq = lidar_3ds.t_l.make_frequency(index);
                rclcpp::Rate loop(freq * play_speed);
                loop.sleep();
              }
            }
            break;
          }
          case sensor_lidar_right: {
            if (lidar3d_checked) {
              lidar_3ds.index_callback(index, true, type);
              if (is_single) {
                double freq = lidar_3ds.t_r.make_frequency(index);
                rclcpp::Rate loop(freq * play_speed);
                loop.sleep();
              }
            }
            break;
          }
          case sensor_gps_ins: {
            if (gps_checked) {
              gps_ins.index_callback(index, true, is_radar_dataset);
              if (is_single) {
                double freq = gps_ins.t.make_frequency(index);
                rclcpp::Rate loop(freq * play_speed);
                loop.sleep();
              }
            }
            break;
          }
          case sensor_lidar_front: {
            if (lidar2d_checked) {
              lidar_2ds.index_callback(index, true, type);
              if (is_single) {
                double freq = lidar_2ds.t_f.make_frequency(index);
                rclcpp::Rate loop(freq * play_speed);
                loop.sleep();
              }
            }
            break;
          }
          case sensor_lidar_rear: {
            if (lidar2d_checked) {
              lidar_2ds.index_callback(index, true, type);
              if (is_single) {
                double freq = lidar_2ds.t_r.make_frequency(index);
                rclcpp::Rate loop(freq * play_speed);
                loop.sleep();
              }
            }
            break;
          }
          case sensor_lidar_old: {
            if (lidarold_checked) {
              lidar_old.index_callback(index, true);
              if (is_single) {
                double freq = lidar_old.t.make_frequency(index);
                rclcpp::Rate loop(freq * play_speed);
                loop.sleep();
              }
            }
            break;
          }
        };
        if (press_flag) break;
        if (!is_single) {
          double freq = 100;
          rclcpp::Rate loop(freq * play_speed);
          loop.sleep();
        }
      }
      // wait signals from the mainwindow
      if (!run_flag) wait.sleep();
    } else
      // wait signals from the mainwindow
      wait.sleep();
  }
}

//---------------------------------run-----------------------------------//
void RCLThread::run() {
  qDebug() << "rclcpp thread running..";
  // play all sensors
  if (stereo_added || radar_added || mono_added || lidar3d_added || gps_added ||
      lidar2d_added || lidarold_added) {
    // for the stereo(bool width): 0 (left+right), 1(left+centre).
    // and the remaining publisher (centre or right) is empty.
    play_sensors();
  } else
    qDebug() << "Data has not been loaded yet, Sorry.";
  executor.spin();
}

//--------------------------------slots----------------------------------//
void RCLThread::map_slot(sensor_type type) {
  qDebug() << "Add signal received, waitting...";
  switch (type) {
    case sensor_stereo: {
      if (!stereo_added) {
        int size = stereo_camera.t.size();
        for (int i = 0; i < size; ++i) {
          timestamps.insert(make_pair(stol(stereo_camera.t.timestamplist[i]),
                                      make_pair(i, type)));
        }
        stereo_added = true;
        qDebug() << "Timestamps of the stereo camera are added."
                    "Current size ="
                 << timestamps.size();
      } else
        qDebug() << "Timestamps of the stereo camera have already added. "
                    "Current size ="
                 << timestamps.size();
      break;
    }
    case sensor_radar: {
      if (!radar_added) {
        int size = spin_radar.t.size();
        for (int i = 0; i < size; ++i) {
          timestamps.insert(make_pair(stol(spin_radar.t.timestamplist[i]),
                                      make_pair(i, type)));
        }
        radar_added = true;
        qDebug() << "Timestamps of the radar are added."
                    "Current size ="
                 << timestamps.size();
      } else
        qDebug() << "Timestamps of the radar have already added. "
                    "Current size ="
                 << timestamps.size();
      break;
    }
    case sensor_lidar_left: {
      if (!lidar3d_added) {
        int size = lidar_3ds.t_l.size();
        for (int i = 0; i < size; ++i) {
          timestamps.insert(make_pair(stol(lidar_3ds.t_l.timestamplist[i]),
                                      make_pair(i, sensor_lidar_left)));
        }
        size = lidar_3ds.t_r.size();
        for (int i = 0; i < size; ++i) {
          timestamps.insert(make_pair(stol(lidar_3ds.t_r.timestamplist[i]),
                                      make_pair(i, sensor_lidar_right)));
        }
        lidar3d_added = true;
        qDebug() << "Timestamps of the 3D LiDARs are added."
                    "Current size ="
                 << timestamps.size();
      } else
        qDebug() << "Timestamps of the 3D LiDARs have already added. "
                    "Current size ="
                 << timestamps.size();
      break;
    }
    case sensor_lidar_right: {
      // above
      break;
    }
    case sensor_gps_ins: {
      if (!gps_added) {
        int size = gps_ins.t.size();
        for (int i = 0; i < size; ++i) {
          timestamps.insert(
              make_pair(stol(gps_ins.t.timestamplist[i]), make_pair(i, type)));
        }
        gps_added = true;
        qDebug() << "Timestamps of the GPS/INS are added."
                    "Current size ="
                 << timestamps.size();
      } else
        qDebug() << "Timestamps of the GPS/INS have already added. "
                    "Current size ="
                 << timestamps.size();
      break;
    }
    case sensor_lidar_front: {
      if (!lidar2d_added) {
        int size = lidar_2ds.t_f.size();
        for (int i = 0; i < size; ++i) {
          timestamps.insert(make_pair(stol(lidar_2ds.t_f.timestamplist[i]),
                                      make_pair(i, sensor_lidar_front)));
        }
        size = lidar_2ds.t_r.size();
        for (int i = 0; i < size; ++i) {
          timestamps.insert(make_pair(stol(lidar_2ds.t_r.timestamplist[i]),
                                      make_pair(i, sensor_lidar_rear)));
        }
        lidar2d_added = true;
        qDebug() << "Timestamps of the 2D LiDARs are added."
                    "Current size ="
                 << timestamps.size();
      } else
        qDebug() << "Timestamps of the 2D LiDARs have already added. "
                    "Current size ="
                 << timestamps.size();
      break;
    }
    case sensor_lidar_rear: {
      // above
      break;
    }
    case sensor_lidar_old: {
      if (!lidarold_added) {
        int size = lidar_old.t.size();
        for (int i = 0; i < size; ++i) {
          timestamps.insert(make_pair(stol(lidar_old.t.timestamplist[i]),
                                      make_pair(i, sensor_lidar_old)));
        }
        lidarold_added = true;
        qDebug() << "Timestamps of the front 3D LiDAR is added."
                    "Current size ="
                 << timestamps.size();
      } else
        qDebug() << "Timestamps of the front 3D LiDAR has already added. "
                    "Current size ="
                 << timestamps.size();
      break;
    }
    default: {  // mono
      if (!mono_added) {
        int size = mono_cameras.t_l.size();
        for (int i = 0; i < size; ++i) {
          timestamps.insert(make_pair(stol(mono_cameras.t_l.timestamplist[i]),
                                      make_pair(i, sensor_mono_left)));
        }

        size = mono_cameras.t_re.size();
        for (int i = 0; i < size; ++i) {
          timestamps.insert(make_pair(stol(mono_cameras.t_re.timestamplist[i]),
                                      make_pair(i, sensor_mono_rear)));
        }

        size = mono_cameras.t_ri.size();
        for (int i = 0; i < size; ++i) {
          timestamps.insert(make_pair(stol(mono_cameras.t_ri.timestamplist[i]),
                                      make_pair(i, sensor_mono_right)));
        }

        mono_added = true;
        qDebug() << "Timestamps of the mono cameras are added."
                    "Current size ="
                 << timestamps.size();
      } else
        qDebug() << "Timestamps of the mono cameras have already added. "
                    "Current size ="
                 << timestamps.size();
      break;
    }
  };
  // copy to the vector
  timestamps_v.clear();
  for (timestamp_map::iterator timestamp_iter = timestamps.begin();
       timestamp_iter != timestamps.end(); ++timestamp_iter) {
    timestamps_v.push_back(*timestamp_iter);
  }
  qDebug() << "Add signal processing complete.";
  qDebug() << "Set signal is emitted.";
  emit range_signal(timestamps_v.size() - 1);
  this->sleep(1);
}
