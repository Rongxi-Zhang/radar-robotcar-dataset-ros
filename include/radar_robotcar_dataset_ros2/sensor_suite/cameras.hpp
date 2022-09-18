/*==================================================================
  Software License Agreement (GNU General Public License v3.0)
  http://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2022, Rongxi Zhang
  All rights reserved.

  E-mail: rongxizhangcar@gmail.com
 ===================================================================*/

#ifndef CAMERAS_HPP
#define CAMERAS_HPP

// opencv
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
// ROS2
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

// mine
#include "radar_robotcar_dataset_ros2/config.hpp"

using namespace cv;
using namespace dnn;

// declare
class extrinsic;
class timestamp;

class calibration {
 public:
  string name;
  string path;
  sensor_type type;

  // Image heigth, width
  Eigen::Vector2d image_size;
  // Rotation Matrix, R
  Eigen::Matrix<double, 3, 3> R;
  // Projection Matrix, P
  Eigen::Matrix<double, 3, 4> P = Eigen::Matrix<double, 3, 4>::Identity();
  // Camera intrinsics, K
  Eigen::Matrix<double, 3, 3> K = Eigen::Matrix3d::Identity();
  // Distortion parameters, default empty. (SDK not used)
  Eigen::Matrix<double, 1, 5> D = Eigen::Matrix<double, 1, 5>::Zero();
  // distortion_lut (SDK use lut to undistort)
  vector<double> lut_1, lut_2;

 public:
  void initial(string sp, string n, sensor_type t) {
    name = n;
    type = t;
    path = sp + "/models/";
    load_intrinsics(path + name + ".txt");
    load_distortion_lut(path + name + "_distortion_lut.bin");
    cout << "Stereo camera model (" + name + ") load completed." << endl;
  }

  // convert them to CameraInfo (RobotCarPlayer doesn't use this func)
  // Cuz those params are constant, there is no need to convert them to
  // CameraInfo msg. If you need CameraInfo msg, you can write the corresponding
  // function by yourself.
  void convert2RosInfo(sensor_msgs::msg::CameraInfo &info) {
    for (size_t i = 0; i < 5; ++i) info.d.push_back(D(0, i));
    for (size_t i = 0; i < 3; ++i)
      for (size_t j = 0; j < 4; ++j) {
        if (j < 3) {
          info.k[i * 3 + j] = K(i, j);
          info.r[i * 3 + j] = R(i, j);
        }
        info.p[i * 4 + j] = P(i, j);
      }
    info.distortion_model = "plumb_bob";
  }

  void load_intrinsics(string filename) {
    ifstream file(filename, ios::in);
    size_t index;
    string strline;
    // K
    getline(file, strline);
    index = strline.find_first_of(' ');
    K(0, 0) = stod(strline.substr(0, index));
    strline = strline.substr(++index);
    index = strline.find_first_of(' ');
    K(1, 1) = stod(strline.substr(0, index));
    strline = strline.substr(++index);
    index = strline.find_first_of(' ');
    K(0, 2) = stod(strline.substr(0, index));
    strline = strline.substr(++index);
    K(1, 2) = stod(strline);
    // R (same)
    R << 0.0, -0.0, 1.0, 1.0, 0.0, -0.0, 0.0, 1.0, 0.0;
    // P
    P.block(0, 0, 3, 3) = R;
    getline(file, strline);
    index = strline.find_last_of(' ');
    P(0, 3) = stod(strline.substr(index, strline.size() - index));
    getline(file, strline);
    index = strline.find_last_of(' ');
    P(1, 3) = stod(strline.substr(index, strline.size() - index));
    file.close();  // closed

    switch (type) {  // lut.size((x,y)|double) 2*960*1280 = 2457600
      case sensor_stereo: {
        image_size(0) = 960;
        image_size(1) = 1280;
        break;
      }
      default:  // mono, lut.size 2*1024*1024 = 2097152
      {
        image_size(0) = 1024;
        image_size(1) = 1024;
        break;
      }
    };
  }
  // According to the SDK, the content of a lut.bin is the new coordinates of
  // remapping the pixel coordinates of a distorted image.
  void load_distortion_lut(string filename) {
    lut_1.clear();
    lut_2.clear();
    // calc its size first
    size_t start, end, cnt = 0;
    ifstream filesize(filename, ios::in | ios::binary);
    start = filesize.tellg();
    filesize.seekg(0, ios::end);
    end = filesize.tellg();
    filesize.close();  // closed
    cnt = (end - start) / (sizeof(double) * 2);
    // then load
    ifstream input(filename, ios::in | ios::binary);
    for (size_t i = 0; i < cnt; ++i) {
      if (input.good() && !input.eof()) {
        double coordinate_1;
        input.read((char *)&(coordinate_1), sizeof(double));
        lut_1.push_back(coordinate_1);
      }
    }
    for (size_t i = 0; i < cnt; ++i) {
      if (input.good() && !input.eof()) {
        double coordinate_2;
        input.read((char *)&(coordinate_2), sizeof(double));
        lut_2.push_back(coordinate_2);
      }
    }
    input.close();  // closed
  }

  // undistort image by cv::remap and the lut
  void undistort(cv::Mat &src, cv::Mat &dst) {
    cv::Mat map_x = cv::Mat::zeros(image_size(0), image_size(1), CV_32F);
    cv::Mat map_y = cv::Mat::zeros(image_size(0), image_size(1), CV_32F);
    int cnt = 0;  // h * w (h rows, each row is w in length)
    for (int i = 0; i < image_size(0); ++i) {    // h
      for (int j = 0; j < image_size(1); ++j) {  // w
        map_y.at<float>(i, j) = lut_2[cnt];
        map_x.at<float>(i, j) = lut_1[cnt++];
      }
    }
    // SDK: [map_coordinates(image[:, :, channel], lut, order=1) for channel
    // in range(0, image.shape[2])].
    // So I use the cv::remap to realize the same result.
    cv::remap(src, dst, map_x, map_y, cv::INTER_LINEAR);
  }

 public:
  explicit calibration() {}

};  // class calibration

class stereo {
 public:
  // basic parameters
  string sdk_path;
  string dataset_path;
  extrinsic e_c, e_r;
  timestamp t;
  // calibration
  calibration c_l, c_c, c_r;
  // rclcpp
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_publisher,
      centre_publisher, right_publisher;
  sensor_msgs::msg::Image::SharedPtr left_msg_ptr, centre_msg_ptr,
      right_msg_ptr;
  // data caches
  // mutex cache_mutex;
  const int cache_size = 400;
  deque<sensor_msgs::msg::Image> left_cache, centre_cache, right_cache;

 public:
  void inital(string sp, string dp, sensor_type type) {
    sdk_path = sp;
    dataset_path = dp;
    e_c.inital("stereo", sp);
    e_r.inital("stereo", sp);
    t.inital("stereo", dp);
    c_l.initial(sp, "stereo_wide_left", type);
    c_c.initial(sp, "stereo_narrow_right", type);
    c_r.initial(sp, "stereo_wide_right", type);
    e_c.trans.transform.translation.y = c_c.P(1, 3);
    e_r.trans.transform.translation.y = c_r.P(1, 3);
    cout << "The stereo camera initialization completed." << endl;
  }

  void cache_read(int index, bool width) {
    left_cache.clear();
    centre_cache.clear();
    right_cache.clear();
    for (int i = index; i < cache_size; ++i)
      index_callback(i, false, width, true);
    cout << "Stereo camera data loading completed." << endl;
  }

  void create_publisher(rclcpp::Node::SharedPtr n) {
    node = n;
    left_publisher = node->create_publisher<sensor_msgs::msg::Image>(
        "/robotcar/stereo/left", 20);
    centre_publisher = node->create_publisher<sensor_msgs::msg::Image>(
        "/robotcar/stereo/centre", 20);
    right_publisher = node->create_publisher<sensor_msgs::msg::Image>(
        "/robotcar/stereo/right", 20);
    e_c.tf_publisher =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(*node);
    e_r.tf_publisher =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(*node);
    cout << "Created publishers for the stereo camera." << endl;
  }

  void index_callback(int index, bool flag, bool width, bool remove) {
    cv::Mat left_image, centre_image, right_image;
    string left_path, centre_path, right_path;
    string time_stamp = t.timestamplist[index];

    string filename = time_stamp + ".png";
    left_path = dataset_path + "/stereo/left/" + filename;
    centre_path = dataset_path + "/stereo/centre/" + filename;
    right_path = dataset_path + "/stereo/right/" + filename;

    left_image = cv::imread(left_path, -1);
    cv::cvtColor(left_image, left_image, cv::COLOR_BayerGR2BGR);
    if (remove) c_l.undistort(left_image, left_image);  // undistort
    left_msg_ptr =
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", left_image)
            .toImageMsg();
    left_msg_ptr->header.stamp = t.Timestamp2Rostime(time_stamp);
    left_msg_ptr->header.frame_id = stereo_frame;
    // (left + centre) || (left + right)
    if (width) {
      centre_image = cv::imread(centre_path, -1);
      cv::cvtColor(centre_image, centre_image, cv::COLOR_BayerGR2BGR);
      if (remove) c_c.undistort(centre_image, centre_image);
      centre_msg_ptr =
          cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", centre_image)
              .toImageMsg();
      centre_msg_ptr->header.stamp = left_msg_ptr->header.stamp;
      centre_msg_ptr->header.frame_id = stereo_centre_frame;
    } else {
      right_image = cv::imread(right_path, -1);
      cv::cvtColor(right_image, right_image, cv::COLOR_BayerGR2BGR);
      if (remove) c_r.undistort(right_image, right_image);
      right_msg_ptr =
          cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", right_image)
              .toImageMsg();
      right_msg_ptr->header.stamp = right_msg_ptr->header.stamp;
      right_msg_ptr->header.frame_id = stereo_right_frame;
    }

    if (flag) {
      left_publisher->publish(*left_msg_ptr);
      if (width) {
        e_c.make_transforms(left_msg_ptr->header.stamp, stereo_frame,
                            stereo_centre_frame);
        centre_publisher->publish(*centre_msg_ptr);
      } else {
        e_r.make_transforms(left_msg_ptr->header.stamp, stereo_frame,
                            stereo_right_frame);
        right_publisher->publish(*right_msg_ptr);
      }
    } else {
      left_cache.push_back(*left_msg_ptr);
      if (width)
        centre_cache.push_back(*centre_msg_ptr);
      else
        right_cache.push_back(*right_msg_ptr);
    }
  }

  void index_publish(int index, bool width) {
    left_publisher->publish(left_cache.at(index));
    if (width)
      centre_publisher->publish(centre_cache.at(index));
    else
      right_publisher->publish(right_cache.at(index));
  }

 public:
  explicit stereo() {}
  explicit stereo(string sp, string dp)
      : sdk_path(sp),
        dataset_path(dp),
        e_c("stereo", sp),
        e_r("stereo", sp),
        t("stereo", dp) {
    cout << "The stereo camera initialization completed." << endl;
  }
};  // class stereo

//-----------------------------------------------------------------------------//

class mono {
 public:
  // basic parameters
  string sdk_path;
  string dataset_path;
  extrinsic e_l, e_re, e_ri;
  timestamp t_l, t_re, t_ri;
  // calibration
  calibration c_l, c_re, c_ri;
  // rclcpp
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_publisher,
      rear_publisher, right_publisher;
  sensor_msgs::msg::Image::SharedPtr left_msg_ptr, rear_msg_ptr, right_msg_ptr;
  // data caches
  // mutex cache_mutex;
  const int cache_size = 400;
  deque<sensor_msgs::msg::Image> left_cache, rear_cache, right_cache;

 public:
  void inital(string sp, string dp) {
    sdk_path = sp;
    dataset_path = dp;
    e_l.inital("mono_left", sp);
    t_l.inital("mono_left", dp);
    e_re.inital("mono_rear", sp);
    t_re.inital("mono_rear", dp);
    e_ri.inital("mono_right", sp);
    t_ri.inital("mono_right", dp);

    c_l.initial(sp, "mono_left", sensor_mono_left);
    c_re.initial(sp, "mono_rear", sensor_mono_rear);
    c_ri.initial(sp, "mono_right", sensor_mono_right);
    cout << "The mono cameras initialization completed." << endl;
  }

  void cache_read(int index) {
    left_cache.clear();
    rear_cache.clear();
    right_cache.clear();
    for (int i = index; i < cache_size; ++i) {
      index_callback(i, false, sensor_mono_left, true);
      index_callback(i, false, sensor_mono_rear, true);
      index_callback(i, false, sensor_mono_right, true);
    }
    cout << "Mono cameras data loading completed." << endl;
  }

  void create_publisher(rclcpp::Node::SharedPtr n) {
    node = n;
    left_publisher = node->create_publisher<sensor_msgs::msg::Image>(
        "/robotcar/mono/left", 20);
    rear_publisher = node->create_publisher<sensor_msgs::msg::Image>(
        "/robotcar/mono/rear", 20);
    right_publisher = node->create_publisher<sensor_msgs::msg::Image>(
        "/robotcar/mono/right", 20);
    e_l.tf_publisher =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(*node);
    e_re.tf_publisher =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(*node);
    e_ri.tf_publisher =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(*node);
    cout << "Created publishers for the mono cameras." << endl;
  }

  void index_callback(int index, bool flag, sensor_type type, bool remove) {
    cv::Mat image;
    string path, time_stamp, filename;

    switch (type) {
      case sensor_mono_left: {
        time_stamp = t_l.timestamplist[index];
        filename = time_stamp + ".png";
        path = dataset_path + "/mono_left/" + filename;

        image = cv::imread(path, -1);
        cv::cvtColor(image, image, cv::COLOR_BayerBG2BGR);
        if (remove) c_l.undistort(image, image);
        left_msg_ptr =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image)
                .toImageMsg();
        left_msg_ptr->header.stamp = t_l.Timestamp2Rostime(time_stamp);
        left_msg_ptr->header.frame_id = mono_left_frame;

        if (flag) {
          left_publisher->publish(*left_msg_ptr);
          // It seems inclined? I have no idea what is the real extrinsic.(@_@)
          e_l.make_inverse_transforms(left_msg_ptr->header.stamp, stereo_frame,
                                      mono_left_frame);
        } else
          left_cache.push_back(*left_msg_ptr);

        break;
      }
      case sensor_mono_rear: {
        time_stamp = t_re.timestamplist[index];
        filename = time_stamp + ".png";
        path = dataset_path + "/mono_rear/" + filename;

        image = cv::imread(path, -1);
        cv::cvtColor(image, image, cv::COLOR_BayerBG2BGR);
        if (remove) c_re.undistort(image, image);
        rear_msg_ptr =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image)
                .toImageMsg();
        rear_msg_ptr->header.stamp = t_re.Timestamp2Rostime(time_stamp);
        rear_msg_ptr->header.frame_id = mono_rear_frame;

        if (flag) {
          rear_publisher->publish(*rear_msg_ptr);
          e_re.make_inverse_transforms(rear_msg_ptr->header.stamp, stereo_frame,
                                       mono_rear_frame);
        } else
          rear_cache.push_back(*rear_msg_ptr);

        break;
      }
      case sensor_mono_right: {
        time_stamp = t_ri.timestamplist[index];
        filename = time_stamp + ".png";
        path = dataset_path + "/mono_right/" + filename;

        image = cv::imread(path, -1);
        cv::cvtColor(image, image, cv::COLOR_BayerBG2BGR);
        if (remove) c_ri.undistort(image, image);
        right_msg_ptr =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image)
                .toImageMsg();
        right_msg_ptr->header.stamp = t_ri.Timestamp2Rostime(time_stamp);
        right_msg_ptr->header.frame_id = mono_right_frame;

        if (flag) {
          right_publisher->publish(*right_msg_ptr);
          e_ri.make_inverse_transforms(right_msg_ptr->header.stamp,
                                       stereo_frame, mono_right_frame);
        } else
          right_cache.push_back(*right_msg_ptr);

        break;
      }
      default:
        break;
    };
  }

  void index_publish(int index) {
    left_publisher->publish(left_cache.at(index));
    rear_publisher->publish(rear_cache.at(index));
    right_publisher->publish(right_cache.at(index));
  }

 public:
  explicit mono() {}
  explicit mono(string sp, string dp)
      : sdk_path(sp),
        dataset_path(dp),
        e_l("mono_left", sp),
        e_re("mono_rear", sp),
        e_ri("mono_right", sp),
        t_l("mono_left", dp),
        t_re("mono_rear", dp),
        t_ri("mono_right", dp) {
    cout << "The mono cameras initialization completed." << endl;
  }

};  // class mono

#endif