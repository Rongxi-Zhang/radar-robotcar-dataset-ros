/*==================================================================
  Software License Agreement (GNU General Public License v3.0)
  http://www.gnu.org/licenses/gpl-3.0.html

  Copyright (c) 2022, Rongxi Zhang
  All rights reserved.

  E-mail: rongxizhangcar@gmail.com
 ===================================================================*/

#ifndef RADAR_HPP
#define RADAR_FPP

// opencv
#include <opencv2/opencv.hpp>
// ROS2
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/msg/image.hpp>
// mine
#include "radar_robotcar_dataset_ros2/config.hpp"

// declare
class extrinsic;
class timestamp;

class radar {
 public:
  // basic parameters
  string sdk_path;
  string dataset_path;
  extrinsic e;
  timestamp t;
  // radar parameters
  int range_bins = 3768;
  int encoder_size = 5600;
  float radar_resolution = 0.0432;
  // rclcpp
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr polar_publisher,
      cart_publisher;
  sensor_msgs::msg::Image::SharedPtr polar_msg_ptr, cart_msg_ptr;
  // data caches
  // mutex cache_mutex;
  const int cache_size = 100;
  deque<sensor_msgs::msg::Image> polar_cache, cart_cache;

 public:
  void inital(string sp, string dp) {
    sdk_path = sp;
    dataset_path = dp;
    e.inital("radar", sp);
    t.inital("radar", dp);
    cout << "The radar initialization completed." << endl;
  }

  void cache_read(int index) {
    polar_cache.clear();
    cart_cache.clear();
    for (int i = index; i < cache_size; ++i) index_callback(i, false);
    cout << "Radar data loading completed." << endl;
  }

  void create_publisher(rclcpp::Node::SharedPtr n) {
    node = n;
    polar_publisher = node->create_publisher<sensor_msgs::msg::Image>(
        "/robotcar/radar/polar", 20);
    cart_publisher = node->create_publisher<sensor_msgs::msg::Image>(
        "/robotcar/radar/cart", 20);
    e.tf_publisher =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(*node);
    cout << "Created publishers for the radar." << endl;
  }

  void load_radar(cv::Mat &polar_data, cv::Mat &fft_data,
                  std::vector<bool> &valid, std::vector<double> &azimuths,
                  std::vector<int64_t> &timestamps) {
    int N = polar_data.rows;
    timestamps = std::vector<int64_t>(N, 0);
    azimuths = std::vector<double>(N, 0);
    valid = std::vector<bool>(N, true);
    fft_data = cv::Mat::zeros(N, range_bins, CV_32F);

    for (int i = 0; i < N; ++i) {
      uchar *byteArray = polar_data.ptr<uchar>(i);
      timestamps[i] = *((int64_t *)(byteArray));
      azimuths[i] =
          *((uint16_t *)(byteArray + 8)) * 2 * M_PI / double(encoder_size);
      valid[i] = byteArray[10] == 255;
      for (int j = 42; j < range_bins; j++) {
        fft_data.at<float>(i, j) = (float)*(byteArray + 11 + j) / 255.0;
      }
    }
  }

  void radar_polar_to_cartesian(cv::Mat &fft_data,
                                std::vector<double> &azimuths,
                                float cart_resolution, int cart_pixel_width,
                                bool interpolate_crossover, cv::Mat &cart_img,
                                int output_type) {
    float cart_min_range = (cart_pixel_width / 2) * cart_resolution;
    if (cart_pixel_width % 2 == 0)
      cart_min_range = (cart_pixel_width / 2 - 0.5) * cart_resolution;

    cv::Mat map_x = cv::Mat::zeros(cart_pixel_width, cart_pixel_width, CV_32F);
    cv::Mat map_y = cv::Mat::zeros(cart_pixel_width, cart_pixel_width, CV_32F);

    for (int j = 0; j < map_y.cols; ++j) {
      for (int i = 0; i < map_y.rows; ++i) {
        map_y.at<float>(i, j) = -1 * cart_min_range + j * cart_resolution;
      }
    }

    for (int i = 0; i < map_x.rows; ++i) {
      for (int j = 0; j < map_x.cols; ++j) {
        map_x.at<float>(i, j) = cart_min_range - i * cart_resolution;
      }
    }
    cv::Mat range = cv::Mat::zeros(cart_pixel_width, cart_pixel_width, CV_32F);
    cv::Mat angle = cv::Mat::zeros(cart_pixel_width, cart_pixel_width, CV_32F);

    double azimuth_step = azimuths[1] - azimuths[0];
    for (int i = 0; i < range.rows; ++i) {
      for (int j = 0; j < range.cols; ++j) {
        float x = map_x.at<float>(i, j);
        float y = map_y.at<float>(i, j);
        float r = (sqrt(pow(x, 2) + pow(y, 2)) - radar_resolution / 2) /
                  radar_resolution;
        if (r < 0) r = 0;
        range.at<float>(i, j) = r;

        float theta = atan2f(y, x);
        if (theta < 0) theta += 2 * M_PI;
        angle.at<float>(i, j) = (theta - azimuths[0]) / azimuth_step;
      }
    }
    if (interpolate_crossover) {
      cv::Mat a0 = cv::Mat::zeros(1, fft_data.cols, CV_32F);
      cv::Mat aN_1 = cv::Mat::zeros(1, fft_data.cols, CV_32F);
      for (int j = 0; j < fft_data.cols; ++j) {
        a0.at<float>(0, j) = fft_data.at<float>(0, j);
        aN_1.at<float>(0, j) = fft_data.at<float>(fft_data.rows - 1, j);
      }
      cv::vconcat(aN_1, fft_data, fft_data);
      cv::vconcat(fft_data, a0, fft_data);
      angle = angle + 1;
    }
    cv::remap(fft_data, cart_img, range, angle, cv::INTER_LINEAR,
              cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    if (output_type == CV_8UC1) {
      double min, max;
      cv::minMaxLoc(cart_img, &min, &max);
      cart_img.convertTo(cart_img, CV_8UC1, 255.0 / max);
    }
  }

  void index_callback(int index, bool flag) {
    std::vector<bool> valid;
    std::vector<double> azimuths;
    std::vector<int64_t> timestamps;

    cv::Mat polar_radar, fft_data, cart_radar;
    string radar_path;

    string time_stamp = t.timestamplist[index];
    string filename = time_stamp + ".png";
    radar_path = dataset_path + "/radar/" + filename;

    polar_radar = cv::imread(radar_path, 0);
    load_radar(polar_radar, fft_data, valid, azimuths, timestamps);
    radar_polar_to_cartesian(fft_data, azimuths, 0.2592, 924, true, cart_radar,
                             CV_8UC1);

    polar_msg_ptr =
        cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", polar_radar)
            .toImageMsg();
    polar_msg_ptr->header.frame_id = radar_frame;
    polar_msg_ptr->header.stamp = t.Timestamp2Rostime(time_stamp);

    cart_msg_ptr =
        cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", cart_radar)
            .toImageMsg();
    cart_msg_ptr->header.frame_id = radar_frame;
    cart_msg_ptr->header.stamp = polar_msg_ptr->header.stamp;

    if (flag) {
      polar_publisher->publish(*polar_msg_ptr);
      cart_publisher->publish(*cart_msg_ptr);
      e.make_transforms(polar_msg_ptr->header.stamp, stereo_frame, radar_frame);
    } else {
      // for savebag
      polar_cache.push_back(*polar_msg_ptr);
      cart_cache.push_back(*cart_msg_ptr);
    }
  }

  void index_publish(int index) {
    polar_publisher->publish(polar_cache.at(index));
    cart_publisher->publish(cart_cache.at(index));
    polar_cache.pop_front();
    cart_cache.pop_front();
    index_callback(cache_size, false);
  }

 public:
  explicit radar() {}
  explicit radar(string sp, string dp)
      : sdk_path(sp), dataset_path(dp), e("radar", sp) {
    cout << "The radar initialization completed." << endl;
  }

};  // class radar

#endif
