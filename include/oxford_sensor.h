#include <string>
#include <cmath>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <rosbag/bag.h>

#include <Eigen/Core>
#include <Eigen/StdVector>

using namespace std;

namespace sensor
{
    // oxford_tf
    tf2_msgs::TFMessage tf2_msgs;
    // oxford_bag
    rosbag::Bag oxford_bag;
    bool save_flag;
    //-------------------sensors----------------------
    // camera calibration
    struct CameraCalibration
    {
        // Image heigth, width
        Eigen::Vector2d image_size;
        // Rotation Matrix, R
        Eigen::Matrix<double, 3, 3> R;
        // Projection Matrix, P
        Eigen::Matrix<double, 3, 4> P = Eigen::Matrix<double, 3, 4>::Identity();
        // Camera intrinsics, K
        Eigen::Matrix<double, 3, 3> K = Eigen::Matrix3d::Identity();
        // Distortion parameters, default empty.
        Eigen::Matrix<double, 1, 5> D = Eigen::Matrix<double, 1, 5>::Zero();
        // convert them to ROS
        void ToCameraInfo(sensor_msgs::CameraInfo &info);
        void readcali(sensor_msgs::CameraInfo &info, string filename);
    };

    class mono
    {
    private:
        string mono_model;
        string mono_path;
        ros::NodeHandle n;
        vector<string> LeftTimeList, RightTimeList, RearTimeList;

        sensor_msgs::CameraInfo info_mono_left, info_mono_right, info_mono_rear;
        std::mutex mono_mutex;

    public:
        mono(const string &config_file, const string &dataset_path, ros::NodeHandle node);
        void publish(int flag); // 0:left, 1:right, 2:rear

        std::thread thread_left()
        {
            return std::thread(&mono::publish, this, 0);
        }
        std::thread thread_right()
        {
            return std::thread(&mono::publish, this, 1);
        }
        std::thread thread_rear()
        {
            return std::thread(&mono::publish, this, 2);
        }

    }; // class mono

    class stereo
    {
    private:
        string stereo_model;
        string stereo_path;
        ros::NodeHandle n;
        vector<string> imageTimeList;

        sensor_msgs::CameraInfo info_wide_left, info_wide_right;
        sensor_msgs::CameraInfo info_narrow_left, info_narrow_right;
        std::mutex stereo_mutex;

    public:
        stereo(const string config_file, const string dataset_path, ros::NodeHandle node);
        void publish();

    }; // class stereo

    class radar
    {
    private:
        string radar_path;
        ros::NodeHandle n;
        vector<string> RadarTimeList;
        // radar parameters for current scan
        int range_bins;
        int encoder_size;
        float radar_resolution;
        // current parameters should be locked
        std::mutex radar_mutex;

    public:
        radar(const string config_file, const string dataset_path, ros::NodeHandle node);
        void load_radar(cv::Mat &polar_data, cv::Mat &fft_data, std::vector<bool> &valid,
                        std::vector<double> &azimuths, std::vector<int64_t> &timestamps);
        void radar_polar_to_cartesian(cv::Mat &fft_data, std::vector<bool> &valid, std::vector<double> &azimuths,
                                      std::vector<int64_t> &timestamps, float cart_resolution, int cart_pixel_width,
                                      bool interpolate_crossover, cv::Mat &cart_img, int output_type);
        void publish();

    }; // class radar

    class lidar
    {
    private:
        ros::NodeHandle n;
        string left_path, right_path;
        vector<string> LidarLeftTimeList, LidarRightTimeList;
        std::mutex lidar_mutex;

    public:
        lidar(const string config_file, const string dataset_path, ros::NodeHandle node);
        void publish(vector<string> TimeList, bool flag); // left=0, right=1
        void publeft()
        {
            lidar_mutex.lock();
            publish(LidarLeftTimeList, 0);
            lidar_mutex.unlock();
        }
        void pubright()
        {
            lidar_mutex.lock();
            publish(LidarRightTimeList, 1);
            lidar_mutex.unlock();
        }
        std::thread thread_left()
        {
            return std::thread(&lidar::publish, this, LidarLeftTimeList, 0);
        }
        std::thread thread_Right()
        {
            return std::thread(&lidar::publish, this, LidarRightTimeList, 1);
        }
    }; // class lidar

    class gps
    {
    private:
        struct GpsStructure
        {
            string timestamp;
            uint num_satellites;
            double latitude, longitude, altitude;
            double latitude_sigma, longitude_sigma, altitude_sigma;
            double northing, easting, down;
            string utm_zone;
        };
        struct InsStructure
        {
            string timestamp, ins_status;
            double latitude, longitude, altitude;
            double northing, easting, down;
            string utm_zone;
            double velocity_north, velocity_east, velocity_down;
            float roll, pitch, yaw;
        };
        ros::NodeHandle n;
        string gps_path, ins_path;
        vector<GpsStructure> oxford_gps;
        vector<InsStructure> oxford_ins;
        std::mutex gps_mutex, ins_mutex;

    public:
        gps(const string config_file, const string dataset_path, ros::NodeHandle node);
        void publishgps();
        void publishins();
    }; // class gps/ins

    // namespace funcs
    std::mutex oxford_mutex;
    ros::Time Timestamp2Rostime(string timestamp);
    uint64 Timestamp(string timestamp);
    double Frequency(string next, string now);
    void TransformBroadcaster(geometry_msgs::TransformStamped);
    void TransformBroadcaster(vector<float> trans, ros::Time timestamp, string frameId, string childId);
    void SaveTf2Rosbag(ros::Time timestamp);
    geometry_msgs::TransformStamped Transform(vector<float> trans, ros::Time timestamp, string frameId, string childId);

} // namespace sensor