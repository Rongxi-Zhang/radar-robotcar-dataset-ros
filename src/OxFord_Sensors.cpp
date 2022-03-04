#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <csignal>
#include <oxford_sensor.h>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "oxford_sensors");
	ros::NodeHandle n;
	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
	// if not include mono cameras
	//sensor::mono ros_mono(argv[1], argv[2], n); // annotate like this
	// sensor::mono ros_mono(argv[1], argv[2], n);
	// sensor::stereo ros_stereo(argv[1], argv[2], n);
	sensor::radar ros_radar(argv[1], argv[2], n);
	// sensor::lidar ros_lidar(argv[1], argv[2], n);
	sensor::gps ros_gps(argv[1], argv[2], n);

	//  0:not save 1:save
	sensor::save_flag = stoi(argv[3]);
	// multithread publish
	if (!sensor::save_flag)
	{
		// std::thread thread_mono_left = ros_mono.thread_left();
		// std::thread thread_mono_right = ros_mono.thread_right(); // annotate like this
		// std::thread thread_mono_rear = ros_mono.thread_rear();
		// std::thread thread_mono_left = ros_mono.thread_left();
		// std::thread thread_mono_right = ros_mono.thread_right();
		// std::thread thread_mono_rear = ros_mono.thread_rear();
		// std::thread thread_stereo(&sensor::stereo::publish, &ros_stereo);
		std::thread thread_radar(&sensor::radar::publish, &ros_radar);
		// std::thread thread_lidar_left = ros_lidar.thread_left();
		// std::thread thread_lidar_right = ros_lidar.thread_Right();
		std::thread thread_gps(&sensor::gps::publishgps, &ros_gps);
		std::thread thread_ins(&sensor::gps::publishins, &ros_gps);

		// thread_mono_left.join();
		// thread_mono_right.join();   // annotate like this
		// thread_mono_rear.join();
		// thread_mono_left.join();
		// thread_mono_right.join();
		// thread_mono_rear.join();
		thread_radar.join();
		// thread_stereo.join();
		// thread_lidar_left.join();
		// thread_lidar_right.join();
		thread_gps.join();
		thread_ins.join();
	}
	else // save a rosbag.It will take a while.
	{
		sensor::oxford_bag.open(argv[4], rosbag::bagmode::Write);
		// 	ros_mono.publish(0);
		// ros_mono.publish(1); // annotate like this
		// ros_mono.publish(2);
		// ros_mono.publish(0);
		// ros_mono.publish(1);
		// ros_mono.publish(2);
		// ros_stereo.publish();
		ros_radar.publish();
		// ros_lidar.publeft();
		// ros_lidar.pubright();
		// ros_gps.publishgps();
		ros_gps.publishins();
		sensor::oxford_bag.close();
	}

	ros::shutdown();
	return 0;
}