#!/usr/bin/env python
# coding=UTF-8
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError
      

import argparse # argparse是python用于解析命令行参数和选项的标准模块
import os   # 添加python环境下对文件，文件夹执行操作的一个模块
# radar.py中的函数
from radar import load_radar, radar_polar_to_cartesian
import numpy as np
import cv2
import threading

# 创建一个argparse解析对象，添加对象描述
parser = argparse.ArgumentParser(description='Play back radar data from a given directory')
# 添加命令行参数dir，类型string，表示radar数据所在的文件夹
parser.add_argument('dir', type=str, help='Directory containing radar data.')
# args获取命令行参数
args = parser.parse_args(rospy.myargv()[1:])
# 获取radar时间戳的路径
# os.path: 把目录和文件名合成一个路径；os.pardir: 获取当前目录的父目录
timestamps_path = os.path.join(os.path.join(args.dir, os.pardir, 'radar.timestamps'))
# 判断该路径是否是radar时间戳文件
if not os.path.isfile(timestamps_path):
    raise IOError("Could not find timestamps file")

# Cartesian Visualsation Setup: 笛卡尔可视化设置
# Resolution of the cartesian form of the radar scan in metres per pixel:
# 笛卡尔形式雷达扫描图的的分辨率，单位：米/像素
cart_resolution = .65
# Cartesian visualisation size (used for both height and width)
# 笛卡尔雷达图尺寸（用于高度和宽度）
cart_pixel_width = 500  # pixels
interpolate_crossover = True    # 相邻回波进 w行插值
# 读radar时间戳txt文件，文件名有s
timestamps = np.loadtxt(timestamps_path, delimiter=' ', usecols=[0], dtype=np.int64)
# 遍历时间戳txt文件，从而获取.png的文件名

def RadarProcess(radar_timestamp):
    filename = os.path.join(args.dir, str(radar_timestamp) + '.png')
    # 判断是否是radar的.png文件
    if not os.path.isfile(filename):
        raise FileNotFoundError("Could not find radar example: {}".format(filename))
    # 调用load_radar函数获取radar信息
    timestamps, azimuths, valid, fft_data, radar_resolution = load_radar(filename)
    # 生成笛卡尔坐标雷达扫描图
    cart_img = radar_polar_to_cartesian(azimuths, fft_data, radar_resolution, cart_resolution, cart_pixel_width,
                                        interpolate_crossover)
    # Combine polar and cartesian for visualisation
    # The raw polar data is resized to the height of the cartesian representation
    # 雷达极坐标和笛卡尔坐标共同可视化
    # 原始极轴数据的大小调整为笛卡尔表示的高度
    downsample_rate = 4
    fft_data_vis = fft_data[:, ::downsample_rate]
    resize_factor = float(cart_img.shape[0]) / float(fft_data_vis.shape[0]) #缩放系数
    fft_data_vis = cv2.resize(fft_data_vis, (0, 0), None, resize_factor, resize_factor)
    vis = cv2.hconcat((fft_data_vis, cart_img))
    # Convert 32FC1 to 8UC1(mono8), for ros imgmsg
    vis_mono8 = (vis[:, :].copy().view(np.float32)[:, :, np.newaxis]*255).astype(np.uint8)
    polar = vis_mono8[0:fft_data_vis.shape[0], 0:fft_data_vis.shape[1]]
    cart = vis_mono8[0:fft_data_vis.shape[0], fft_data_vis.shape[1]: vis_mono8.shape[1]]
    # cv2.imshow("polar_radar", polar)
    # cv2.imshow("cart_radar",cart) 
    # cv2.waitKey(1)
    return polar, cart

def Timestamps2Rostime(radar_timestamp):
    strtime = str(radar_timestamp)
    sec = int(strtime[:-9])
    nsec = int(strtime[-9:])
    rostime = rospy.Time(sec, nsec)
    return rostime

def Duration(stamp_now, stamp_next):
    sec_now = stamp_now.to_sec()
    nsec_now = stamp_now.to_nsec()
    sec_next = stamp_next.to_sec()
    nsec_next = stamp_next.to_nsec()
    dur = rospy.Duration(sec_next - sec_now, nsec_next - nsec_now)
    return dur

def spin_once():
    rospy.spin()

def pub_radar(radar_timestamps):    
    rospy.init_node('build_radar',anonymous=True)
    pub_polar = rospy.Publisher('oxford/polar_radar', Image, 10)
    pub_cart = rospy.Publisher('oxford/cart_radar', Image, 10)

    i = 0
    bridge = CvBridge()
    sentry = np.size(radar_timestamps) - 1
    rospy.loginfo("Publishing Radar...")
    rate = rospy.Rate(4) 

    for radar_timestamp in radar_timestamps:
        if rospy.is_shutdown():
            break
        polar_radar, cart_radar = RadarProcess(radar_timestamp)

        polar_msg = bridge.cv2_to_imgmsg(polar_radar, "mono8")
        polar_msg.header.frame_id = 'oxford_radar'
        polar_msg.header.stamp = Timestamps2Rostime(radar_timestamp) 
        pub_polar.publish(polar_msg)

        cart_msg = bridge.cv2_to_imgmsg(cart_radar, "mono8")
        cart_msg.header.frame_id = 'oxford_radar'
        cart_msg.header.stamp = Timestamps2Rostime(radar_timestamp) 
        pub_cart.publish(cart_msg)
        
        if(i < sentry):
            rate.sleep()
            i =i +1
        

def save_cart(dir,radar_timestamps):
    path = dir + "/cart"
    if not os.path.exists(path):
        print("No cart radar dir, mkdir in dataset/radar/ .")
        os.mkdir(path)
    for radar_timestamp in radar_timestamps:        
        polar_radar, cart_radar = RadarProcess(radar_timestamp)
        filename = os.path.join(path, str(radar_timestamp) + '.png')
        cv2.imwrite(filename, cart_radar,[cv2.IMWRITE_PNG_COMPRESSION, 0])


if __name__ == '__main__':
    try:
        save_cart(args.dir, timestamps)
    except rospy.ROSInterruptException:
        pass