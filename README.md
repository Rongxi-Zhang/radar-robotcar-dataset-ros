# radar-robotcar-dataset-ros
### ROS interface of The Oxford Radar RobotCar Dataset

*I'm glad to be provided access to download the full dataset. This interface is my first work on this dataset because I usually use ROS for research. This work is based on **ROS1** Melodic/Noetic.*

The Oxford Radar RobotCar Dataset is a radar extension to The Oxford RobotCar Dataset. We provide data from a **Navtech CTS350-X Millimetre-Wave FMCW radar** and **Dual Velodyne HDL-32E LIDARs** with **optimised ground truth radar odometry** for **280 km of driving around Oxford, UK** (in addition to all sensors in the original [Oxford RobotCar Dataset](https://robotcar-dataset.robots.ox.ac.uk/)).  

**The official dataset and SDK are here:**  
[radar-robotcar-dataset-sdk](https://github.com/oxford-robotics-institute/radar-robotcar-dataset-sdk)  
[Oxford Radar RobotCar Dataset](https://oxford-robotics-institute.github.io/radar-robotcar-dataset/)

## Sensor Suite
<div align=center>
<img src = pictures/radar-robotcar.png width="450" height="450" />
</div>

**Cameras:**  
1 x Point Grey Bumblebee XB3  
3 x Point Grey Grasshopper2  

**LIDAR:**  
2 x SICK LMS-151 2D LIDAR  

**GPS/INS:**  
1 x NovAtel SPAN-CPT ALIGN inertial and GPS navigation system  

**Radar:**  
1 x Navtech CTS350-X - Mounted in the centre of the roof aligned with the vehicles axes.  

**LIDAR:**  
2 x Velodyne HDL-32E - Mounted to the left and right of the Navtech CTS350-X radar.

**This ROS interface doesn't contain 2D LIDARs so far.**

<div align=center>
<img src = pictures/dataset-directory.png width="450" height="350" />
</div>

## ROS Build

Before compiling this project, make sure that the relevant data of all sensors are in the above figure. **If you only use some of them, you should comment on the code related to sensors not included in the main function (see src/Oxford_Sensors.cpp).** 

```bash
mkdir -p catkin_ws/src
cd catkin/src
git clone https://github.com/Rongxi-Zhang/radar-robotcar-dataset-ros.git
cd ..
rosdep install --from-paths src --ignore-src --rosdistro=melodic -y
catkin_make install -DCATKIN_WHITELIST_PACKAGES="radar_robotcar_dataset_ros"
source ~/catkin_ws/devel/setup.bash
```

## ROS Launch

Please edit the launch file named **Oxford_Sensors.launch** under the launch folder. You can choose to save raw data as a rosbag or publish them directly. 

```xml
<launch>
    <!-- Offical SDK -->
    <arg name="config_file" value=".../robotcar-dataset-sdk"/>
    <!-- Dataset Path -->
    <arg name="dataset_path" value=".../2019-01-10-14-36-48-radar-oxford-10k-partial"/>
    <!-- Save flag: 0 No, 1 Yes -->
    <arg name="save_flag" value="0"/>
    <!-- Save Path -->
    <arg name="save_path" value=".../2019-01-10-14-36-48-radar-oxford-10k-partial.bag" />
	<!--The following is hidden-->
</launch>
```
Then execute the command: 
```shell
roslaunch radar_robotcar_dataset_ros OxFord_Sensors.launch
```

## Operation Results
**Topics:**

<img src = pictures/robotcar-monitor.png width="720" height="402" />



**Rviz:**

![rviz](pictures/robotcar-rviz.gif)



**[Mapviz](https://github.com/swri-robotics/mapviz):**

![mapviz](pictures/robotcar-mapviz.gif)



*A new version based on **ROS2** Foxy will be released in the future. Because of the limited personal level, there are still some deficiencies and the need to improve the place, still, need to be perfect. I would appreciate it if you'd help me improve this project. If you have any questions just get in touch.*
rongxizhangcar@gmail.com

