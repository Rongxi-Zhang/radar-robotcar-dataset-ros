
# RobotCarPlayer [![License](https://img.shields.io/static/v1?label=License&message=GPLv3&color=green)](http://www.gnu.org/licenses/gpl-3.0.html)
### ROS2 Interface of the Oxford (Radar) RobotCar Dataset

*RobotCarPlayer converts the original data from the [Oxford RobotCar Dataset](https://robotcar-dataset.robots.ox.ac.uk/) and the [Oxford Radar RobotCar Dataset](https://oxford-robotics-institute.github.io/radar-robotcar-dataset/)  to topic messages of **ROS2** and publishes them. The player is based on **ROS2** and **QT5**, and written in pure **C++**, which is different from the Matlab and Python scripts in the original [robotcar-dataset-sdk](https://github.com/ori-mrg/robotcar-dataset-sdk). If you do not use **ROS2** but want to read the original data through **C++**, this work is also a good reference.*

*This work has been tested on **ROS2 Foxy**.*  
*I believe it can also operate well on the future **[ROS](https://www.ros.org/)** :turtle:.*  


## Sensor Suite :car:

<div align=center>
	<img src = assets/RobotCar.png />

| **Sensor** | Oxford RobotCar Dataset | Oxford Radar RobotCar Dataset |
| :----: | :----: | :----: |
| Stereo Camera | 1 x Point Grey Bumblebee XB3 | same |
| Mono Camera | 3 x Point Grey Grasshopper2 | same |
| 2D LiDAR | 2 x SICK LMS-151 2D LiDAR | same |
| GPS/INS | 1 x NovAtel SPAN-CPT | same |
| 3D LiDAR | 1 x SICK LD-MRS 3D LiDAR | 2 x Velodyne HDL-32E |
| mmWave Radar | no | 1 x Navtech CTS350-X |

</div>

## Basic Features :book:
<div align=center>
	<img src = assets/RobotCarPlayer.png />
</div>
:one: Any sensor can be loaded dynamically.  
:two: Support single and multi-sensor modes.  
:three: Support switching between two datasets.  
:four: Support the use of the slider to drag the playback point position.  
:five: It can dynamically remove or not remove the distortion of all camera images.  
:six: The stereo camera can be dynamically switched between the wide mode and narrow mode.  
:seven: For the GPS/INS, this player publishes GPS and UTM odometry topics at the same time.  
:eight: In multi-sensor mode, all sensors have played synchronously according to their timestamps.  
:nine: *Continuous updating ...*

## ROS2 Build :computer:
This work has been tested on **Ubuntu 20.04 LTS**.  
```bash
mkdir -p radar_robotcar_dataset_ros2/src
cd radar_robotcar_dataset_ros2/src
git clone https://github.com/Rongxi-Zhang/radar-robotcar-dataset-ros.git
cd ..
rosdep install src --from-paths -i -y
colcon build --packages-select radar_robotcar_dataset_ros2
source ./devel/setup.bash
```

If you want to build RobotCarPlayer on **Windows**, please refer to the following link: 
[Building ROS 2 on Windows](https://docs.ros.org/en/foxy/Installation/Windows-Development-Setup.html)

## How to use :movie_camera:

```bash
ros2 run radar_robotcar_dataset_ros2 RobotCarPlayer
```
**[Then this video will tell you how to use this player.](https://youtu.be/LG4cgrkSddY)**  

**And if you want to use the [mapviz-2.2.0](https://github.com/swri-robotics/mapviz):**

```bash
ros2 launch radar_robotcar_dataset_ros2 mapviz.launch.py
```

## Citation :star: 
Don't forget to cite RobotCarPlayer in your publications if RobotCarPlayer helps your research.  Your citation will be appreciated. **BibTeX** Entry:
```bibtex
@misc{robotcarplayer,
  author = {Rongxi Zhang},
  title = {RobotCarPlayer: ROS2 Interface of the Oxford (Radar) RobotCar Dataset},
  year = {2022},
  publisher = {GitHub},
  journal = {GitHub repository},
  howpublished = {\url{https://github.com/Rongxi-Zhang/radar-robotcar-dataset-ros}},
}
```
Or, a **footnote** is appreciated: **`github.com/Rongxi-Zhang/radar-robotcar-dataset-ros`**.

If you use RobotCarPlayer in your academic work, please also cite the **official** [paper](https://ieeexplore.ieee.org/document/9196884).  

## Operation Results :tv: 

**rqt:**

<center>
	<img src="assets/rqt.gif">
</center>  
**rviz2:**

<center>
	<img src="assets/rviz2.gif">
</center>  

**[mapviz](https://youtu.be/ojq7yk8xtKU):**

<center>
	<img src="assets/mapviz.gif">
</center>  

*If you use **ROS1**, please refer to the old branch.*  
*If you have any questions just get in touch or you can create an issuse.*  
*And If you want to cooperate with me to improve this work, please also contact me.*  

As a millimeter-wave radar researcher, I will publish some work on radar in the future. Your attention is welcome. 

:e-mail:	rongxizhangcar@gmail.com
