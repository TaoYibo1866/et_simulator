# et_simulator
## Prerequisites
+ download webots_2021a_amd64.deb from https://github.com/cyberbotics/webots/releases/tag/R2021a
+ install webots and other packages
```Bash
$ sudo dpkg -i webots_2021a_amd64.deb
$ sudo echo "export WEBOTS_HOME=/usr/local/webots" >> ~/.bashrc
$ source ~/.bashrc
$ sudo apt-get install libeigen3-dev
$ sudo apt-get install ros-noetic-webots-ros
$ sudo apt-get install ros-noetic-joy
```
## Build
```Bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/et_simulator.git
$ cd ~/catkin_ws
$ catkin_make
```
## Run
```Bash
$ roslaunch et_webots ET3116A_M100.launch
$ roslaunch et_webots atp_control.launch
$ roslaunch et_webots uav_control.launch
```
![image](demo.png)