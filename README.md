# ROS-Turtlesim
## Demo
https://user-images.githubusercontent.com/49962634/169905243-c13794ab-ec46-4e3a-afb9-45f8df491e14.mov

## Environment
ROS distro: noetic <br>
ROS version: 1.15.14 <br>
OS: Ubuntu 20.04.4 LTS x86_64
## How to use
Download `install_script.sh`

Give execute permission
```
chmod +x install_script.sh
```
Then
```
./install_script.sh
```
Configure the ros environment
```
source /opt/ros/noetic/setup.bash
```
Source the pet_ws workspace
```
cd pet_ws/
```
```
. .bashrc
```
roslaunch
```
roslaunch system_bringup system_bringup.launch
```

**Use rosservice, in the second terminal**

Note: remember to source `.bashrc` under `/pet_ws` every time start a new terminal

To run in a circle with radius = 1
```
rosservice call /turtle_drive "{task: circle, radius: 1}"
```
To run in a square with side length = 1
```
rosservice call /turtle_drive "{task: square, length: 1}"
```
To follow waypoints
```
rosservice call /turtle_drive "{task: custom, waypoints: {poses: [{pose: {position: {x: 1, y: 1}}}, {pose: {position: {x: 3, y: 5}}}, {pose: {position: {x: 7, y: 7}}}]}}"
```
