# ROS-Turtlesim
## Sample video


https://user-images.githubusercontent.com/49962634/168674137-924506a0-6be0-4cff-b24d-e1160dcc3ae6.mov



## How to use
**Download `install_script.sh`**

**Give execute permission**
```
chmod +x install_script.sh
```

**In the first terminal**
```
./install_script.sh
```
Note: remember to source `.bashrc` under `/pet_ws` every time start a new terminal

Launch the nodes:
```
roslaunch turtle_driver turtle_driver.launch
```

**In the Second terminal**

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
