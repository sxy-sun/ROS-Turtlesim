# ROS-Turtlesim

Download `install_script.sh`

Give execute permission
```
$ chmod +x install_script.sh
```

Run the command in the terminal 
```
$ ./install_script.sh
```
Note: remember to source `.bashrc` under `/pet_ws` every time start a new terminal


First terminal
```
$ roscore
```

Second terminal
```
$ rosrun turtlesim turtlesim_node
```

Third terminal
```
$ rosrun turtle_driver turtle_driver_server.py
```

Fourth terminal
```
# to run in a circle with radius = 1
$ rosservice call /turtle_drive "{task: circle, radius: 1}"

# to run in a square with side length = 1
$ rosservice call /turtle_drive "{task: square, length: 1}"

# to follow waypoints
$ rosservice call /turtle_drive "{task: custom, waypoints: {poses: [{pose: {position: {x: 1, y: 1}}}, {pose: {position: {x: 3, y: 5}}}]}}"

```