# ROS-Turtlesim

Download `install_script.sh`

Give execute permission
```
chmod +x install_script.sh
```

Run the command in the terminal 
```
./install_script.sh
```
Note: remember to source `.bashrc` under `/pet_ws` every time start a new terminal


First terminal
```
roscore
```

Second terminal
```
rosrun turtlesim turtlesim_node
```

Third terminal
```
rosrun turtle_drive turtle_drive.py
```

Fourth terminal
```
# to run in a circle with radius = 1
$ rosservice call /turtle_drive "{task: circle, radius: 1}"
```