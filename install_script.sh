mkdir -p ~/pet_ws/src
cd ~/pet_ws/src
git clone https://github.com/sxy-sun/ROS-Turtlesim.git
cd ~/pet_ws
catkin_make

echo "source ~/pet_ws/devel/setup.bash" >> ~/pet_ws/.bashrc
source ~/pet_ws/.bashrc

# Show current path
echo "Current settings:"
printenv | grep ROS
