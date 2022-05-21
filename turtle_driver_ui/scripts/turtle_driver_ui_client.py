#!/usr/bin/env python3

from secrets import choice
import rospy
import time
import sys
from turtle_driver_ui.srv import DriveTurtleSrv
from nav_msgs.msg import Path
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped 
import re



def drive(task, radius, side_length, waypoints):
    rospy.wait_for_service('turtle_drive')
    try:
        turtle_drive = rospy.ServiceProxy('turtle_drive', DriveTurtleSrv)
        indicator = turtle_drive(task, radius, side_length, waypoints)
        if indicator:
            print("Manuever was successful")
    except rospy.ServiceException as e:
        print("Manuever has failed")
        print(e)


if __name__ == "__main__":
    rospy.init_node('turtle_drive_ui_node')
    custom_path_publisher = rospy.Publisher('custom_path', Path, queue_size=20)

    while not rospy.is_shutdown():
        switcher = input("Turtle command (h for help) > ")
        if switcher == 'h':
            print("circle <radius> => drive in a circle of specified radius")
            print("square <side_length> => drive in a square of specified side length")
            print("custom <(x1,y1)> <(x2,y2)> <(x3,y3)>... => follow these points sequentially")
        else:
            try:
                switcher = switcher.split()    # this will give a input list

                task = switcher[0]
                radius = None
                length = None
                waypoints = Path()

                if switcher[0] == 'circle':
                    radius = float(switcher[1])
                elif switcher[0] == 'square':
                    length = float(switcher[1])
                elif switcher[0] == 'custom':
                    # Note: waypoints there are a list of string
                    # x = ['(1,2)', '(2,3)']
                    #   float(x[0][1]) = 1.0
                    #   float(x[0][3]) = 2.0
                    
                    waypoints_input = switcher[1:]  
                    non_decimal = re.compile(r'[^\d,.]+')
                    
                    for i in range(len(waypoints_input)):
                        waypoint_input = non_decimal.sub('', waypoints_input[i])
                        waypoint_input_x = waypoint_input.split(',')[0]
                        waypoint_input_y = waypoint_input.split(',')[1]

                        waypoint = PoseStamped()
                        waypoint.header.frame_id = "turtle_waypoints_frame"
                        waypoint.pose.position.x = float(waypoint_input_x)
                        waypoint.pose.position.y = float(waypoint_input_y)      
                        waypoints.poses.append(waypoint)
                        custom_path_publisher.publish(waypoints)
                elif switcher[0] == 'reset':
                    pass        
                else:
                    print("Missing Task Name")
                    break

                print("Requesting...")
                drive(task, radius, length, waypoints)
            except rospy.ServiceException as e:
                print("Manuever has failed")
                print(e)