#!/usr/bin/env python3
"""
This is a node called turtle_driver_ui - client
"""

from secrets import choice
import rospy
import time
import sys
from turtle_driver_ui.srv import DriveTurtleSrv
from nav_msgs.msg import Path
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped 



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
    # print out three commonds
    while True:
        switcher = input("Turtle command (h for help) > ")
        if switcher == 'h':
            print("circle <radius> => drive in a circle of specified radius")
            print("square <side_length> => drive in a square of specified side length")
            print("custom <p1> <p2> <p3>... => follow these points sequentially")
            continue
        else:
            switcher = switcher.split()    # this will give a input list
            print(switcher)

            task = switcher[0]
            radius = None
            length = None
            waypoints = Path()

            if switcher[0] == 'circle':
                radius = float(switcher[1])
            elif switcher[0] == 'square':
                length = float(switcher[1])
            elif switcher[0] == 'custom':
                """
                Note: waypoints there are a list of string
                x = ['(1,2)', '(2,3)']
                float(x[0][1]) = 1.0
                float(x[0][3]) = 2.0
                """
                waypoints_input = switcher[1:]  
                for i in range(len(waypoints_input)):
                    waypoint = PoseStamped()
                    waypoint.pose.position.x = float(waypoints_input[i][1])
                    waypoint.pose.position.y = float(waypoints_input[i][3])
                    
                    waypoints.poses.append(waypoint)
            else:
                print("Bad Input")
                sys.exit(1)

            print("Requesting...")
            drive(task, radius, length, waypoints)