#!/usr/bin/env python3
"""
This is a node called turtle_driver_ui - client
"""

from secrets import choice
import rospy
import sys
from turtle_driver_ui.srv import DriveTurtleSrv


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
    need_help = input("Turtle command (h for help) > ")
    if need_help == 'h':
        print("circle <radius> => drive in a circle of specified radius")
        print("square <side_length> => drive in a square of specified side length")
        print("custom <p1> <p2> <p3>... => follow these points sequentially")

        moveing_choice = input().split()    # this will give a input list
        print(moveing_choice)

        task = moveing_choice[0]
        radius = None
        length = None
        waypoints = None

        if moveing_choice[0] == 'circle':
            radius = float(moveing_choice[1])
        elif moveing_choice[0] == 'square':
            length = float(moveing_choice[1])
        elif moveing_choice[0] == 'custom':
            # Note: waypoints there are a list of string
            # ['(1,2)', '(2,3)']
            waypoints = moveing_choice[1:]  
        else:
            print("Bad Input")
            sys.exit(1)

        print("Requesting...")
        drive(task, radius, length, waypoints)
