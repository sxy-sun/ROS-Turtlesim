#!/usr/bin/env python3
"""
This is a node called turtle_driver_ui - client
"""

from secrets import choice
import rospy
import sys
from turtle_driver_ui.srv import DriveCircle, DriveSquare, Waypoints


def drive_circle(radius):
    rospy.wait_for_service('turtle_drive')
    try:
        turtle_drive = rospy.ServiceProxy('turtle_drive', DriveCircle)
        indicator = turtle_drive(radius)
        if indicator:
            print("Manuever was successful")
    except rospy.ServiceException as e:
        print("Manuever has failed")


def drive_square(side_length):
    rospy.wait_for_service('turtle_drive')
    try:
        turtle_drive = rospy.ServiceProxy('turtle_drive', DriveSquare)
        indicator = turtle_drive(side_length)
        if indicator:
            print("Manuever was successful")
    except rospy.ServiceException as e:
        print("Manuever has failed")


def waypoint_following(waypoints):
    rospy.wait_for_service('turtle_drive')
    try:
        turtle_drive = rospy.ServiceProxy('turtle_drive', Waypoints)
        indicator = turtle_drive(waypoints)
        if indicator:
            print("Manuever was successful")
    except rospy.ServiceException as e:
        print("Manuever has failed")


if __name__ == "__main__":
    need_help = input("Turtle command (h for help) > ")
    if need_help == 'h':
        print("circle <radius> => drive in a circle of specified radius")
        print("square <side_length> => drive in a square of specified side length")
        print("custom <p1> <p2> <p3>... => follow these points sequentially")

        moveing_choice = input().split()    # this will give a input list
        print(moveing_choice)
        if moveing_choice[0] == 'circle':
            radius = float(moveing_choice[1])
            print("Requesting...")
            print("%s", drive_circle(radius))
        elif moveing_choice[0] == 'square':
            side_length = float(moveing_choice[1])
            print("Requesting...")
            print("%s", drive_square(side_length))
        elif moveing_choice[0] == 'custom':
            waypoints = moveing_choice[1:]    
            print("Requesting...")
            print("%s", waypoint_following(waypoints))  
        else:
            print("Bad Input")
            sys.exit(1)
