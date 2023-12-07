#!/usr/bin/env python
"""
Based on main.py from lab7
"""
import sys
import argparse
import numpy as np
import rospkg
import roslaunch

from paths.trajectories import LinearTrajectory, CircularTrajectory
from paths.paths import MotionPath
from paths.path_planner import PathPlanner
from controllers.controllers import ( 
    PIDJointVelocityController, 
    FeedforwardJointVelocityController
)
from utils.utils import *

from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Point, Twist
from trac_ik_python.trac_ik import IK
from intera_interface import gripper as robot_gripper
import rospy
import tf2_ros
import intera_interface
from moveit_msgs.msg import DisplayTrajectory, RobotState
from sawyer_pykdl import sawyer_kinematics
from std_msgs.msg import Float64


def high_tuck():
    """
    Tuck the robot arm to the start position. Use with caution
    """
    if input('Would you like to tuck the arm? (y/n): ') == 'y':
        rospack = rospkg.RosPack()
        path = rospack.get_path('sawyer_full_stack')
        launch_path = path + '/launch/custom_sawyer_tuck.launch'
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_path])
        launch.start()
    else:
        print('Canceled. Not tucking the arm.')


def low_tuck():
    rospack = rospkg.RosPack()
    path = rospack.get_path('sawyer_full_stack')
    launch_path = path + '/launch/lower_sawyer_tuck.launch'
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_path])
    launch.start()

def lookup_tag(tag_number):
    """
    Given an AR tag number, this returns the position of the AR tag in the robot's base frame.
    You can use either this function or try starting the scripts/tag_pub.py script.  More info
    about that script is in that file.  

    Parameters
    ----------
    tag_number : int

    Returns
    -------
    3x' :obj:`numpy.ndarray`
        tag position
    """

    
    # TODO: initialize a tf buffer and listener as in lab 3
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    try:
        # TODO: lookup the transform and save it in trans
        # The rospy.Time(0) is the latest available 
        # The rospy.Duration(10.0) is the amount of time to wait for the transform to be available before throwing an exception
        trans = tfBuffer.lookup_transform('base', f'ar_marker_{tag_number}', rospy.Time(0), rospy.Duration(5.0))
    except Exception as e:
        print(e)
        print("Retrying ...")

    tag_pos = [getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')]
    tag_orientation = [getattr(trans.transform.rotation, dim) for dim in ('x', 'y', 'z', 'w')]
    return np.array(tag_pos), tag_orientation

def is_aligned(actual_pos, expected_pos, tolerance= 0.0001):
    x_error = actual_pos[0] - expected_pos[0]
    y_error = actual_pos[1] - expected_pos[1]
    print("X error is: " + str(x_error))
    print("Y error is: " + str(y_error))
    total_error = x_error**2 + y_error**2
    print("Total Error is: " + str(total_error))
    if total_error <= tolerance:
        print("Aligned!")
        return True
    else:
        print("Not aligned")
        return False

def main():
    """
    Main logic for project
    """

    """
        * Assuming some logic will be implemented here to allow for arguments to get parsed in
    """

    rospy.init_node('moveit_node')
    high_tuck()
    rospy.sleep(3)
    low_tuck()
    rospy.sleep(10)
    pos6, orient6 = lookup_tag('6')
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        inp = input('Ready to scan? (y/n)')
        if inp == 'y':
            pos14, orient14 = lookup_tag('3')
            aligned = is_aligned(pos6, pos14)
        r.sleep()
    print("program finished")

    
if __name__ == "__main__":
    main()