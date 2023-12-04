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

from trac_ik_python.trac_ik import IK
from intera_interface import gripper as robot_gripper
import rospy
import tf2_ros
import intera_interface
from moveit_msgs.msg import DisplayTrajectory, RobotState
from sawyer_pykdl import sawyer_kinematics


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
        trans = tfBuffer.lookup_transform('base', f'ar_marker_{tag_number}', rospy.Time(0), rospy.Duration(10.0))
    except Exception as e:
        print(e)
        print("Retrying ...")

    tag_pos = [getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')]
    return np.array(tag_pos)

def get_trajectory(limb, kin, ik_solver, tag_pos, z_adjustment, path_time):
    """
    Returns an appropriate robot trajectory for the specified task.  You should 
    be implementing the path functions in paths.py and call them here
    
    Parameters
    ----------
    task : string
        name of the task.  Options: line, circle, square
    tag_pos : 3x' :obj:`numpy.ndarray`
        
    Returns
    -------
    :obj:`moveit_msgs.msg.RobotTrajectory`
    """

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    try:
        trans = tfBuffer.lookup_transform('base', 'right_hand', rospy.Time(0), rospy.Duration(10.0))
    except Exception as e:
        print(e)

    current_position = np.array([getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')])
    print("Current Position:", current_position)

    target_pos = np.copy(tag_pos[0])
    target_pos[2] += z_adjustment #linear path moves to a Z position above AR Tag.
    target_pos[2] = max(target_pos[2], 0)
    print("TARGET POSITION:", target_pos)
    trajectory = LinearTrajectory(start_position=current_position, goal_position=target_pos, total_time=path_time)

    path = MotionPath(limb, kin, ik_solver, trajectory)
    return path.to_robot_trajectory(20, True)

def get_controller(controller_name, limb, kin):
    """
    Gets the correct controller from controllers.py

    Parameters
    ----------
    controller_name : string

    Returns
    -------
    :obj:`Controller`
    """
    if controller_name == 'open_loop':
        controller = FeedforwardJointVelocityController(limb, kin)
    elif controller_name == 'pid':
        Kp = 0.7 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
        Kd = 0.02 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
        Ki = 0.02 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
        Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
        controller = PIDJointVelocityController(limb, kin, Kp, Ki, Kd, Kw)
    else:
        raise ValueError('Controller {} not recognized'.format(controller_name))
    return controller


def pickup_object(limb, gripper, kin, ik_solver, tag_pos):
    """
    Moves the robot arm down to the AR tag, closes the gripper to pick up the object,
    and then lifts it.

    Parameters
    ----------
    limb : intera_interface.Limb
        The robot's limb interface.
    gripper : intera_interface.Gripper
        The robot's gripper interface.
    kin : sawyer_kinematics
        Kinematics object for the Sawyer robot.
    ik_solver : IK
        Inverse Kinematics solver.
    tag_pos : numpy.ndarray
        Position of the AR tag.
    """
    target_pos = np.copy(tag_pos)
    target_pos[2] -= 0.2  # lowers by 20cm

    lower_trajectory = LinearTrajectory(start_position=tag_pos, goal_position=target_pos, total_time=2)
    lower_path = MotionPath(limb, kin, ik_solver, lower_trajectory)
    lower_robot_traj = lower_path.to_robot_trajectory(20, True)
    controller = get_controller('pid', limb, kin)
    controller.execute_path(lower_robot_traj)
    print("GRIPPER IS READY == " + str(gripper.is_ready()))
    # Close the gripper to pick up the object
    if gripper.is_ready():
        gripper.open()
        rospy.sleep(2.0)
        gripper.close()
        rospy.sleep(2.0)
    # Lift the object
    lift_pos = np.copy(tag_pos)
    lift_pos[2] += 0.2  # lifts by 20cm
    #might need to change the total time to make it slower/faster
    lift_trajectory = LinearTrajectory(start_position=target_pos, goal_position=lift_pos, total_time=2)
    lift_path = MotionPath(limb, kin, ik_solver, lift_trajectory)
    lift_robot_traj = lift_path.to_robot_trajectory(20, True)
    controller.execute_path(lift_robot_traj)

    return True

def move_to_pos(limb, gripper, kin, ik_solver, pos, z_adjustment, path_time):
    """
    Move robot arm to the desired position
    
    Parameters
    ----------
    limb : intera_interface.Limb
        The robot's limb interface.
    gripper : intera_interface.Gripper
        The robot's gripper interface.
    kin : sawyer_kinematics
        Kinematics object for the Sawyer robot.
    ik_solver : IK
        Inverse Kinematics solver.
    tag_pos : numpy.ndarray
        Position of the AR tag.
    """
    robot_trajectory = get_trajectory(limb, kin, ik_solver, pos, z_adjustment, path_time)

    # This is a wrapper around MoveIt! for you to use.  We use MoveIt! to go to the start position
    # of the trajectory
    planner = PathPlanner('right_arm')

    # By publishing the trajectory to the move_group/display_planned_path topic, you should 
    # be able to view it in RViz.  You will have to click the "loop animation" setting in 
    # the planned path section of MoveIt! in the menu on the left side of the screen.
    pub = rospy.Publisher('move_group/display_planned_path', DisplayTrajectory, queue_size=10)
    disp_traj = DisplayTrajectory()
    disp_traj.trajectory.append(robot_trajectory)
    disp_traj.trajectory_start = RobotState()
    pub.publish(disp_traj)

    plan = planner.plan_to_joint_pos(robot_trajectory.joint_trajectory.points[0].positions)
    controller = get_controller("pid", limb, kin)
    done = controller.execute_path(
            robot_trajectory
        )
    if not done:
        print('Failed to move to position')
        sys.exit(0)

    print("REACHED END")
    return pos

def main():
    """
    Main logic for project
    """

    """
        * Assuming some logic will be implemented here to allow for arguments to get parsed in
    """

    rospy.init_node('moveit_node')
    high_tuck()
    gripper = robot_gripper.Gripper('right_gripper')
    gripper.open()
    rospy.sleep(1.0)
    gripper.close()
    rospy.sleep(1.0)

    """
        Logic for AR tag position 
    """
    # this is used for sending commands (velocity, torque, etc) to the robot
    #right_gripper_tip normally
    #stp_022310TP99251_tip for amir robot
    ik_solver = IK("base", "right_gripper_tip")
    limb = intera_interface.Limb("right")
    #right_gripper = robot_gripper.Gripper('right_gripper')
    print('Calibrating...')
    gripper.calibrate()
    rospy.sleep(2.0)
    kin = sawyer_kinematics("right")
    low_tuck()
    rospy.sleep(2.0)
    goal_pos = [lookup_tag('0')]
    pos_7 = [lookup_tag('7')]
    pos_8 = [lookup_tag('8')]
    pos_9 = [lookup_tag('9')]
    tags = ['7', '8', '9']
    for tag in tags:
        tag_pos = [lookup_tag(tag)]
        print("PICKING POSITION " + str(tag_pos))
        above_tag = move_to_pos(limb, gripper, kin, ik_solver, tag_pos, 0.5, 5)
        gripper.open()
        rospy.sleep(0.5)
        grab_tag = move_to_pos(limb, gripper, kin, ik_solver, tag_pos, 0, 5)
        gripper.close()
        rospy.sleep(0.5)
        print("GOAL POSITION " + str(goal_pos))
        above_goal = move_to_pos(limb, gripper, kin, ik_solver, goal_pos, 0.5, 5)
        go_goal = move_to_pos(limb, gripper, kin, ik_solver, goal_pos, 0.23, 5)
        gripper.open()
        rospy.sleep(0.5)
        move_to_pos(limb, gripper, kin, ik_solver, goal_pos, 0.5, 5)
        low_tuck()
        rospy.sleep(2)
        goal_pos = [lookup_tag(tag)]
    
if __name__ == "__main__":
    main()
