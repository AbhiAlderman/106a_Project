#!/usr/bin/env python
"""
Starter script for 106a lab7. 
Author: Chris Correa
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


def tuck():
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

def get_trajectory(limb, kin, ik_solver, tag_pos, args):
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
    num_way = args.num_way
    task = args.task

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    try:
        trans = tfBuffer.lookup_transform('base', 'right_hand', rospy.Time(0), rospy.Duration(10.0))
    except Exception as e:
        print(e)

    current_position = np.array([getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')])
    print("Current Position:", current_position)

    if task == 'line':
        target_pos = tag_pos[0]
        target_pos[2] += 0.4 #linear path moves to a Z position above AR Tag.
        print("TARGET POSITION:", target_pos)
        trajectory = LinearTrajectory(start_position=current_position, goal_position=target_pos, total_time=4)
    elif task == 'circle':
        target_pos = tag_pos[0]
        target_pos[2] += 0.3
        print("TARGET POSITION:", target_pos)
        trajectory = CircularTrajectory(center_position=target_pos, radius=0.1, total_time=15)

    else:
        raise ValueError('task {} not recognized'.format(task))
    
    path = MotionPath(limb, kin, ik_solver, trajectory)
    return path.to_robot_trajectory(num_way, True)

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

def move_to_pos(limb, gripper, kin, ik_solver, pos):
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
    target_pos = np.copy(pos)
    #might need to make total time a parameter and calculate it??
    trajectory = LinearTrajectory(start_position=pos, goal_position=target_pos, total_time=3)
    path = MotionPath(limb, kin, ik_solver, trajectory)
    robot_trajectory = path.to_robot_trajectory(20, True)
    controller = get_controller('pid', limb, kin)
    controller.execute_path(robot_trajectory)
    return 



def main():
    """
    Examples of how to run me:
    python scripts/main.py --help <------This prints out all the help messages
    and describes what each parameter is
    python scripts/main.py -t line -ar_marker 3 -c torque --log
 
    You can also change the rate, timeout if you want
    """


    parser = argparse.ArgumentParser()
    parser.add_argument('-task', '-t', type=str, default='line', help=
        'Options: line, circle.  Default: line'
    )
    parser.add_argument('-ar_marker', '-ar', nargs='+', help=
        'Which AR marker to use.  Default: 1'
    )
    parser.add_argument('-controller_name', '-c', type=str, default='moveit', 
        help='Options: moveit, open_loop, pid.  Default: moveit'
    )
    parser.add_argument('-rate', type=int, default=200, help="""
        This specifies how many ms between loops.  It is important to use a rate
        and not a regular while loop because you want the loop to refresh at a
        constant rate, otherwise you would have to tune your PD parameters if 
        the loop runs slower / faster.  Default: 200"""
    )
    parser.add_argument('-timeout', type=int, default=None, help=
        """after how many seconds should the controller terminate if it hasn\'t already.  
        Default: None"""
    )
    parser.add_argument('-num_way', type=int, default=50, help=
        'How many waypoints for the :obj:`moveit_msgs.msg.RobotTrajectory`.  Default: 300'
    )
    parser.add_argument('--log', action='store_true', help='plots controller performance')
    args = parser.parse_args()


    rospy.init_node('moveit_node')
    tuck()
    right_gripper = robot_gripper.Gripper('right_gripper')
    right_gripper.open()
    rospy.sleep(1.0)
    right_gripper.close()
    rospy.sleep(1.0)
    
    # this is used for sending commands (velocity, torque, etc) to the robot
    #right_gripper_tip normally
    #stp_022310TP99251_tip for amir robot
    ik_solver = IK("base", "right_gripper_tip") 
    limb = intera_interface.Limb("right")
    #right_gripper = robot_gripper.Gripper('right_gripper')
    print('Calibrating...')
    right_gripper.calibrate()
    rospy.sleep(2.0)
    kin = sawyer_kinematics("right")

    # Lookup the AR tag position.
    print(args.ar_marker)
    tag_pos = [lookup_tag(marker) for marker in args.ar_marker]
    print(tag_pos)
    # Get an appropriate RobotTrajectory for the task (circular, linear, or square)
    # If the controller is a workspace controller, this should return a trajectory where the
    # positions and velocities are workspace positions and velocities.  If the controller
    # is a jointspace or torque controller, it should return a trajectory where the positions
    # and velocities are the positions and velocities of each joint.
    robot_trajectory = get_trajectory(limb, kin, ik_solver, tag_pos, args)

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

    # Move to the trajectory start position
    plan = planner.plan_to_joint_pos(robot_trajectory.joint_trajectory.points[0].positions)
    if args.controller_name != "moveit":
        plan = planner.retime_trajectory(plan, 0.3)
    planner.execute_plan(plan[1])

    if args.controller_name == "moveit":
        try:
            input('Press <Enter> to execute the trajectory using MOVEIT')
        except KeyboardInterrupt:
            sys.exit()
        # Uses MoveIt! to execute the trajectory.
        planner.execute_plan(robot_trajectory)
    else:
        controller = get_controller(args.controller_name, limb, kin)
        try:
            input('Press <Enter> to execute the trajectory using YOUR OWN controller')
        except KeyboardInterrupt:
            sys.exit()
        # execute the path using your own controller.
        done = controller.execute_path(
            robot_trajectory, 
            rate=args.rate, 
            timeout=args.timeout, 
            log=args.log
        )
        if not done:
            print('Failed to move to position')
            sys.exit(0)

    print("We have reached our end")
    
    pickup_object(limb, right_gripper, kin, ik_solver, tag_pos[0])
    print("we picked up")
    right_gripper.open()
    

if __name__ == "__main__":
    main()