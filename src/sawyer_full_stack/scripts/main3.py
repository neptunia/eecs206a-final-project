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

import rospy
import tf2_ros
import intera_interface
from moveit_msgs.msg import DisplayTrajectory, RobotState
from sawyer_pykdl import sawyer_kinematics
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander

from numpy import linalg

from intera_interface import gripper as robot_gripper

pos_x1, pos_y1, pos_z1, pos_x2, pos_y2, pos_z2 = [0,0,0,0,0,0]
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
        trans = tfBuffer.lookup_transform('base', "ar_marker_%s"%(tag_number), rospy.Time(0), rospy.Duration(10.0))
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
        trajectory = LinearTrajectory(start_position=current_position, goal_position=target_pos, total_time=7)
    elif task == 'circle':
        target_pos = tag_pos[0]
        target_pos[2] += 0.5
        print("TARGET POSITION:", target_pos)
        trajectory = CircularTrajectory(center_position=target_pos, radius=0.2, total_time=10)

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
        Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
        Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
        Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
        Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
        controller = PIDJointVelocityController(limb, kin, Kp, Ki, Kd, Kw)
    else:
        raise ValueError('Controller {} not recognized'.format(controller_name))
    return controller



def convertcontours(list):
    '''constants'''
    #numbers of segments drawing is sliced into
    num_x = 850
    num_y = 1100
    #side length of square in cm
    scale = 0.0254
    #I hope math works like this
    #also i hope the x-y axis are x is parallel to the computer monitor and y is towards the computer
    #angle of paper from the y axis
    #needs angle of corner of paper from side of paper to make this easier i think, standard numbers
    diagonal_angle = np.arctan(8.5/11)
    #subtract the diagonal angle from the angle of tag1->tag2 vector to get angle of side of paper from y axis
    total_angle = np.arctan(pos_x2 - pos_x1, pos_y2 - pos_y1)
    paper_angle = total_angle - diagonal_angle

    for contour in list:
        for point in contour:
            #transform i hop this is how math works, no clue tho
            #i hope the paper coords were in (x,y)
            #a copy in case i change it in the x before i change the y
            orig = [point[0], point[1]]
            point[0] = pos_x1 + (orig[0] * np.sin(paper_angle) + orig[1] * np.cos(paper_angle)) * scale
            point[1] = pos_y1 + (orig[0] * np.cos(paper_angle) + orig[1] * np.sin(paper_angle)) * scale
    return list

def move(compute_ik, px, py, pz, ox, oy, oz, ow):
    '''also from lab 5, move to make it easier to code on phone on bart oop'''
    # Construct the request
    request = GetPositionIKRequest()
    request.ik_request.group_name = "right_arm"

    # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
    # did above instructions
    link = "right_wrist"

    request.ik_request.ik_link_name = link
    # request.ik_request.attempts = 20
    request.ik_request.pose_stamped.header.frame_id = "base"
    
    # Set the desired orientation for the end effector HERE
    request.ik_request.pose_stamped.pose.position.x = px
    request.ik_request.pose_stamped.pose.position.y = py
    request.ik_request.pose_stamped.pose.position.z = pz     
    request.ik_request.pose_stamped.pose.orientation.x = ox
    request.ik_request.pose_stamped.pose.orientation.y = oy
    request.ik_request.pose_stamped.pose.orientation.z = oz
    request.ik_request.pose_stamped.pose.orientation.w = ow
    
    try:
        # Send the request to the service
        response = compute_ik(request)
        
        # Print the response HERE
        print(response)
        group = MoveGroupCommander("right_arm")

        # Setting position and orientation target
        group.set_pose_target(request.ik_request.pose_stamped)

        # Plan IK
        plan = group.plan()
        
        # Execute IK if safe
        #removed the safety thing, the points are close together so shouldnt need it
        group.execute(plan[1])

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)



def drawcontours(convertedlist):
    '''Yoinked from lab 5, where we had the sawyer arm move to hardcoded locations'''
    '''i know you said lab 8, but it wouldn't load while I was on the bart'''
    # Wait for the IK service to become available

    ik_solver = IK("base", "right_gripper_tip")
    limb = intera_interface.Limb("right")
    kin = sawyer_kinematics("right")
    # Create the function used to call the service
    #compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    # Set up the right gripper
    #right_gripper = robot_gripper.Gripper('right_gripper')

    # Calibrate the gripper (other commands won't work unless you do this first)
    #Uhhh it says it has to do this, maybe just screw the marker holder onto only one side...
    #print('Calibrating...')
    #right_gripper.calibrate()
    #rospy.sleep(2.0)


    '''some constants, will need adjustment probably'''
    z_height = (pos_z1 + pos_z2) / 2
    table_height = z_height
    pen_length = 10 #a guess i did not measure
    lift = 2 #i think its in cm
    #put up and put down pen, deffo needs adjustment
    lift_pen = table_height + pen_length + lift
    drop_pen = table_height + pen_length 
    #this might not be the orientation where the pen is pointing down
    ox = 0
    oy = 1
    oz = 0
    ow = 0


    '''loop over the list of list of points that make up the contours'''
    while not rospy.is_shutdown():
        for contour in convertedlist:
            #go to first point in contour, lifted
            move(ik_solver, contour[0][0], contour[0][1], lift_pen, ox, oy, oz, ow)
            for point in contour:
                #put down pen at first point and keep pen down while going to other points
                move(ik_solver, point[0], point[1], drop_pen, ox, oy, oz, ow)
            #lift pen up at end of contour
            move(ik_solver, contour[-1][0], contour[-1][1], lift_pen, ox, oy, oz, ow)

l = [[(721, 583), (414, 731), (389, 737), (378, 731), (379, 699), (388, 646), (434, 445), (481, 189), (511, 154), (702, 107), (743, 92), (772, 91), (779, 104), (789, 168), (852, 429), (859, 493), (851, 500), (824, 487), (606, 271), (496, 174)], [(852, 501), (852, 514), (850, 516), (850, 518), (849, 519), (848, 522), (844, 527), (844, 528), (838, 534), (837, 534), (835, 536), (833, 536), (832, 537), (829, 537), (828, 538), (827, 538), (824, 540), (822, 540), (821, 541), (820, 541), (817, 543), (815, 543), (814, 544), (812, 544), (811, 545), (809, 545), (808, 546), (806, 546), (804, 548), (803, 548), (802, 549), (801, 549), (800, 550), (799, 550), (798, 551), (797, 551), (796, 552), (795, 552), (792, 554), (790, 554), (787, 556), (785, 556), (784, 557), (783, 557), (782, 558), (780, 558), (777, 560), (775, 560), (774, 561), (772, 561), (771, 562), (770, 562), (767, 564), (765, 564), (762, 566), (760, 566), (759, 567), (757, 567), (754, 569), (752, 569), (750, 571), (748, 571), (747, 572), (746, 572), (745, 573), (744, 573), (743, 574), (742, 574), (739, 576), (737, 576), (736, 577), (734, 577), (733, 578), (731, 578), (729, 580), (728, 580), (727, 581), (726, 581), (725, 582), (722, 583)]]



def main():
    """
    Examples of how to run me:
    python scripts/main.py --help <------This prints out all the help messages
    and describes what each parameter is
    python scripts/main.py -t line -ar_marker 3 -c torque --log
 
    You can also change the rate, timeout if you want
    """

    rospy.init_node('moveit_node')
    
    tuck()
    


    # Lookup the AR tag position.
    tag_pos = [lookup_tag(marker) for marker in [0,6]]
    part1, part2 = tag_pos
    pos_x1, pos_y1, pos_z1 = part1
    pos_x2, pos_y2, pos_z2 = part2
    print(tag_pos)
    drawcontours(l)

if __name__ == "__main__":
    main()
