#!/usr/bin/env python
"""
Script for trajectory execution for drawing
"""
import sys
import argparse
import numpy as np
import rospkg
import roslaunch
import math

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

def get_trajectory(limb, kin, ik_solver, tag_pos):
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

    target_pos = np.array(tag_pos)
    print("TARGET POSITION:", target_pos)
    target_pos[2] += 0.4 - 13.335/100 #linear path moves to a Z position above AR Tag.
    print(type(target_pos))
    print("TARGET POSITION:", target_pos)
    trajectory = LinearTrajectory(start_position=current_position, goal_position=target_pos, total_time=1)

    path = MotionPath(limb, kin, ik_solver, trajectory)
    print(path)
    return path.to_robot_trajectory(2, True)

def get_trajectory_slow(limb, kin, ik_solver, tag_pos):


    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    try:
        trans = tfBuffer.lookup_transform('base', 'right_hand', rospy.Time(0), rospy.Duration(10.0))
    except Exception as e:
        print(e)

    current_position = np.array([getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')])
    print("Current Position:", current_position)

    target_pos = np.array(tag_pos)
    print("TARGET POSITION:", target_pos)
    target_pos[2] += 0.4 - 13.335/100 #linear path moves to a Z position above AR Tag.
    print(type(target_pos))
    print("TARGET POSITION:", target_pos)
    trajectory = LinearTrajectory(start_position=current_position, goal_position=target_pos, total_time=5)

    path = MotionPath(limb, kin, ik_solver, trajectory)
    print(path)
    return path.to_robot_trajectory(10, True)

def get_trajectory_above(limb, kin, ik_solver, tag_pos):


    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    try:
        trans = tfBuffer.lookup_transform('base', 'right_hand', rospy.Time(0), rospy.Duration(10.0))
    except Exception as e:
        print(e)

    current_position = np.array([getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')])
    print("Current Position:", current_position)

    target_pos = np.array(tag_pos)
    print("TARGET POSITION:", target_pos)
    target_pos[2] += 0.6 #linear path moves to a Z position above AR Tag.
    print(type(target_pos))
    print("TARGET POSITION:", target_pos)
    trajectory = LinearTrajectory(start_position=current_position, goal_position=target_pos, total_time=5)

    path = MotionPath(limb, kin, ik_solver, trajectory)
    print(path)
    return path.to_robot_trajectory(10, True)




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


#def convert_to_real_world_coordinates(rel_point, tag_pos):

'''
def convert_to_real_world_coordinates(rel_point, tag_pos):
    # Extract the corners of the paper from tag_pos
    in2m = 0.0254 #1 inch in meters
    x1, y1 = tag_pos[0][:2]
    x2, y2 = tag_pos[1][:2]
    xCen = (x1 + x2)/2
    yCen = (y1 + y2)/2
    
    # Real-world dimensions of the paper (8.5x11 inches)
    paper_width = 11.0 * in2m
    paper_height = 8.5 * in2m
    ar_half = 0.03
    
    # Calculate the diagonal distance between the corners (real-world space)
    #real_diagonal = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    
    # Calculate the diagonal length of the paper based on its real dimensions
    #paper_diagonal = math.sqrt(paper_width ** 2 + paper_height ** 2)
    
    # Scale factor to adjust the paper's actual size
    #scale_factor = real_diagonal / paper_diagonal
    
    # Calculate the real width and height of the paper (considering the aspect ratio)
    #real_width = paper_width * scale_factor
    #real_height = paper_height * scale_factor* in2m
    real_width = paper_width
    real_height = paper_height
    
    # Calculate the rotation angle (in radians)
    paper_base_angle = math.atan2(0.14+0.03, 0.108+0.03)
    frame_rotate_offset = math.pi/2
    theta = math.atan2(y2 - y1, x2 - x1) - paper_base_angle + frame_rotate_offset
    
    # Extract the relative point coordinates
    rel_x, rel_y = rel_point
    
    # Calculate the dimensions of the relative coordinate system
    rel_width = 1100
    rel_height = 850
    #13.335
    # Calculate the scaling factors for x and y
    scale_x = real_width / rel_width
    scale_y = real_height / rel_height
    
    # Step 1: Translate the relative po.cos(-theta) - rel_y_centered * math.sin(-theta)int to the origin
    rel_x_centered = rel_x - rel_width / 2
    rel_y_centered = rel_y - rel_height / 2
    
    # Step 2: Rotate the point by -theta to undo the rotation
    rotated_x = rel_x_centered * math.cos(-theta) - rel_y_centered * math.sin(-theta)
    rotated_y = rel_x_centered * math.sin(-theta) + rel_y_centered * math.cos(-theta)
    
    # Step 3: Scale the rotated coordinates to the real-world size
    real_x = xCen + rotated_x * scale_x
    real_y = yCen + rotated_y * scale_y
    
    return [real_x, real_y]

def convert_to_real_world_coordinates(rel_point, tag_pos):
    b = tag_pos[0][:2]
    a = tag_pos[1][:2]
    POINT_TO_MAP = rel_point

    vec_ab = np.array([b[0]-a[0], b[1]-a[1]])
    s = np.linalg.norm(vec_ab)
    #s_long = s / 13.901 * 11.0
    #s_short = s / 13.901 * 8.5
    s_long = s / 17.220214217 * 11.0 #6cm added to account for the AR positionS (2 half distances)
    s_short = s / 17.220214217 * 8.5 #ditto

    inv_tan = 0.698659825 #inverse tan of 8.5in + 6cm / 11in + 6cm
    #0.657889 # inverse tan of 8.5/11

    rot_matr = np.array([[np.cos(inv_tan), -np.sin(inv_tan)],[np.sin(inv_tan),np.cos(inv_tan)]])
    rotated = (rot_matr) @ (vec_ab / s)
    rotated_2 = np.array([[0, -1],[1,0]]) @ rotated

    # distance along the red line
    ar_offset = (0.10737290909 * s_long * rotated - 0.13895317647 * s_short * rotated_2)
    MAPPED_POINT  = a + (POINT_TO_MAP[0]/1100. * s_long) * rotated - (POINT_TO_MAP[1]/850 * s_short) * rotated_2  + ar_offset
    
    return MAPPED_POINT
    '''

def convert_to_real_world_coordinates(rel_point, tag_pos):
    # Extract the corners of the paper from tag_pos
    in2m = 0.0254 #1 inch in meters
    x1, y1 = tag_pos[0][:2]
    x2, y2 = tag_pos[1][:2]
    xCen = (x1 + x2)/2
    yCen = (y1 + y2)/2
    
    # Real-world dimensions of the paper (8.5x11 inches)
    paper_width = 11.0 * in2m
    paper_height = 8.5 * in2m
    ar_half = 0.03
    
    # Calculate the diagonal distance between the corners (real-world space)
    #real_diagonal = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    
    # Calculate the diagonal length of the paper based on its real dimensions
    #paper_diagonal = math.sqrt(paper_width ** 2 + paper_height ** 2)
    
    # Scale factor to adjust the paper's actual size
    #scale_factor = real_diagonal / paper_diagonal
    
    # Calculate the real width and height of the paper (considering the aspect ratio)
    #real_width = paper_width * scale_factor
    #real_height = paper_height * scale_factor* in2m
    real_width = paper_width
    real_height = paper_height
    
    # Calculate the rotation angle (in radians)
    theta = math.atan2(y2 - y1, x2 - x1) - math.atan2(0.14+0.03, 0.108+0.03)
    
    # Extract the relative point coordinates
    rel_x, rel_y = rel_point
    
    # Calculate the dimensions of the relative coordinate system
    rel_width = 1100
    rel_height = 850
    #13.335
    # Calculate the scaling factors for x and y
    scale_x = real_width / rel_width
    scale_y = real_height / rel_height
    
    # Step 1: Translate the relative point to the origin
    rel_x_centered = rel_x - rel_height / 2
    rel_y_centered = rel_y - rel_width / 2
    
    # Step 2: Rotate the point by -theta to undo the rotation
    rotated_x = rel_x_centered * math.cos(-theta) - rel_y_centered * math.sin(-theta)
    rotated_y = rel_x_centered * math.sin(-theta) + rel_y_centered * math.cos(-theta)
    
    # Step 3: Scale the rotated coordinates to the real-world size
    real_x = xCen + rotated_x * scale_x
    real_y = yCen + rotated_y * scale_y
    
    return [real_x, real_y]


def convertcontours(list, tag_pos):
    '''constants'''
    
    out = []
    for point in list:
        #transform i hop this is how math works, no clue tho
        #i hope the paper coords were in (x,y)
        #a copy in case i change it in the x before i change the y
        new = convert_to_real_world_coordinates(point, tag_pos)
        new += [tag_pos[0][2]/2 + tag_pos[1][2]/2]
        out.append(new)
    return out


def do_stuff(pose, compute_ik):


    request = GetPositionIKRequest()
    request.ik_request.group_name = "right_arm"

    # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
    link = "right_gripper_tip"

    request.ik_request.ik_link_name = link
    # request.ik_request.attempts = 20
    request.ik_request.pose_stamped.header.frame_id = "base"
    
    # Set the desired orientation for the end effector HERE
    request.ik_request.pose_stamped.pose.position.x = pose[0]
    request.ik_request.pose_stamped.pose.position.y = pose[1]
    request.ik_request.pose_stamped.pose.position.z = pose[2]
    request.ik_request.pose_stamped.pose.orientation.x = 0.0
    request.ik_request.pose_stamped.pose.orientation.y = 1.0
    request.ik_request.pose_stamped.pose.orientation.z = 0.0
    request.ik_request.pose_stamped.pose.orientation.w = 0.0
    
    response = compute_ik(request)
        
    group = MoveGroupCommander("right_arm")

    group.set_pose_target(request.ik_request.pose_stamped)

    plan = group.plan()
    group.execute(plan[1])
#l = [[(721, 583), (414, 731), (389, 737), (378, 731), (379, 699), (388, 646), (434, 445), (481, 189), (511, 154), (702, 107), (743, 92), (772, 91), (779, 104), (789, 168), (852, 429), (859, 493), (851, 500), (824, 487), (606, 271), (496, 174)], [(852, 501), (852, 514), (850, 516), (850, 518), (849, 519), (848, 522), (844, 527), (844, 528), (838, 534), (837, 534), (835, 536), (833, 536), (832, 537), (829, 537), (828, 538), (827, 538), (824, 540), (822, 540), (821, 541), (820, 541), (817, 543), (815, 543), (814, 544), (812, 544), (811, 545), (809, 545), (808, 546), (806, 546), (804, 548), (803, 548), (802, 549), (801, 549), (800, 550), (799, 550), (798, 551), (797, 551), (796, 552), (795, 552), (792, 554), (790, 554), (787, 556), (785, 556), (784, 557), (783, 557), (782, 558), (780, 558), (777, 560), (775, 560), (774, 561), (772, 561), (771, 562), (770, 562), (767, 564), (765, 564), (762, 566), (760, 566), (759, 567), (757, 567), (754, 569), (752, 569), (750, 571), (748, 571), (747, 572), (746, 572), (745, 573), (744, 573), (743, 574), (742, 574), (739, 576), (737, 576), (736, 577), (734, 577), (733, 578), (731, 578), (729, 580), (728, 580), (727, 581), (726, 581), (725, 582), (722, 583)]]

#l = [[(392, 892), (359, 851), (289, 653), (185, 603), (105, 524), (274, 387), (270, 250), (249, 202)], [(271, 252), (299, 263), (417, 401), (439, 415), (456, 409), (598, 289), (635, 276), (652, 253)], [(405, 166), (402, 171), (402, 174), (401, 175), (401, 188), (402, 189), (402, 190), (401, 191), (401, 203), (402, 204), (402, 208)], [(482, 641), (484, 648), (473, 664), (400, 873), (391, 883)], [(484, 649), (607, 683), (647, 685), (635, 650), (530, 535), (536, 502), (630, 318), (636, 276)], [(279, 641), (279, 640), (280, 639), (280, 627), (279, 626), (279, 613)]]

l = [[(577, 389), (487, 356), (392, 348), (387, 323), (366, 313), (292, 312), (277, 335), (240, 343), (191, 396), (182, 458), (209, 548), (270, 605), (270, 667), (290, 695), (369, 718), (476, 719), (503, 685), (512, 614), (271, 605)], [(391, 348), (382, 422), (372, 443), (352, 454), (313, 454), (288, 445), (276, 424), (277, 336)], [(578, 390), (603, 409), (606, 430), (602, 440), (588, 450), (528, 459), (510, 475), (503, 496), (511, 512), (564, 529), (589, 547), (599, 562), (602, 599), (584, 610), (512, 613)]]

def main():


    rospy.wait_for_service('compute_ik')
    # Create the function used to call the service
    
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)


    
    rospy.init_node('moveit_node')
    
    tuck()
    
    # this is used for sending commands (velocity, torque, etc) to the robot
    ik_solver = IK("base", "right_gripper_tip")
    limb = intera_interface.Limb("right")
    kin = sawyer_kinematics("right")

    # Lookup the AR tag position.
    tag_pos = [lookup_tag(marker) for marker in [0,6]]

    print(tag_pos)

    # [array([ 0.84005421, -0.03714028, -0.24398073]), array([ 0.5088102 ,  0.30288555, -0.21465782])]

    part1, part2 = tag_pos
    pos_x1, pos_y1, pos_z1 = part1
    pos_x2, pos_y2, pos_z2 = part2

    # Get an appropriate RobotTrajectory for the task (circular, linear, or square)
    # If the controller is a workspace controller, this should return a trajectory where the
    # positions and velocities are workspace positions and velocities.  If the controller
    # is a jointspace or torque controller, it should return a trajectory where the positions
    # and velocities are the positions and velocities of each joint.

    for contour in l:
        newarc = convertcontours(contour, tag_pos)
        
        #go above the first -point in the arc
        np = newarc[0][:]
        np[2] += 0.2
        do_stuff(np, compute_ik)
        np = newarc[0][:]
        np[2] += 0.13
        do_stuff(np, compute_ik)
        for point in newarc[1:]:
            np = point[:]
            np[2]  += 0.13
            do_stuff(np, compute_ik)
        np = newarc[-1][:]
        np[2] += 0.2
        do_stuff(np, compute_ik)

if __name__ == "__main__":
    main()
