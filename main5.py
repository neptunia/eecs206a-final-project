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

from copy import deepcopy
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
    print(paper_base_angle)
    theta = math.atan2(y2 - y1, x2 - x1) - 0.5 * paper_base_angle# + frame_rotate_offset
    print(theta)
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
    rel_y_centered = rel_y - rel_width / 2
    
    # Step 2: Rotate the point by -theta to undo the rotation
    rotated_x = rel_x_centered * math.cos(theta) - rel_y_centered * math.sin(theta)
    rotated_y = rel_x_centered * math.sin(theta) + rel_y_centered * math.cos(theta)
    
    # Step 3: Scale the rotated coordinates to the real-world size
    real_x = xCen + rotated_x * scale_x
    real_y = yCen + rotated_y * scale_y
    
    return [real_x, real_y]
    '''

def convert_to_real_world_coordinates_2(rel_point, tag_pos):
    a = tag_pos[0][:2]
    b = tag_pos[1][:2]
    POINT_TO_MAP = (rel_point[0], rel_point[1])

    vec_ab = np.array([b[0]-a[0], b[1]-a[1]])
    s = np.linalg.norm(vec_ab)
    #s_long = s / 13.901 * 11.0
    #s_short = s / 13.901 * 8.5
    s_long = s / 17.2199328686 * 11.0 #6cm added to account for the AR positionS (2 half distances)
    s_short = s / 17.2199328686 * 8.5 #ditto

    inv_tan = 0.682560536 #inverse tan of 8.5in + 6cm / 11in + 6cm
    #0.657889 # inverse tan of 8.5/11

    # -0.225
    rot_matr = np.array([[np.cos(inv_tan), -np.sin(inv_tan)],[np.sin(inv_tan),np.cos(inv_tan)]])
    rotated = (rot_matr) @ (vec_ab / s)
    rotated_2 = np.array([[0, -1],[1,0]]) @ rotated

    rotated_2 = -(rotated_2)

    print(rotated)
    print(rotated_2)

    # distance along the red line
    ar_offset = (0.1 * s_long * (11+6/2.54)/11. * rotated) + 2 * (0.1 * s_short * (8.5+6/2.54)/8.5 * rotated_2)
    MAPPED_POINT  = a + (POINT_TO_MAP[0]/850. * s_short) * rotated_2 + (POINT_TO_MAP[1]/1100. * s_long) * rotated + ar_offset
    
    return MAPPED_POINT.tolist()

def force_straight_angle(rel_point, tag_pos):
    a = tag_pos[0][:2]
    b = tag_pos[1][:2]
    if (a[0]-b[0])**2 > (a[1]-b[1])**2:
        # x is the long axis
        POINT_TO_MAP = (rel_point[1], 850-rel_point[0])
        mapped = [a[0] + POINT_TO_MAP[0]/1100. * (b[0]-a[0]) * (11/(11 + 2*1.1811)) + (b[0]-a[0])*(1.1811/(11 + 2*1.1811)),
                  a[1] + POINT_TO_MAP[1]/850. * (b[1]-a[1]) * (8.5/(8.5 + 2*1.1811)) + (b[1]-a[1])*(1.1811/(8.5 + 2*1.1811))]
    else:
        # y is the long axis
        POINT_TO_MAP = (rel_point[0], rel_point[1])
        mapped = [a[0] + POINT_TO_MAP[0]/850. * (b[0]-a[0]) * (8.5/(8.5 + 2*1.1811)) + (b[0]-a[0])*(1.1811/(8.5 + 2*1.1811)),
                  a[1] + POINT_TO_MAP[1]/1100. * (b[1]-a[1]) * (11/(11 + 2*1.1811)) + (b[1]-a[1])*(1.1811/(11 + 2*1.1811))]
    return mapped


'''
def convert_to_real_world_coordinates(rel_point, tag_pos):
    b = tag_pos[0][:2]
    a = tag_pos[1][:2]
    POINT_TO_MAP = (rel_point[0], 1100-rel_point[1])

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
    ar_offset = (0.13895317647 * s_long * rotated + 0.10737290909 * s_short * rotated_2)
    MAPPED_POINT  = a + (POINT_TO_MAP[0]/850. * s_short) * rotated - (POINT_TO_MAP[1]/1100 * s_long) * rotated_2 + 2 * ar_offset
    
    return MAPPED_POINT.tolist()
'''
def convertcontours(list2, tag_pos):
    '''constants'''
    
    out = []
    for point in list2:
        #transform i hop this is how math works, no clue tho
        #i hope the paper coords were in (x,y)
        #a copy in case i change it in the x before i change the y
        new = convert_to_real_world_coordinates_2(point, tag_pos)
        new += [tag_pos[0][2]/2 + tag_pos[1][2]/2]
        out.append(new)
    return out


def create_pose(pose, base_pose):
    newpose = deepcopy(base_pose)
    newpose.position.x = pose[0]
    newpose.position.y = pose[1]
    newpose.position.z = pose[2]
    newpose.orientation.x = 0.0
    newpose.orientation.y = 1.0
    newpose.orientation.z = 0.0
    newpose.orientation.w = 0.0
    return newpose



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

#l = [[(577, 389), (487, 356), (392, 348), (387, 323), (366, 313), (292, 312), (277, 335), (240, 343), (191, 396), (182, 458), (209, 548), (270, 605), (270, 667), (290, 695), (369, 718), (476, 719), (503, 685), (512, 614), (271, 605)], [(391, 348), (382, 422), (372, 443), (352, 454), (313, 454), (288, 445), (276, 424), (277, 336)], [(578, 390), (603, 409), (606, 430), (602, 440), (588, 450), (528, 459), (510, 475), (503, 496), (511, 512), (564, 529), (589, 547), (599, 562), (602, 599), (584, 610), (512, 613)]]

#l = [[(627, 538), (437, 513), (382, 563), (372, 588), (327, 620), (272, 623), (149, 601), (108, 508), (64, 457), (35, 455)], [(65, 457), (131, 428), (173, 382), (218, 383), (338, 401), (379, 447), (370, 469), (337, 507), (299, 511), (184, 493)], [(380, 445), (399, 439), (427, 412), (456, 405), (628, 417), (664, 434), (693, 491), (739, 538), (677, 588), (640, 643), (624, 648), (422, 632), (404, 623), (374, 588)], [(740, 538), (751, 539), (752, 540), (759, 540), (760, 541), (784, 541), (785, 542), (789, 542), (794, 544), (799, 544)]]

#l = [[(577, 389), (487, 356), (392, 348), (387, 323), (366, 313), (292, 312), (277, 335), (240, 343), (191, 396), (182, 458), (209, 548), (270, 605), (270, 667), (290, 695), (369, 718), (476, 719), (503, 685), (512, 614), (271, 605)], [(391, 348), (382, 422), (372, 443), (352, 454), (313, 454), (288, 445), (276, 424), (277, 336)], [(578, 390), (603, 409), (606, 430), (602, 440), (588, 450), (528, 459), (510, 475), (503, 496), (511, 512), (564, 529), (589, 547), (599, 562), (602, 599), (584, 610), (512, 613)]]
#l = [[(385, 709), (397, 706), (398, 691), (387, 685), (355, 701), (331, 702), (316, 697), (303, 686), (262, 684), (250, 679), (251, 662), (261, 650), (260, 637), (250, 632), (252, 621), (266, 605), (269, 591), (253, 588), (248, 591)], [(254, 587), (263, 562), (258, 558), (249, 558), (245, 554), (241, 555), (246, 553), (255, 532), (255, 528), (252, 525), (246, 525), (240, 522), (249, 498), (229, 497), (200, 504)], [(168, 465), (182, 452), (211, 437), (266, 419), (283, 417), (287, 414)], [(302, 369), (332, 364), (336, 347), (348, 333), (360, 325), (373, 323), (427, 356), (426, 367), (411, 380), (407, 392), (392, 398), (382, 396), (378, 382), (369, 373), (354, 366), (333, 364)], [(385, 399), (382, 406), (381, 416), (379, 418), (382, 423), (398, 425), (403, 429), (408, 430), (412, 435), (413, 441), (416, 443), (436, 442), (448, 437), (451, 430), (450, 418), (447, 409), (446, 384), (434, 386), (422, 391), (408, 393)], [(378, 432), (377, 417), (357, 412), (288, 414), (287, 394), (301, 370), (295, 365), (245, 364), (243, 352), (224, 343), (170, 335), (139, 351), (118, 394), (121, 431), (152, 462), (184, 408), (211, 382), (244, 364)], [(250, 497), (344, 494), (331, 472), (398, 475), (412, 497), (454, 474), (503, 413), (509, 435), (551, 428), (517, 345), (501, 245), (550, 186), (613, 168), (682, 180), (736, 224), (729, 466), (720, 487), (700, 491)], [(669, 489), (665, 507), (652, 513), (656, 513), (657, 534), (654, 537), (638, 538), (632, 530), (615, 523), (611, 512), (610, 490), (612, 464), (614, 458), (618, 455)], [(575, 424), (574, 444), (572, 446), (570, 452), (570, 463), (568, 475), (568, 498), (566, 507)], [(569, 522), (577, 522), (578, 523), (584, 523), (585, 524), (593, 524), (594, 525), (606, 525), (607, 526), (609, 526), (614, 523)], [(637, 538), (634, 565), (628, 569), (556, 566), (540, 560), (524, 559)], [(520, 606), (576, 632), (629, 650), (621, 708), (602, 766), (588, 793)], [(663, 753), (668, 756), (652, 821), (646, 827), (635, 826), (600, 807), (587, 793), (572, 786), (538, 742), (515, 688), (506, 684)], [(439, 646), (446, 624), (453, 581), (458, 501), (455, 474)], [(408, 498), (419, 545), (420, 582), (416, 586), (382, 562), (363, 561), (359, 566), (359, 633), (356, 636), (412, 627), (410, 643), (387, 684)], [(397, 707), (406, 711), (411, 718), (407, 724), (399, 728), (323, 743), (293, 743), (269, 739), (245, 731), (222, 719)], [(229, 697), (224, 707), (222, 718), (213, 723), (200, 723), (196, 721), (182, 720), (172, 716), (162, 714), (153, 718), (149, 722)], [(134, 726), (148, 722), (157, 738), (189, 758), (229, 753), (240, 771), (240, 794), (226, 812), (208, 815), (184, 787), (132, 775), (100, 742), (70, 680), (64, 621), (108, 503), (152, 463), (167, 465), (200, 505), (228, 602), (229, 696), (248, 696), (254, 683)], [(269, 669), (273, 664), (276, 645), (280, 638), (285, 634), (284, 629), (286, 634), (288, 635), (300, 635), (325, 642), (330, 648), (328, 659), (321, 665), (307, 669), (273, 665)], [(263, 561), (277, 560), (279, 558), (275, 538), (276, 520), (323, 518), (332, 523), (335, 528), (336, 536), (335, 547), (329, 553), (319, 556), (280, 556)], [(402, 476), (407, 473), (409, 471), (409, 469), (407, 467), (406, 463), (401, 459), (401, 455), (398, 452), (398, 451), (396, 449), (391, 449), (390, 450), (397, 449), (398, 447), (405, 447), (408, 445), (410, 445), (413, 442)], [(509, 436), (521, 502), (523, 559), (519, 606), (505, 662), (506, 683), (489, 701), (440, 664), (439, 647), (414, 635), (416, 587)], [(412, 719), (418, 719), (419, 720), (423, 720), (424, 721), (425, 721), (426, 720), (429, 720), (430, 721), (433, 721)], [(564, 527), (568, 523), (568, 520), (566, 516), (565, 508), (564, 509), (552, 508), (551, 507), (548, 507), (547, 506), (545, 506), (541, 504), (538, 504), (537, 503), (522, 503)], [(558, 422), (575, 423), (583, 414), (616, 421), (619, 455), (651, 457), (669, 488), (701, 494), (693, 630), (669, 755)], [(190, 759), (190, 760), (189, 761), (189, 766), (188, 767), (188, 772), (187, 773), (187, 776), (186, 777), (186, 783), (185, 784), (185, 786)], [(162, 713), (164, 627), (161, 603), (151, 571), (139, 546), (121, 522), (108, 516)]]
import pickle
with open("./data.pickle", "rb") as f:
    l = pickle.load(f)

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
        np[2] += 0.17
        do_stuff(np, compute_ik)

        waypoints = []
        group = MoveGroupCommander("right_arm")
        wpose = group.get_current_pose().pose
           
        np = newarc[0][:]
        np[2] += 0.12
        do_stuff(np, compute_ik)
        for point in newarc[1:]:
            np = point[:]
            np[2]  += 0.12
            waypoints.append(create_pose(np, wpose))

        np = newarc[-1][:]
        np[2] += 0.17
        waypoints.append(create_pose(np, wpose))

        (plan, fraction) = group.compute_cartesian_path(
            waypoints,
            0.001,
            0.0
        )
        # slow it down
        scale = 0.2
        for point in plan.joint_trajectory.points:
            point.time_from_start /= scale
            point.velocities = [v * scale for v in point.velocities]
            point.accelerations = [v * scale for v in point.accelerations]
        group.execute(plan)

if __name__ == "__main__":
    main()
