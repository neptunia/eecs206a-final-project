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
        new = convert_to_real_world_coordinates(point, tag_pos)
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

l = [[(577, 389), (487, 356), (392, 348), (387, 323), (366, 313), (292, 312), (277, 335), (240, 343), (191, 396), (182, 458), (209, 548), (270, 605), (270, 667), (290, 695), (369, 718), (476, 719), (503, 685), (512, 614), (271, 605)], [(391, 348), (382, 422), (372, 443), (352, 454), (313, 454), (288, 445), (276, 424), (277, 336)], [(578, 390), (603, 409), (606, 430), (602, 440), (588, 450), (528, 459), (510, 475), (503, 496), (511, 512), (564, 529), (589, 547), (599, 562), (602, 599), (584, 610), (512, 613)]]
#l = [[(403, 695), (395, 700), (387, 691), (326, 706), (297, 682), (240, 685), (230, 675), (220, 579), (199, 511), (161, 469), (125, 448), (113, 408), (129, 362), (151, 343), (182, 338), (231, 352), (234, 369), (181, 411), (148, 463)], [(147, 468), (110, 504), (86, 552), (59, 642), (74, 703), (119, 772), (151, 789), (180, 792), (202, 818), (232, 809), (238, 780), (226, 759), (180, 761), (155, 742), (151, 730)], [(160, 720), (161, 637), (156, 603), (146, 573), (135, 552), (115, 528), (106, 524)], [(162, 469), (167, 463), (211, 440), (284, 417), (286, 395), (294, 371), (276, 366), (237, 365)], [(295, 372), (332, 365), (337, 346), (348, 333), (354, 329), (371, 327), (416, 351), (421, 357), (426, 383), (401, 395), (383, 397), (365, 375), (333, 365)], [(383, 398), (382, 416), (380, 419), (351, 414), (320, 415), (299, 418), (285, 417)], [(399, 468), (401, 469), (403, 477), (409, 488), (407, 501), (409, 521), (416, 544), (417, 575), (414, 578)], [(437, 645), (441, 641), (447, 614), (452, 573), (456, 516), (456, 488), (453, 480)], [(458, 450), (500, 424), (510, 434), (524, 563), (507, 692), (488, 704), (439, 665), (436, 646), (411, 642), (413, 579), (371, 566), (357, 575), (358, 634), (409, 637), (388, 690)], [(395, 701), (395, 702), (396, 703), (396, 705), (397, 706), (397, 708), (398, 709), (398, 716), (399, 717), (399, 719), (402, 722), (406, 722), (407, 723), (414, 723), (415, 724), (423, 724)], [(263, 677), (263, 674), (264, 673), (264, 650), (258, 644), (257, 644), (254, 641), (253, 641), (249, 637), (249, 636)], [(241, 505), (240, 525), (251, 533), (248, 553), (243, 559), (246, 558), (258, 564), (255, 590), (250, 593), (254, 592), (263, 597), (247, 637), (229, 638)], [(265, 649), (269, 645), (283, 640), (307, 642), (319, 646), (325, 653), (325, 656), (321, 666), (311, 672), (304, 674), (297, 681)], [(239, 686), (236, 693), (225, 706), (220, 726), (212, 728), (192, 727), (163, 723), (160, 721), (151, 729), (137, 728)], [(187, 764), (187, 765), (186, 766), (186, 768), (185, 769), (185, 772), (184, 773), (184, 779), (183, 780), (183, 787), (182, 788), (182, 789), (181, 790), (181, 791)], [(400, 721), (396, 729), (389, 733), (316, 748), (269, 744), (221, 726)], [(259, 564), (260, 563), (269, 563), (270, 562), (272, 562), (274, 559), (273, 558), (273, 554), (272, 553), (272, 536), (270, 533), (268, 533), (267, 534), (252, 534)], [(275, 559), (316, 559), (324, 557), (331, 550), (332, 535), (330, 530), (325, 525), (318, 522), (310, 521), (277, 524), (273, 528), (271, 533)], [(198, 509), (260, 499), (333, 497), (347, 472), (391, 474), (399, 467), (402, 443), (393, 430), (382, 425), (381, 419)], [(426, 384), (442, 389), (445, 394), (449, 441), (445, 443), (429, 443), (421, 445), (403, 444)], [(449, 442), (457, 450), (456, 463), (455, 464), (455, 470), (454, 471), (453, 479), (438, 484), (424, 486), (416, 489), (410, 489)], [(524, 506), (564, 509), (573, 523), (614, 525), (639, 536), (635, 563), (631, 568), (625, 570), (562, 569), (525, 563)], [(565, 509), (567, 506), (568, 479), (572, 455), (572, 442), (574, 427), (571, 424)], [(552, 424), (551, 424), (546, 429), (544, 429), (543, 430), (538, 430), (537, 431), (531, 431), (530, 432), (524, 432), (523, 433), (515, 433), (514, 434), (511, 434)], [(619, 455), (618, 458), (615, 461), (613, 468), (613, 479), (612, 480), (612, 487), (611, 488), (611, 510), (612, 511), (612, 517), (614, 524)], [(640, 536), (649, 535), (653, 533), (656, 525), (657, 518), (659, 516), (660, 513), (666, 507), (669, 495), (672, 491)], [(703, 493), (725, 485), (734, 454), (737, 216), (698, 183), (644, 164), (573, 172), (526, 206), (507, 243), (506, 290), (551, 422), (615, 422), (621, 456), (654, 460), (672, 490), (703, 494), (688, 683), (647, 828), (574, 790), (507, 693)], [(520, 612), (622, 649), (629, 656), (623, 705), (608, 754), (590, 792)]]

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
        np[2] += 0.135
        do_stuff(np, compute_ik)
        for point in newarc[1:]:
            np = point[:]
            np[2]  += 0.135
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
