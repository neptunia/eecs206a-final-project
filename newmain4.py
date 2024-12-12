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


def convert_to_real_world_coordinates(rel_point, tag_pos):
    # Extract the corners of the paper from tag_pos
    in2m = 0.0254 #1 inch in meters
    x1, y1 = tag_pos[0][:2]
    x2, y2 = tag_pos[1][:2]

    print(x1, y1, x2, y2)
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
    #theta = math.atan2(y2 - y1, x2 - x1) - math.atan2(0.14+0.03, 0.108+0.03)
    
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
    
    # Step 1: Translate the relative point to the origin
    rel_x_centered = rel_x - rel_width / 2
    rel_y_centered = rel_y - rel_height / 2
    
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

#star
#l = [[(392, 892), (359, 851), (289, 653), (185, 603), (105, 524), (274, 387), (270, 250), (249, 202)], [(271, 252), (299, 263), (417, 401), (439, 415), (456, 409), (598, 289), (635, 276), (652, 253)], [(405, 166), (402, 171), (402, 174), (401, 175), (401, 188), (402, 189), (402, 190), (401, 191), (401, 203), (402, 204), (402, 208)], [(482, 641), (484, 648), (473, 664), (400, 873), (391, 883)], [(484, 649), (607, 683), (647, 685), (635, 650), (530, 535), (536, 502), (630, 318), (636, 276)], [(279, 641), (279, 640), (280, 639), (280, 627), (279, 626), (279, 613)]]


# amogus
l = [[(577, 389), (487, 356), (392, 348), (387, 323), (366, 313), (292, 312), (277, 335), (240, 343), (191, 396), (182, 458), (209, 548), (270, 605), (270, 667), (290, 695), (369, 718), (476, 719), (503, 685), (512, 614), (271, 605)], [(391, 348), (382, 422), (372, 443), (352, 454), (313, 454), (288, 445), (276, 424), (277, 336)], [(578, 390), (603, 409), (606, 430), (602, 440), (588, 450), (528, 459), (510, 475), (503, 496), (511, 512), (564, 529), (589, 547), (599, 562), (602, 599), (584, 610), (512, 613)]]

#l = [[(256, 518), (521, 680), (519, 791), (56, 492), (46, 563), (617, 937), (658, 408), (259, 516)], [(522, 679), (526, 671), (532, 605), (548, 506), (548, 432)], [(618, 937), (664, 930), (716, 891), (740, 395), (714, 270), (382, 350), (66, 468), (56, 491)]]

#l = [[(367, 557), (363, 561), (362, 561), (361, 562), (360, 562), (356, 566), (349, 565), (346, 568), (340, 568)], [(331, 557), (336, 550), (340, 536), (349, 532), (352, 522), (351, 519), (343, 519), (334, 526), (329, 535), (330, 542), (337, 549), (346, 551), (351, 566), (355, 566), (358, 569), (365, 569), (375, 565), (379, 561), (385, 548), (386, 538), (389, 549), (388, 568)], [(346, 550), (346, 549), (347, 549), (348, 548), (351, 548), (354, 545), (354, 543), (353, 542), (353, 541), (351, 539), (351, 538), (350, 537), (350, 534), (349, 533)], [(356, 519), (357, 518), (362, 518), (363, 517), (365, 518), (369, 518), (370, 519), (372, 519), (373, 520), (372, 523), (376, 527), (380, 527), (382, 528), (383, 530), (384, 528), (383, 527), (383, 525), (381, 521), (384, 530), (384, 532), (385, 533), (384, 535), (385, 536), (386, 535), (386, 537)], [(372, 508), (360, 496), (351, 493), (346, 489), (338, 495), (333, 490), (332, 486), (326, 490), (318, 514), (315, 535), (315, 561), (321, 588)], [(356, 789), (357, 785), (361, 780), (354, 775), (349, 782), (340, 774), (335, 760), (333, 750), (333, 731), (337, 717)], [(350, 705), (349, 706), (349, 707), (347, 709), (347, 710), (345, 712), (345, 713), (344, 714), (344, 715), (343, 716), (343, 717), (342, 718), (342, 720), (341, 721), (341, 722), (340, 723), (340, 728), (339, 729)], [(776, 726), (695, 751), (658, 753), (606, 718), (572, 725), (566, 715), (513, 721), (464, 788), (427, 798), (373, 800), (370, 784), (348, 774), (334, 748)], [(338, 742), (340, 739), (340, 731), (338, 730), (336, 732), (336, 736), (335, 737), (335, 746), (341, 740), (342, 749), (345, 755), (345, 761), (343, 764), (344, 764), (345, 766), (346, 760), (347, 760), (350, 763), (354, 773)], [(391, 766), (386, 771), (385, 771), (382, 774), (379, 775), (375, 778), (373, 778), (362, 789), (362, 793)], [(417, 799), (418, 803), (428, 813), (439, 820), (457, 825), (474, 824), (495, 815), (510, 802), (519, 786), (523, 769), (517, 773), (514, 784), (505, 798), (494, 807), (479, 814), (471, 816), (448, 815), (436, 810), (424, 799)], [(402, 745), (402, 742), (403, 741), (403, 735), (404, 734), (404, 727), (403, 726), (403, 721)], [(381, 735), (378, 739), (366, 744), (360, 731), (353, 727), (349, 719), (351, 713), (358, 707), (364, 708), (364, 712), (357, 718), (354, 724), (354, 730), (349, 735), (349, 740), (358, 748), (364, 749), (367, 746), (371, 751), (375, 752), (387, 749), (396, 740), (400, 729), (400, 723), (396, 713), (393, 710), (388, 712), (383, 704), (375, 704)], [(382, 712), (380, 710), (378, 710), (374, 706), (374, 705), (373, 706), (368, 706), (367, 707), (366, 707), (365, 708), (364, 713), (366, 715), (366, 717), (370, 721), (370, 722), (371, 723), (373, 723), (370, 724), (370, 725), (367, 727), (361, 727), (360, 728), (359, 728), (359, 729)], [(299, 754), (299, 752), (298, 751), (298, 745), (297, 746), (290, 746), (291, 747), (292, 754), (294, 757), (294, 760), (295, 761), (296, 766), (305, 784), (307, 782), (307, 779), (304, 773), (304, 771), (303, 770), (303, 768), (301, 764), (300, 757), (299, 756)], [(316, 789), (312, 791), (308, 786), (313, 777), (306, 756), (269, 753), (257, 747), (202, 737), (181, 729), (148, 710)], [(140, 745), (150, 760), (179, 789), (207, 809), (231, 821), (276, 832), (307, 831), (331, 824)], [(339, 809), (341, 809), (342, 808), (344, 808), (345, 807), (348, 807), (349, 806), (352, 806), (353, 805), (355, 805), (356, 804), (359, 804), (360, 803), (363, 803), (364, 802), (368, 802), (369, 801), (371, 801), (372, 800)], [(394, 874), (414, 888), (445, 919), (447, 925), (433, 924), (416, 911), (393, 878), (393, 874), (385, 867), (333, 844), (288, 836)], [(282, 805), (287, 804), (289, 806), (291, 806), (297, 801), (305, 798), (312, 793), (325, 802), (324, 803), (321, 803), (320, 804), (317, 804), (312, 806), (295, 806), (294, 805), (288, 806), (284, 808), (279, 808), (278, 806), (273, 806)], [(289, 715), (291, 715), (292, 714), (295, 714), (296, 713), (300, 713), (301, 712), (301, 707), (300, 706), (299, 707), (294, 707), (293, 708), (290, 708), (289, 709), (282, 709), (281, 710), (271, 710), (270, 709), (258, 709)], [(456, 609), (458, 612), (462, 626), (463, 638), (464, 639), (464, 655), (467, 667), (467, 672), (470, 679)], [(512, 637), (507, 618), (498, 599), (479, 578), (472, 576), (466, 578), (459, 587), (453, 614), (453, 651), (458, 677), (463, 681), (470, 680), (476, 684), (493, 679), (505, 668)], [(551, 650), (556, 651), (560, 655), (567, 715)], [(606, 717), (597, 709), (583, 700), (576, 693), (573, 685), (573, 676), (578, 661)], [(682, 579), (670, 603), (664, 620), (662, 632), (644, 632), (632, 627), (628, 618), (630, 608), (638, 588), (657, 552), (657, 549)], [(575, 570), (576, 577), (581, 586), (588, 592), (596, 595), (608, 562), (634, 514)], [(704, 428), (718, 421), (724, 415), (735, 394), (733, 393), (723, 405), (709, 412), (697, 413), (684, 410), (667, 400), (655, 388), (626, 341)], [(605, 310), (615, 294), (627, 283), (635, 278), (652, 273), (672, 274), (686, 280), (700, 293), (707, 304), (707, 308), (693, 298), (684, 297)], [(742, 371), (742, 373), (741, 374), (741, 377), (740, 378), (740, 381), (739, 382), (739, 384), (738, 385), (738, 387), (737, 388), (737, 389), (736, 390), (736, 391), (735, 392), (734, 392)], [(720, 421), (738, 425), (742, 423), (774, 335)], [(750, 470), (751, 469), (752, 469), (753, 468), (755, 468), (756, 467), (759, 467), (760, 466), (764, 466), (765, 465), (767, 465), (768, 464), (770, 464), (771, 463), (773, 463), (774, 462), (776, 462)], [(721, 638), (723, 639), (727, 639), (730, 641), (732, 641), (739, 635), (750, 629), (752, 629), (756, 627), (759, 627), (760, 626), (767, 626), (768, 625), (770, 625), (771, 626), (776, 626)], [(727, 598), (728, 596), (734, 596), (735, 595), (740, 595), (741, 594), (750, 593), (751, 592), (753, 592), (754, 591), (756, 591), (757, 590), (759, 590), (763, 588), (766, 588), (767, 587), (767, 606)], [(719, 638), (720, 638), (721, 637), (721, 627), (720, 626), (720, 623), (719, 622), (718, 619), (711, 612), (707, 612)], [(679, 640), (679, 649), (680, 650), (682, 664), (683, 665), (683, 668), (684, 669), (684, 672), (687, 681), (687, 687), (688, 688), (687, 701)], [(608, 386), (659, 425), (740, 439), (749, 470), (629, 466), (561, 585), (569, 601), (597, 604), (629, 620), (630, 639), (660, 713)], [(596, 639), (599, 624), (574, 623), (569, 624), (563, 630), (561, 636), (562, 643), (565, 649), (578, 660), (587, 647), (594, 640), (597, 641), (599, 651), (634, 716)], [(658, 754), (694, 797), (717, 801), (714, 808), (692, 810), (650, 769), (621, 764), (595, 778), (539, 841), (504, 860), (470, 857), (408, 820), (376, 820)], [(428, 888), (450, 911), (509, 944), (567, 961), (602, 960), (635, 950), (673, 928), (686, 912), (645, 904), (615, 890), (594, 871), (542, 891), (489, 889), (374, 820), (355, 826)], [(477, 774), (479, 775), (484, 775), (485, 776), (500, 776), (501, 775), (507, 775), (517, 772), (517, 769), (518, 768), (518, 759), (517, 758), (516, 749), (510, 738), (505, 734)], [(577, 800), (583, 836), (570, 844), (563, 851), (545, 856), (540, 856), (533, 859)], [(566, 850), (571, 850), (577, 847), (579, 847), (582, 845), (584, 845), (585, 844), (585, 842), (584, 841), (584, 838), (583, 837), (585, 845), (587, 849), (587, 852), (588, 853), (590, 861), (593, 866), (594, 870)], [(613, 889), (612, 892), (608, 896), (607, 896), (601, 902), (600, 902), (597, 905), (596, 905), (592, 909), (591, 909), (589, 911), (588, 911), (584, 915)], [(644, 904), (636, 909), (612, 916), (587, 918), (582, 916), (558, 928), (531, 934)], [(662, 909), (659, 913), (622, 930), (585, 939), (550, 940), (535, 938), (530, 934), (493, 930)], [(688, 913), (705, 915), (634, 970), (530, 980), (414, 931), (343, 859), (187, 826), (122, 765), (89, 673), (81, 563), (134, 400), (201, 330), (339, 267), (450, 159), (511, 154), (443, 186), (342, 287), (301, 298)], [(345, 285), (383, 269), (430, 238), (481, 190), (495, 172)], [(512, 152), (552, 122), (589, 116), (633, 126), (677, 154), (729, 158), (778, 171)], [(630, 165), (634, 165), (643, 160), (652, 158), (654, 154), (635, 146), (612, 142), (590, 143), (574, 147), (557, 155), (544, 165)], [(558, 176), (558, 177), (557, 178), (556, 178), (555, 179), (554, 179), (553, 180), (552, 180), (550, 182), (549, 182), (548, 183), (547, 183), (546, 184), (545, 184), (544, 185), (543, 185), (541, 187), (540, 187), (539, 188), (538, 188), (537, 189), (535, 189)], [(510, 205), (511, 205), (512, 204), (515, 203), (517, 201), (520, 200), (525, 196), (526, 196), (527, 195), (528, 195), (530, 193), (531, 193), (534, 190), (534, 188), (533, 187), (532, 187), (531, 186), (530, 186), (529, 185), (526, 184)], [(597, 231), (613, 189), (630, 166), (596, 165), (562, 176), (543, 166), (520, 187), (507, 207), (436, 253)], [(433, 308), (471, 283), (511, 271), (548, 273), (584, 291), (596, 231), (541, 233), (492, 246), (445, 270), (406, 302)], [(320, 374), (322, 375), (326, 375), (328, 371), (328, 367), (337, 359), (351, 353), (360, 353), (369, 358), (373, 362), (371, 365), (363, 369), (354, 372), (351, 372), (350, 373), (337, 374), (336, 375), (327, 375)], [(311, 442), (311, 443), (313, 445), (320, 448), (317, 457), (303, 451), (293, 450), (292, 449), (276, 449), (275, 451)], [(264, 451), (268, 451), (269, 452), (272, 452), (274, 451), (276, 454), (283, 454), (284, 455), (288, 455), (289, 456), (296, 457), (308, 463), (315, 469)], [(328, 493), (329, 498), (333, 502), (325, 521), (321, 525), (321, 534), (318, 537), (319, 545), (322, 548), (324, 569), (335, 599)], [(307, 531), (307, 529), (308, 528), (308, 522), (309, 521), (309, 515), (310, 514), (310, 508), (311, 507), (311, 502), (312, 501), (312, 496), (313, 495), (313, 491), (314, 490), (314, 485), (315, 484), (315, 481)], [(293, 494), (288, 517), (285, 519), (288, 527), (285, 547), (285, 580), (292, 630)], [(258, 747), (289, 746), (295, 728), (297, 743), (305, 744), (299, 688), (299, 613), (307, 532), (284, 524), (288, 518)], [(325, 522), (326, 523), (326, 524), (327, 525), (327, 529), (322, 534), (323, 534), (323, 537), (324, 538), (324, 544), (322, 546), (322, 547)], [(339, 476), (339, 479), (340, 480), (340, 482), (345, 487), (346, 487), (348, 485), (348, 479), (349, 478), (349, 465)], [(452, 439), (464, 451), (466, 451), (466, 452), (467, 451), (472, 451), (473, 452), (483, 452), (484, 453), (484, 455), (481, 458), (480, 458), (478, 460), (476, 460), (474, 462), (473, 462), (470, 459), (470, 458), (468, 456), (467, 453)], [(472, 462), (456, 468), (439, 468), (429, 465), (417, 457), (409, 458), (430, 451), (451, 439), (465, 424), (493, 384), (497, 382), (504, 392), (508, 405), (508, 427), (498, 452), (480, 469)], [(485, 453), (486, 452), (488, 452), (494, 446), (500, 436), (500, 434), (502, 431), (503, 422), (504, 421), (504, 416), (503, 415), (499, 415), (498, 414), (488, 412), (487, 411), (476, 410)], [(498, 453), (523, 453), (524, 452), (537, 451), (544, 448), (550, 441), (556, 428), (557, 423)], [(575, 353), (581, 349), (579, 346), (591, 354), (611, 376), (608, 385), (590, 406), (569, 419), (555, 423), (531, 423), (511, 419), (504, 414), (501, 398), (493, 387), (498, 378), (526, 351), (541, 358), (557, 360), (573, 355), (575, 345), (555, 340), (540, 343), (530, 349)], [(776, 334), (712, 314), (706, 308), (677, 296), (664, 299), (651, 307), (625, 340), (605, 311), (585, 292)], [(663, 632), (671, 639), (681, 638), (694, 628), (714, 602), (723, 599), (753, 603), (766, 607), (777, 614)], [(709, 628), (710, 628), (711, 629), (712, 629), (714, 631), (714, 632), (716, 634), (716, 635), (717, 636), (717, 637), (719, 639), (719, 653)], [(663, 867), (716, 854), (762, 836), (733, 751), (667, 763)], [(640, 767), (638, 769), (634, 769), (633, 770), (628, 770), (627, 771), (623, 771), (622, 772), (617, 772), (616, 773), (611, 773), (610, 774), (606, 774), (605, 773)], [(776, 920), (743, 919), (742, 918), (729, 918), (728, 917), (718, 917), (717, 916), (708, 916), (706, 915)], [(293, 632), (281, 619), (278, 601), (278, 562), (283, 524), (239, 522), (218, 525), (183, 535), (147, 552)], [(150, 510), (205, 456), (261, 437), (408, 458), (438, 476), (479, 469), (522, 539), (550, 626), (546, 661), (509, 728), (523, 768), (569, 726)], [(157, 456), (186, 415), (225, 375), (282, 334), (356, 308), (405, 303), (432, 308), (385, 355), (352, 343), (330, 352), (319, 374), (302, 370)]]

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
