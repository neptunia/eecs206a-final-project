#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys
from intera_interface import gripper as robot_gripper
import tf2_ros

'''i do not remember at all how to do this, probably is so wrong for getting the location of the tags'''
#pls tell me this is how we get the 
tfBuffer = tf2_ros.Buffer()
tfListener = tf2_ros.TransformListener(tfBuffer)
trans1 = tfBuffer.lookup_transform('ar_tag_01', 'base', rospy.Time())
trans2 = tfBuffer.lookup_transform('ar_tag_02', 'base', rospy.Time())
#going with tag 01 is top left and tag 02 is bottom right (if my math later isnt bs)
pox_x1 = trans1.transform.translation.x
pos_y1 = trans1.transform.translation.y
pos_z1 = trans1.transform.translation.z
pox_x2 = trans2.transform.translation.x
pos_y2 = trans2.transform.translation.y
pos_z2 = trans2.transform.translation.z
#get the table height by average to be safe
z_height = (pos_z1 + pos_z2) / 2


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
    total_angle = np.arctan(pox_x2 - pox_x1, pos_y2 - pos_y1)
    paper_angle = total_angle - diagonal_angle

    for contour in list:
        for point in contour:
            #transform i hop this is how math works, no clue tho
            #i hope the paper coords were in (x,y)
            #a copy in case i change it in the x before i change the y
            orig = [point[0], point[1]]
            point[0] = pox_x1 + (orig[0] * np.sin(paper_angle) + orig[1] * np.cos(paper_angle)) * scale
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
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    # Set up the right gripper
    right_gripper = robot_gripper.Gripper('right_gripper')

    # Calibrate the gripper (other commands won't work unless you do this first)
    #Uhhh it says it has to do this, maybe just screw the marker holder onto only one side...
    print('Calibrating...')
    right_gripper.calibrate()
    rospy.sleep(2.0)


    '''some constants, will need adjustment probably'''
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
            move(compute_ik, contour[0][0], contour[0][1], lift_pen, ox, oy, oz, ow)
            for point in contour:
                #put down pen at first point and keep pen down while going to other points
                move(compute_ik, point[0], point[1], drop_pen, ox, oy, oz, ow)
            #lift pen up at end of contour
            move(compute_ik, contour[-1][0], contour[-1][1], lift_pen, ox, oy, oz, ow)



# Python's syntax for a main() method
if __name__ == '__main__':
    drawcontours(convertcontours('''put the list of list of points here i guess'''))

