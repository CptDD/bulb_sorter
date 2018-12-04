#!/usr/bin/python

import rospy
import rospkg
import numpy as np
import math
import copy
import sys
import numpy as np
import operator

import baxter_interface

from geometry_msgs.msg import *
from std_msgs.msg import *

from baxter_core_msgs.srv import*
from tf.transformations import quaternion_from_euler

def read_valid_points(filename):
	f=open(filename,'r')

	points=dict()

	for line in f:
		if '---' not in line:
			if '___' not in line:
				if 'Original' in line:
					temp_tags=dict()
					label=line.split(' ')[1].rstrip().replace('p','')

				if 'Position' in line:
					line=line.replace('Position :','').rstrip()
					pos=line.split('\t')
					temp_tags['x']=pos[0]
					temp_tags['y']=pos[1]
					temp_tags['z']=pos[2]

				if 'Orientation' in line:
					line=line.replace('Orientation :','').rstrip()
					line=line.split('\t')
					temp_tags['xo']=line[0]
					temp_tags['yo']=line[1]
					temp_tags['zo']=line[2]
					temp_tags['wo']=line[3]

				if 'Neighbours' in line:
					neighbours=[]
					temp=line.split('\t')
					for i in range(1,len(temp)):
						neighbours.append(temp[i].rstrip().lstrip())

					temp_tags['neighbours']=neighbours
		else:
			points[label]=temp_tags

	f.close()
	return points



def move_to_home(arm):

	ns = "/ExternalTools/right/PositionKinematicsNode/IKService"

	iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
	ikreq=SolvePositionIKRequest()
	hdr=Header(stamp=rospy.Time.now(),frame_id='base')

	pose=PoseStamped()
	pose.header=hdr

	pose.pose.position.x=0.57
	pose.pose.position.y=-0.18
	pose.pose.position.z=0.10

	pose.pose.orientation.x=-0.14
	pose.pose.orientation.y=0.98
	pose.pose.orientation.z=0
	pose.pose.orientation.w=0


	ikreq.pose_stamp.append(pose)
	resp = iksvc(ikreq)

	if(resp.isValid[0]):
		limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
		arm.move_to_joint_positions(limb_joints)
		print('Moving to home')


def move_to_valid(arm,valid):

	v=False

	
	ns = "/ExternalTools/right/PositionKinematicsNode/IKService"

	iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
	ikreq=SolvePositionIKRequest()
	hdr=Header(stamp=rospy.Time.now(),frame_id='base')

	pose=PoseStamped()
	pose.header=hdr

	pose.pose.position.x=float(valid['x'])
	pose.pose.position.y=float(valid['y'])
	pose.pose.position.z=float(valid['z'])

	pose.pose.orientation.x=float(valid['xo'])
	pose.pose.orientation.y=float(valid['yo'])
	pose.pose.orientation.z=float(valid['zo'])
	pose.pose.orientation.w=float(valid['wo'])


	ikreq.pose_stamp.append(pose)
	resp = iksvc(ikreq)

	if(resp.isValid[0]):
		print('Found a plan moving there!')
		limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
		arm.move_to_joint_positions(limb_joints)
		v=True
	else:
		print('Plan was not found!')

	return v

def move_to_pose(arm,pose):


	ns = "/ExternalTools/right/PositionKinematicsNode/IKService"
	iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
	ikreq=SolvePositionIKRequest()
	hdr=Header(stamp=rospy.Time.now(),frame_id='base')

	poseStamped=PoseStamped()
	poseStamped.header=hdr

	poseStamped.pose=pose

	ikreq.pose_stamp.append(poseStamped)
	resp = iksvc(ikreq)

	if(resp.isValid[0]):
		print('Found a plan moving there!')
		limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
		arm.move_to_joint_positions(limb_joints)
	else:
		print('Plan was not found!')



def main():
	#moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('bulb_test_pose')
	
	rospack=rospkg.RosPack()
	path=rospack.get_path('bulb_scanner')
	valid_points_file=path+'/graph/valid.txt'

	print('Reading valid points!')
	valid_points=read_valid_points(valid_points_file)

	arm = baxter_interface.Limb('right')

	pose=Pose()


	pose.position.x=0.494414940473
	pose.position.y=-0.0148086917108
	pose.position.z=0.39721278643

	pose.orientation.x=-0.524787523064
	pose.orientation.y=0.605637086824
	pose.orientation.z=0.391714993968
	pose.orientation.w=0.452063201555

	move_to_home(arm)

	move_to_pose(arm,pose)

	move_to_home(arm)

	move_to_valid(arm,valid_points['64'])




if __name__ == '__main__':
	main()