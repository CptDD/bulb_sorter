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

from bulb_scanner.srv import *

from geometry_msgs.msg import *
from std_msgs.msg import *

from gazebo_msgs.srv import *
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




def main():
	#moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('bulb_scanner_valid_replay')
	
	rospack=rospkg.RosPack()
	path=rospack.get_path('bulb_scanner')
	valid_points_file=path+'/graph/valid.txt'

	print('Reading valid points!')
	valid_points=read_valid_points(valid_points_file)


	for p in valid_points:
		print(p)

	arm = baxter_interface.Limb('right')

	print('Moving to home!')

	move_to_home(arm)

	for k,v in valid_points.items():
		print('Attempting to move to :'+str(k)+' which has coordinates :'+str(v['x'])+" "+str(v['y'])+" "+str(v['z']))

		move_to_valid(arm,v)




if __name__ == '__main__':
	main()