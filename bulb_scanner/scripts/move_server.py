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
						neighbours.append(temp[i].strip())

					temp_tags['neighbours']=neighbours
		else:
			points[label]=temp_tags

	f.close()
	return points

def is_south(target,coords):

	x=float(target['x'])
	y=float(target['y'])
	z=float(target['z'])

	if(float(coords['y'])<y):
		return True
	else:
		return False

def is_north(target,coords):

	x=float(target['x'])
	y=float(target['y'])
	z=float(target['z'])

	if(float(coords['y'])>y):
		return True
	else:
		return False

def is_west(target,coords):

	
	x=float(target['x'])
	y=float(target['y'])
	z=float(target['z'])

	if(float(coords['x'])<x):
		return True
	else:
		return False

def is_east(target,coords):

	
	x=float(target['x'])
	y=float(target['y'])
	z=float(target['z'])

	if(float(coords['x'])>x):
		return True
	else:
		return False


def is_up(target,coords):
	x=float(target['x'])
	y=float(target['y'])
	z=float(target['z'])

	if(float(coords['z'])>z):
		return True
	else:
		return False


def is_down(target,coords):

	x=float(target['x'])
	y=float(target['y'])
	z=float(target['z'])

	if(float(coords['z'])<z):
		return True
	else:
		return False


def get_closest_south(target,neighbours,points):


	south_neighs=dict()
	for i in neighbours:

		if(is_south(points[target],points[i])):
	
			if(float(points[target]['y'])<=0):
				diff=abs(float(points[target]['y']))-abs(float(points[i]['y']))
			else:
				diff=abs(float(points[i]['y']))-abs(float(points[target]['y']))

			if(diff<-0.044):
				south_neighs[i]=diff

	sorted_list= sorted(south_neighs.items(), key=operator.itemgetter(0))

	return sorted_list
	


def get_closest_north(target,neighbours,points):

	north_neighs=dict()

	for i in neighbours:
		if(is_north(points[target],points[i])):
			if(float(points[target]['y'])<0):
				diff=abs(float(points[target]['y']))-abs(float(points[i]['y']))
			else:
				diff=abs(float(points[i]['y']))-abs(float(points[target]['y']))

			if(diff>0.044):
				north_neighs[i]=diff


	sorted_list=sorted(north_neighs.items(),key=operator.itemgetter(0))

	return sorted_list



def get_closest_east(target,neighbours,points):

	east_neighs=dict()

	for i in neighbours:

		if(is_east(points[target],points[i])):

			if(float(points[target]['x'])>0):
				diff=abs(float(points[target]['x']))-abs(float(points[i]['x']))
				if diff<-0.05:
					east_neighs[i]=diff
			else:
				diff=abs(float(points[i]['x']))-abs(float(points[target]['x']))
				if diff>0.05:
					east_neighs[i]=diff


	sorted_list=sorted(east_neighs.items(),key=operator.itemgetter(0),reverse=True)

	return sorted_list

def get_closest_west(target,neighbours,points):

	west_neighs=dict()

	for i in neighbours:

		if(is_west(points[target],points[i])):


			if(float(points[target]['x'])>0 and float(points[i]['x'])>0):
				diff=abs(float(points[target]['x']))-abs(float(points[i]['x']))

				if diff>0.04:
					west_neighs[i]=diff


			elif(float(points[target]['x'])<0 and float(points[i]['x'])<0):
				diff=abs(float(points[i]['x']))-abs(float(points[target]['x']))

				if diff>0.04:
					west_neighs[i]=diff

			else:
				diff=abs(float(points[target]['x']))-abs(float(points[i]['x']))

				if diff<-0.05:
					west_neighs[i]=diff


	sorted_list=sorted(west_neighs.items(),key=operator.itemgetter(0))

	return sorted_list



def get_south_neighbours(target,neighbours,points):

	south_neigh=[]
	south_neigh=get_closest_south(target,neighbours,points)
		#if(is_south(points[target],points[i])):
		#	south_neigh.append(i)

	return south_neigh

def get_north_neighbours(target,neighbours,points):

	north_neigh=[]

	#north_neigh=get_closest_north(target,neighbours,points)

	#for i in neighbours:
	#	if(is_north(points[target],points[i])):
	#		north_neigh.append(i)


	return north_neigh

def get_northern_neighbours(target,neighbours,points):
	north_neighs=[]

	north_neighs=get_closest_north(target,neighbours,points)

	return north_neighs

def get_east_neighbours(target,neighbours,points):

	east_neigh=[]
	east_neigh=get_closest_east(target,neighbours,points)


	#for i in neighbours:
	#	if(is_east(points[target],points[i])):
	#		east_neigh.append(i)

	return east_neigh

def get_west_neighbours(target,neighbours,points):

	west_neigh=[]
	west_neigh=get_closest_west(target,neighbours,points)

	#for i in neighbours:
	#	if(is_west(points[target],points[i])):
	#		west_neigh.append(i)

	return west_neigh


def move_to_home(arm):

	ns = "ExternalTools/" + 'right'+ "/PositionKinematicsNode/IKService"

	iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
	ikreq=SolvePositionIKRequest()
	hdr=Header(stamp=rospy.Time.now(),frame_id='world')

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

	ns = "ExternalTools/" + 'right'+ "/PositionKinematicsNode/IKService"

	iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
	ikreq=SolvePositionIKRequest()
	hdr=Header(stamp=rospy.Time.now(),frame_id='world')

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




def handle(req):

	print('Moving to a pose . . .')

	res=positionResponse()

	rospack=rospkg.RosPack()
	path=rospack.get_path('bulb_scanner')
	valid_points_file=path+'/graph/valid.txt'
	valid_points=read_valid_points(valid_points_file)

	action=req.action.data
	v=req.current_position_label.data

	arm = baxter_interface.Limb('right')

	print('Action is :'+str(action))
	print('Current pose :'+str(v))

	if(action==0):
		print('Getting the northern neighbours!')
		north_neighs=get_northern_neighbours(v,valid_points[v]['neighbours'],valid_points)

		if(len(north_neighs)>0):
			print('The closest northern neighbour is :'+str(north_neighs[0][0]))
			point=valid_points[north_neighs[0][0]]
			move_to_valid(arm,point)
		else:
			print('There are no northern neighbours for this point :'+str(v)+'--->staying in the same position!')

			

	elif(action==1):
		print('Getting the southern neighbours!')
		south_neighs=get_south_neighbours(v,valid_points[v]['neighbours'],valid_points)

		if(len(south_neighs)>0):
			print('The closest southern neighbour is :'+str(south_neighs[0][0]))
			point=valid_points[south_neighs[0][0]]
			move_to_valid(arm,point)
		else:
			print('There are no southern neighbours for point:'+str(v)+'--->staying in the same position!')


	elif(action==2):
		print('Getting the eastern neighbours!')
		east_neighs=get_east_neighbours(v,valid_points[v]['neighbours'],valid_points)

		if(len(east_neighs)>0):
			print('The closest eastern neighbour is :'+str(east_neighs[0][0]))
			point=valid_points[east_neighs[0][0]]
			move_to_valid(arm,point)
		else:
			print('There are no eastern neighbours for point :'+str(v)+'--->staying in the same position!')


	elif(action==3):
		print('Getting the western neighbours!')
		west_neighs=get_west_neighbours(v,valid_points[v]['neighbours'],valid_points)

		if(len(west_neighs)>0):
			print('The closest western point is :'+str(west_neighs[0][0]))
			point=valid_points[west_neighs[0][0]]
			move_to_valid(arm,point)
		else:
			print('There are no northern neighbours for this point :'+str(v)+'--->staying in the same position!')

	elif(action==-1):
		print('Moving to initial pose :'+str(v)+' !')
		point=valid_points[v]

		#move_to_valid(arm,point)


		ns = "ExternalTools/" + 'right'+ "/PositionKinematicsNode/IKService"

		iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
		ikreq=SolvePositionIKRequest()
		hdr=Header(stamp=rospy.Time.now(),frame_id='base')

		print(point)

		pose=PoseStamped()
		pose.header=hdr

		pose.pose.position.x=float(point['x'])
		pose.pose.position.y=float(point['y'])
		pose.pose.position.z=float(point['z'])

		pose.pose.orientation.x=float(point['xo'])
		pose.pose.orientation.y=float(point['yo'])
		pose.pose.orientation.z=float(point['zo'])
		pose.pose.orientation.w=float(point['wo'])

		ikreq.pose_stamp.append(pose)
		resp = iksvc(ikreq)

		if(resp.isValid[0]):
			print('Found a plan moving there!')
			limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
			arm.move_to_joint_positions(limb_joints)
		else:
			print('Plan was not found!')

	else:
		print('The action :'+str(action)+' is a decision action!')

	return res


def main():
	#moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('bulb_scanner_move')
	server=rospy.Service('/bulb_move',position,handle)
	print('Move server up . . .')
	rospy.spin()




if __name__ == '__main__':
	main()