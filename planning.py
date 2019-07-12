#!/usr/bin/python3
import matplotlib.pyplot as plt
import matplotlib.animation as animation
#from matplotlib import collections
import matplotlib.patches as patches
import matplotlib.path as mplpath
#from scipy.interpolate import interp1d
#from scipy.signal import savgol_filter
import imageio
import numpy as np
import yaml
from math import *
from copy import *
import heapq
import time
import dubins
#import reeds_shepp
import itertools
import random

'''Shared functions'''
#######################################################################################################################
# Find point on segment joining two closest points in array given. Find this point at given time.
def findPointOnPathAndSlope(pathArr,time):
	#find closest point time on path below given time through linear interpolation
	closeTime = pathArr.index(min(pathArr,key=lambda x: abs(x[2]-time)))   
	time = round(time, 10) 
	if time - round(pathArr[closeTime][2],10) <= 0:									#finds closest point below given time and uses it's timestamp
		closeTime = closeTime - 1
	if closeTime == pathArr[len(pathArr)-1][2]:
		closeTime -= 1
	
	slope = [pathArr[closeTime+1][0]-pathArr[closeTime][0], pathArr[closeTime+1][1]-pathArr[closeTime][1]]
	if slope == [0,0]:
		slope = [1,0]
	else:
		slope = np.array(slope)/(np.array(slope)**2).sum()**.5					#Convert slope to unit vector
	
	if pathArr[closeTime][2] == time: 										#If requested time is in array, just return that point 
		return [pathArr[closeTime][0],pathArr[closeTime][1]],slope
	#use time to find point on line
	q = (time-pathArr[closeTime][2])/(pathArr[closeTime+1][2]-pathArr[closeTime][2])
	intersect = (1-q)*np.array(pathArr[closeTime][0:2]) + q*np.array(pathArr[closeTime+1][0:2])
	intersect = np.insert(intersect,2,time)
	return intersect,slope

# Rotate a point counterclockwise by a given angle around a given origin.
def rotate(origin, point, angle_degrees):
	angle = radians(angle_degrees)
	ox, oy = origin
	px, py = point
	qx = ox + cos(angle) * (px - ox) - sin(angle) * (py - oy)
	qy = oy + sin(angle) * (px - ox) + cos(angle) * (py - oy)
	return qx, qy

# Find the polygon for the shot of the robot
def findShotPolygon(roboLoc,shot,camDir_unit_vect):
	minShotDistVect = np.array(camDir_unit_vect) * shot['dist_range'][0]
	maxShotDistVect = np.array(camDir_unit_vect) * shot['dist_range'][1]
	angle_values = np.arange(shot['angle_range'][0] + shot['actor_facing'], shot['angle_range'][1] + shot['actor_facing'], 5) #points every 5 degrees
	minShotVect = []
	maxShotVect = []
	for value in angle_values:
		minShotVect.insert(0,rotate([0,0],minShotDistVect,value))
		maxShotVect.append(rotate([0,0],maxShotDistVect,value))
	concatVect = []
	for element in maxShotVect:
		concatVect.append(element+np.array(roboLoc[0:2]))
	for element in minShotVect:
		concatVect.append(element+np.array(roboLoc[0:2]))
	return concatVect

# Find the field of view rays for the robot wrt its config. (needs to be adjusted for shot)
def findFovRays(roboSlope,robot_num,rotatePoint,shot = None):
	if shot is None:
		camDirectionUnit = rotate([0,0],roboSlope,config['robots'][robot_num]['camera_orientation'])
	else:
		camDirectionUnit = rotate([0,0],roboSlope,shot['actor_facing'] + 180)
	camDirectionUnit = (np.array(camDirectionUnit) / (np.array(camDirectionUnit)**2).sum()**0.5)
	camDirection = camDirectionUnit*(config['map']['height']+config['map']['width'])**2 #Make unit vector longer
	fovSlopes = [rotate([0,0],camDirection,config['robots'][robot_num]['fov']/2),rotate([0,0],camDirection,-config['robots'][robot_num]['fov']/2)]
	
	fov = config['robots'][robot_num]['fov']
	fovFillArr = []
	if not fov >= 360:
		fovFillArr.append(rotatePoint[0:2])
	if fov >= 180:
		fovFillArr.append(np.array(fovSlopes[0])+np.array(rotatePoint[0:2]))
		fovFillArr.append(np.array(rotate([0,0],camDirection,90)) + np.array(rotatePoint[0:2]))
		fovFillArr.append(np.array(camDirection) + np.array(rotatePoint[0:2]))
		fovFillArr.append(np.array(rotate([0,0],camDirection,-90)) + np.array(rotatePoint[0:2]))
		fovFillArr.append(np.array((fovSlopes[1])+np.array(rotatePoint[0:2])))
	elif fov >= 90:
		fovFillArr.append(np.array(fovSlopes[0])+np.array(rotatePoint[0:2]))
		fovFillArr.append(np.array(camDirection) + np.array(rotatePoint[0:2]))
		fovFillArr.append(np.array((fovSlopes[1])+np.array(rotatePoint[0:2])))
	else:
		fovFillArr.append((fovSlopes[0]+np.array(rotatePoint[0:2])))
		fovFillArr.append((fovSlopes[1]+np.array(rotatePoint[0:2])))
	return fovFillArr,camDirectionUnit
#######################################################################################################################

def visualize(config, saveName=None, show_path = False):
				
	#update animation
	def update(i):
		#Set actor point for 'i' timestep
		time = round((1.0/config['animation']['fps'])*i,10)
		if not config['actor']['path'] == None:
			actorPtOnPath,actorSlope = findPointOnPathAndSlope(config['actor']['path'],time)
			actorPt.set_data(actorPtOnPath[0],actorPtOnPath[1])
			#Set each robot's point and line connecting it to the actor for each 'i' timestep
			for robot in range(0,len(roboLocArr)):
				if 'path' in config['robots'][robot]:
					if not config['robots'][robot]['path'] == None:
						ptOnPath,slope = findPointOnPathAndSlope(config['robots'][robot]['path'],(1.0/config['animation']['fps'])*i)
						fovRays,camDirectionUnit = findFovRays(slope,robot,ptOnPath)
						fovFill[robot].set_xy(fovRays)
						roboLocArr[robot].set_data(ptOnPath[0],ptOnPath[1])
						segmentArr[robot].set_data([actorPtOnPath[0],ptOnPath[0]],[actorPtOnPath[1],ptOnPath[1]])
						for shotnum,shot in enumerate(config['shots']):
							if shot['start_time'] <= time <= shot['end_time']:
								shotarr = findShotPolygon(ptOnPath,shot,camDirectionUnit)
								pathObject = mplpath.Path(shotarr)
								insidePoly = (pathObject.contains_points([actorPtOnPath[0:2]]))
								#print(time)
								
								if insidePoly:
									roboShot[2*(robot*len(config['shots']) + shotnum)].set_xy(shotarr)
									roboShot[2*(robot*len(config['shots']) + shotnum)+1].set_xy([[0,0]])
									fovRays,_ = findFovRays(slope,robot,ptOnPath,shot)
									fovFill[robot].set_xy(fovRays)
								else:
									roboShot[2*(robot*len(config['shots']) + shotnum)+1].set_xy(shotarr)
									roboShot[2*(robot*len(config['shots']) + shotnum)].set_xy([[0,0]])
							else:
								roboShot[2*(robot*len(config['shots']) + shotnum)].set_xy([[0,0]])
								roboShot[2*(robot*len(config['shots']) + shotnum)+1].set_xy([[0,0]])
			#Combine actor point, each robot's point and line connecting it to the actor, and the fov polygons into combination array for redrawing on screen
			combArr = []
			for segment in segmentArr:
				combArr.append(segment)
			for roboLoc in roboLocArr:
				combArr.append(roboLoc)
			for polyPoint in fovFill:
				combArr.append(polyPoint)
			for shotPoly in roboShot:
				combArr.append(shotPoly)
			combArr.append(actorPt)
			return combArr

	
	print('Visualizing')
	# Read image and create plots
	img1 = imageio.imread(config['map']['path'])
	img1 = [[int(abs(xpoint) > 0) for xpoint in ypoint] for ypoint in img1]
	if show_path:
		fig1, ax1 = plt.subplots()
		ax1.set(xlim=(0, config['map']['width']), ylim=(0, config['map']['height']))
		res1 = ax1.imshow(img1,cmap="gray",extent=[0, config['map']['width'], 0, config['map']['height']]) #gray
		res1.set_clim(0,1)
		for identity,robot in enumerate(solved_config['robots']):
			if 'path' in robot:
				x = np.array([pathpoint[0] for pathpoint in robot['path']])
				y = np.array([pathpoint[1] for pathpoint in robot['path']])
				ax1.plot(x, y, label= "Robot {}".format(identity))
		ax1.legend()
	fig, ax = plt.subplots()
	ax.set(xlim=(0, config['map']['width']), ylim=(0, config['map']['height']))
	res = ax.imshow(img1,cmap="gray",extent=[0, config['map']['width'], 0, config['map']['height']]) #gray
	res.set_clim(0,1)

	#Create plot points and segments for each robots. Create the point for the actor. Create the Field of View for the robots.
	roboLocArr = []
	segmentArr = []
	fovFill = []
	roboShot = []
	actorPt, = ax.plot(0,0,marker='o',c=config['actor']['color'],label = 'Actor')
	for identity,robot in enumerate(config['robots']):
		segment, = ax.plot([],[],c=(0,0,0,.5))
		pt, = ax.plot([],[],marker='o',c=robot['color'], label = 'Robot {}'.format(identity))
		fovFill.append(patches.Polygon([[0,0],[0,0],[0,0]],closed=True, alpha=.25, fc=robot['color'], ec='None'))
		ax.add_patch(fovFill[len(fovFill)-1])
		for configShot in config['shots']:
			roboShot.append(patches.Polygon([[0,0],[1,1],[2,2],[5,5]],closed=True, alpha=.25, fc=config['animation']['in_shot'], ec='None'))
			ax.add_patch(roboShot[len(roboShot)-1])
			roboShot.append(patches.Polygon([[0,0],[1,1],[2,2],[5,5]],closed=True, alpha=.25, fc=config['animation']['out_shot'], ec='None'))
			ax.add_patch(roboShot[len(roboShot)-1])
		roboLocArr.append(pt)
		segmentArr.append(segment)
	ax.legend(loc = 'upper right') 
	#Setup animation
	if not config['actor']['path'] == None:
		maxtime = max(config['actor']['path'],key=lambda x: x[2])
		interval=1000.0/config['animation']['fps']
		frames = config['animation']['fps']*maxtime[2]
		ani = animation.FuncAnimation(fig, update, frames = frames, interval=interval, blit = True)

	#run/save animation
	if not saveName == None:
		#if not config['actor']['path'] == None:
		ani.save(saveName, writer='imagemagick', fps=config['animation']['fps'])	#To save gif of animation
		#ani.save(saveName, writer='ffmpeg', fps=config['animation']['fps'])		#To save mp4 of animation
	else:
		plt.show()


def solve(inputconfig):
	solved_config = deepcopy(inputconfig)
	solve_resolution = solved_config['solve']['resolution']
	already_solved_paths = {}
	img = imageio.imread(config['map']['path'])
	img = [[int(abs(xpoint) > 0) for xpoint in ypoint] for ypoint in img]

	def findPathToShot(current_position,shot_start_loc,robot_cfg,parent_node,shot):
		PRM_num_points = solved_config['solve']['PRM_num_points']

		def isValidPath(path,node):
			current = node
			if len(path[0]) == 4:
				path = [[element[0],element[1],element[3]] for element in path]
			while current != None:
				for robot in current.assigned_robots:
					if type(robot.given_shot) == dict and len(robot.path) > 0:
						assigned_robot_path = [[p[0],p[1],p[3]] for p in robot.path]
						for test_roboPos in path:
							if assigned_robot_path[0][2] <= test_roboPos[2] <= assigned_robot_path[-1][2]:
								planned_pathPos,theta_planned = findPointOnPathAndSlope(assigned_robot_path,test_roboPos[2])
								fovPoly = findFovRays(theta_planned,robot.identity,planned_pathPos,robot.given_shot)[0]
								insidePoly = mplpath.Path(fovPoly).contains_points([test_roboPos[0:2]])
								if insidePoly[0]:
									return False,robot.given_shot
				current = current.parent
			return True,{}

		def PRM_Modified(point_array,start,goal):
			aStar_Timeout = float(solved_config['solve']['PRM_timeout']) #seconds
			class astar_node():
				"""docstring for astar_node"""
				def __init__(self, node_position, g = 0, parent = None):
					self.node_position = node_position
					self.parent = parent
					self.g = g
					self.h = 0
					self.f = 0

				def __lt__(self, other):
					return self.f < other.f

				#def __repr__(self):
				#	return 'astar_node({})'.format(self.__hash__())

				def __hash__(self):
					return hash("astar_node({})".format(self.node_position))
			
				def __eq__(self, other):
					return self.__hash__() == other.__hash__()
		
				def findChildren(self,directed_graph):
					node_list_prm =[]
					for vertex in directed_graph[tuple(self.node_position)]:
						node_list_prm.append(astar_node(vertex[1],vertex[0],self))
					return node_list_prm
					
			point_array.append(goal)
			#Create a dictionary of points which have as indices all the points they connect to with the path length
			directed_graph = {}
			for current_point in point_array:
				node_list_prm = []
				# Don't look for node past the goal node.
				if tuple(current_point) == tuple(goal):
					continue
				# Add all connections from current point to every other point to our point array so we know where our current point can lead
				for dest_point in point_array:
					dubins_path = dubins.shortest_path(current_point,dest_point, robot_cfg['max_turn_rad']).sample_many(solve_resolution)[0]
					# Check to make sure the dubins path doesn't pass through obstacles.
					error = 0
					for point in dubins_path:
						if error == 1:
							break
						pointx = int(point[0] * len(img[0]) / float(solved_config['map']['width']))
						pointy = int(point[1] * len(img) / float(solved_config['map']['height']))
						#print(point,pointx,pointy,len(img[pointy]),len(img))
						if point[0] > solved_config['map']['width'] or point[1] > solved_config['map']['height']:
							error = 1
						if not img[solved_config['map']['height'] - pointy][pointx]:
							error = 1
					if error == 1:
						continue
					dubins_path_length = solve_resolution*len(dubins_path)
					node_list_prm.append((dubins_path_length,tuple(dest_point)))
				directed_graph[tuple(current_point)] = tuple(node_list_prm)
			#Then do the same for the starting node.
			node_list_prm = []
			for dest_point in point_array:
				dubins_path = dubins.shortest_path(start,dest_point, robot_cfg['max_turn_rad']).sample_many(solve_resolution)[0]
				# Check to make sure the dubins path doesn't pass through obstacles.
				error = 0
				for point in dubins_path:
					if error == 1:
						break
					pointx = int(point[0] * len(img[0]) / float(solved_config['map']['width']))
					pointy = int(point[1] * len(img) / float(solved_config['map']['height']))
					#print(point,pointx,pointy,len(img[pointy]),len(img))
					if point[0] > solved_config['map']['width'] or point[1] > solved_config['map']['height']:
						error = 1
					if not img[solved_config['map']['height'] - pointy][pointx]:
						error = 1
				if error == 1:
					continue
				dubins_path_length = solve_resolution*len(dubins_path)
				node_list_prm.append((dubins_path_length,tuple(dest_point)))
			directed_graph[tuple(start)] = tuple(node_list_prm)


			# Run A* on the connected points to find the lowest path length that will produce a usable path
			open_list_prm = []
			closed_list_prm = set(())
			open_list_dict_prm = {}
			# Append Starting node, and run A*
			open_list_prm.append(astar_node(start))
			timeout = time.process_time()
			while len(open_list_prm) > 0 and time.process_time() - timeout < aStar_Timeout:
				current_node_prm = heapq.heappop(open_list_prm)
				closed_list_prm.add(current_node_prm)

				if tuple(current_node_prm.node_position) == tuple(goal): #If goal position reached, then check if valid solution and return if it is.
					current = current_node_prm
					path_print = []
					while current is not None:
						path_print.append(current.node_position)
						current = current.parent
					print('possible solution',path_print)
					#print('possible solution',current_node_prm.parent.node_position, current_node_prm.node_position)
					path = []
					current = current_node_prm
					while current.parent is not None:
						#if solved_config['solve']['algorithm'] == 'dubins':
						dubins_path = dubins.shortest_path(current.parent.node_position, list(current.node_position), robot_cfg['max_turn_rad']).sample_many(solve_resolution)[0]
						path += dubins_path[::-1]
						#else:
						#	reeds_shepp_path = reeds_shepp.path_sample(current.parent.node_position,list(current.node_position),robot_cfg['max_turn_rad'],solve_resolution)
						#	path += reeds_shepp_path[::-1]
						current = current.parent
					# Flip path to face forwards
					path = path[::-1]
					#print(path)
					#Add timestamps for the path
					timestep_increment = (shot['start_time']-current_position[3])/len(path)
					for pos in range(len(path)):
						if (pos+1)*timestep_increment + current_position[3] == shot['start_time']:
							del path[pos]
							continue
						else:
							path[pos] = list(path[pos])
							path[pos].append((pos+1)*timestep_increment+current_position[3])
					if isValidPath(path,parent_node)[0]:
						return path
					else:
						continue
						
				for child_node_prm in current_node_prm.findChildren(directed_graph): #For each of the children, find if it needs to be added to open_list, and do necessary
					if child_node_prm in closed_list_prm and tuple(child_node_prm.node_position) != tuple(goal):
						continue
					child_node_prm.g = current_node_prm.g + child_node_prm.g
					child_node_prm.h = solve_resolution*len(dubins.shortest_path(child_node_prm.node_position,goal, robot_cfg['max_turn_rad']).sample_many(solve_resolution)[0])
					child_node_prm.f = child_node_prm.g + child_node_prm.h
					if child_node_prm in open_list_dict_prm:
						if child_node_prm.g >= open_list_dict_prm[child_node_prm]:
							continue
					open_list_dict_prm[child_node_prm] = child_node_prm.g
					heapq.heappush(open_list_prm, child_node_prm)
			#If no solution is found by A* return -1
			if time.process_time() - timeout < aStar_Timeout:
				print('No solution to PRM')
			else:
				print('PRM timeout hit')
			return -1

		#First check if the path from current_position to shot_start_loc with robot config has already been solved
		already_solved_identifier = (tuple(current_position[0:3]), tuple(shot_start_loc[0:3]), str(robot_cfg),tuple([str(cur_shot) for cur_shot in parent_node.available_shots]))
		if already_solved_identifier in already_solved_paths:
			return list(already_solved_paths[already_solved_identifier][0]),already_solved_paths[already_solved_identifier][1]

		#Check if endpoints are not in another shot
		path = [current_position,shot_start_loc]
		valid,return_shot = isValidPath(path,parent_node)
		if not valid:
			for cur_shot_num,cur_shot in enumerate(solved_config['shots']):
				if cur_shot == shot:
					shot1 = cur_shot_num + 1
				if cur_shot == return_shot:
					shot2 == cur_shot_num + 1
			print('Error: Shot {} overlaps with shot {}. (shots are not zero indexed)'.format(shot1,shot2))
			already_solved_paths[already_solved_identifier] = [],-1
			return [],-1

		#Check if a direct dubins path is valid, and if it is, just return that path
		#if solved_config['solve']['algorithm'] == 'dubins':
		path = dubins.shortest_path(current_position[0:3], shot_start_loc[0:3], robot_cfg['max_turn_rad'])
		path_sampled = path.sample_many(solve_resolution)[0]
		#else:
		#	path_sampled = reeds_shepp.path_sample(current_position[0:3], shot_start_loc[0:3], robot_cfg['max_turn_rad'],solve_resolution)

		#Add on timesteps to dubens path
		timestep_increment = (shot['start_time']-current_position[3])/len(path_sampled)
		for pos in range(len(path_sampled)):
			if (pos+1)*timestep_increment + current_position[3] == shot['start_time']:
				del path_sampled[pos]
				continue
			else:
				path_sampled[pos] = list(path_sampled[pos])
				path_sampled[pos].append((pos+1)*timestep_increment+current_position[3])

		cost_to_go = 1*solve_resolution*len(path_sampled)

		#Check to see if direct path passes over obsticles in environment png file
		error_test = 0
		for point in path_sampled:
			if error_test == 1:
				break
			pointx = int(point[0] * len(img[0]) / float(solved_config['map']['width']))
			pointy = int(point[1] * len(img) / float(solved_config['map']['height']))
			if point[0] > solved_config['map']['width'] or point[1] > solved_config['map']['height']:
				error_test = 1
			if not img[solved_config['map']['height'] - pointy][pointx]:
				error_test = 1


		if isValidPath(path_sampled,parent_node)[0] and error_test != 1:
			already_solved_paths[already_solved_identifier] = (tuple(path_sampled),cost_to_go)
			return path_sampled,cost_to_go


		#Because regular dubins didnt solve the problem, run a modified version of PRM to actually solve the problem

		#Randomly placed point.
		if solved_config['solve']['PRM_points'] == 'random':
			point_arr = [[random.uniform(0,solved_config['map']['width']),random.uniform(0,solved_config['map']['height']),random.uniform(0,2*pi)] for _ in range(PRM_num_points)]
		# Statically placed points
		else:
			cbrt_num_points = int((PRM_num_points + 1)**(1.0/3)) # Add one to the number of PRM points because of python rounding errors
			width_increment = float(solved_config['map']['width'])/cbrt_num_points
			height_increment = float(solved_config['map']['height'])/cbrt_num_points
			theta_increment = 2*pi/cbrt_num_points
			point_arr = []
			for i in range(cbrt_num_points):
				for j in range(cbrt_num_points):
					for k in range(cbrt_num_points):
						point_arr.append([i*width_increment,j*height_increment,k*theta_increment])
		#print(point_arr)
		
		print('Starting PRM')
		path_sampled = PRM_Modified(point_arr,current_position[0:3], shot_start_loc[0:3])
		print('Finished PRM')
		#If PRM failed, just return nothing
		if path_sampled == -1:
			already_solved_paths[already_solved_identifier] = [],-1
			return [],-1
		cost_to_go = 1*solve_resolution*len(path_sampled)
		
		already_solved_paths[already_solved_identifier] = (tuple(path_sampled),cost_to_go)
		return path_sampled,cost_to_go

	
	def findPathAndCost(robot_cfg, current_position, shot,parent_node):
		if shot is None:
			return [],0
		elif shot == 'finished':
			return [],0

		#calculate ctg from starting position to shot start
		actor_xyt,actor_slope_unitVect = findPointOnPathAndSlope(solved_config['actor']['path'],shot['start_time'])
		shot_start_loc = np.multiply(rotate([0,0],actor_slope_unitVect,(sum(shot['angle_range'])/2) + shot['actor_facing']),sum(shot['dist_range'])/2)+actor_xyt[0:2]
		shot_start_loc = np.append(shot_start_loc, atan2(actor_slope_unitVect[1],actor_slope_unitVect[0]))
		shot_start_loc = np.append(shot_start_loc, shot['start_time'])


		#Find a path not obstructed by obsticles, and not obstructed by shots to the start of the shot.
		path_sampled,cost_to_go = findPathToShot(current_position,shot_start_loc,robot_cfg,parent_node,shot)
		if cost_to_go == -1:
			return [],-1

		

		#Add cost to go for traveling too fast on dubins path
		#if len(path_sampled)>1:
		#	speed = hypot(path_sampled[0][0] - path_sampled[1][0],path_sampled[0][1] - path_sampled[1][1])/timestep_increment
		#	if speed > robot_cfg['speed']:
		#		cost_to_go += 1000000			#Change fix please
			
		
		# Find the path to follow the actor
		following_actor = []
		current_position = np.array([])
		time_arr = np.arange(shot['start_time'],shot['end_time'],solved_config['actor']['timestep_resolution'])
		#If endpoint was missed, add it
		if not time_arr[-1] == shot['end_time']:
			time_arr = np.append(time_arr,shot['end_time'])
		for time in time_arr:
			actor_xyt,actor_slope_unitVect = findPointOnPathAndSlope(solved_config['actor']['path'],time)
			current_position = np.multiply(rotate([0,0],actor_slope_unitVect,(sum(shot['angle_range'])/2) + shot['actor_facing']),sum(shot['dist_range'])/2)+actor_xyt[0:2]
			current_position = np.append(current_position, atan2(actor_slope_unitVect[1],actor_slope_unitVect[0]))
			current_position = np.append(current_position, time)
			# Check if the path the robot is taking is possible with its turning constraints.

			def get_intersect(a1, a2, b1, b2):
			    """ 
			    Returns the point of intersection of the lines passing through a2,a1 and b2,b1.
			    a1: [x, y] a point on the first line
			    a2: [x, y] another point on the first line
			    b1: [x, y] a point on the second line
			    b2: [x, y] another point on the second line
			    """
			    s = np.vstack([a1,a2,b1,b2])        # s for stacked
			    h = np.hstack((s, np.ones((4, 1)))) # h for homogeneous
			    l1 = np.cross(h[0], h[1])           # get first line
			    l2 = np.cross(h[2], h[3])           # get second line
			    x, y, z = np.cross(l1, l2)          # point of intersection
			    if z == 0:                          # lines are parallel
			        return (float('inf'), float('inf'))
			    return (x/z, y/z)


			#if len(following_actor) == 0:
			#	previous_point = path_sampled[-1]
			#else:
			#	previous_point = following_actor[-1]
			#if tuple(current_position[0:3]) == tuple(previous_point[0:3]):
			#	found_shot = -1
			#	for find_shot_num,find_shot in enumerate(solved_config['shots']):
			#		if find_shot == shot:
			#			found_shot = find_shot_num + 1
			#	print('At time {} the actor turns too sharply for any robot to capture shot {}'.format(time,found_shot))
			#	return [],-1


			#Then create the circle through the two vectors at given solve resolution
			following_actor.append(current_position.tolist())
			#Add cost for being oustide of shot polygon
			#robot_cam_dir = rotate([0,0],actor_slope_unitVect,robot_cfg['camera_orientation'])
			#shot_poly = findShotPolygon(current_position,shot,robot_cam_dir)
			#pathObject = mplpath.Path(shot_poly)
			#insidePoly = pathObject.contains_points([actor_xyt[0:2]])
			#if not insidePoly:
			#	cost_to_go += 500	#fix please

		#Smooth out path for following actor
		#smoothingnum = 11
		#for itt in range(0,smoothingnum):
		#	x = np.array([pathpoint[0] for pathpoint in following_actor])
		#	y = np.array([pathpoint[1] for pathpoint in following_actor])
		#	t = np.array([pathpoint[3] for pathpoint in following_actor])
		#	#print(len(x),len(y))
		#	window_size, poly_order = int(2.0/solved_config['solve']['time_resolution'])+1, 3
		#	x_sg = savgol_filter(x, window_size, poly_order)
		#	y_sg = savgol_filter(y, window_size, poly_order)
		#	following_actor = [[x_sg[loc],y_sg[loc],following_actor[loc][2],following_actor[loc][3]] for loc in range(0,len(following_actor))]

		#Add cost to go for each segment in the path next to the actor in a given shot
		#for loc in range(1,len(following_actor)):
		#	cost_to_go += hypot(following_actor[loc][0] - following_actor[loc-1][0],following_actor[loc][1] - following_actor[loc-1][1])

		#return total path, and cost to go
		#print(path_sampled)
		total_path = path_sampled + following_actor
		return total_path,cost_to_go

	class Robot():

		def __init__(self, identity, position ,given_shot = None,parent_node = None):
			self.position = position
			self.identity = identity
			self.robot_cfg = solved_config['robots'][identity]
			self.given_shot = given_shot
			self.path, self.cost_to_go = findPathAndCost(self.robot_cfg,position,given_shot,parent_node)
			if self.cost_to_go != -1:
				if given_shot is None:
					self.end_position = position
				elif given_shot == 'finished':
					self.end_position = position
				else:
					self.end_position = self.path[-1]										#(x,y,theta,time)

		def __hash__(self):
			return hash("Node({}{}{})".format(self.identity,self.position,self.given_shot))

		def __eq__(self, other):
			return self.identity == other.identity

	class Node():
		"""A Node for Dykstra Search"""
	
		def __init__(self,robots,available_shots,time,parent=None):
			self.robots = robots
			self.parent = parent
			self.time = time
			self.available_shots = available_shots
			self.unassigned_robots = []
			self.assigned_robots = [] 
			self.g = 0
			self.h = 0
			self.f = 0
			for robot in robots:
				if robot.given_shot is None:
					self.unassigned_robots.append(robot)
				else:
					self.assigned_robots.append(robot)
			self.total_ctg = sum([0]+[robot.cost_to_go for robot in self.assigned_robots if parent == None or (parent != None and robot in parent.unassigned_robots)])
			if parent is not None:
				self.total_ctg += parent.total_ctg

		def findChildren(self):


			node_list = []
			
			#If there are unassigned robots, then find all permutations of shots with robots possible
			if len(self.unassigned_robots) > 0:
				#If there are more or equal unassigned robots than available shots, find all permutations of robots to assign shots too.
				available_shots = deepcopy(self.available_shots)
				for robot in self.unassigned_robots:
					available_shots.append('finished')
				for robot in self.unassigned_robots:
					available_shots.append(None)

				shotPermutations = list(itertools.permutations(available_shots,1))	#for only one robot
				#print(shotPermutations)
				for perm in shotPermutations:	# shotPermutations looks like ((shot1,shot2),(shot1,shot3),(shot2,shot1),(shot2,shot3)...)
					robot_list = []
					unassigned_shots = deepcopy(available_shots)
					unassigned_robots = deepcopy(self.unassigned_robots)
					#for shot in perm:
					shot = perm[0]
					robot = unassigned_robots[0]
					if type(shot) == dict and robot.end_position[3] > shot['start_time']:
						new_robot = Robot(robot.identity,robot.end_position,None,self)
					else:
						new_robot = Robot(robot.identity,robot.end_position,shot,self)
						unassigned_shots.remove(shot)
					if new_robot.cost_to_go == -1:
						continue
					robot_list.append(new_robot)
					unassigned_robots.remove(robot)

					for robot in unassigned_robots:
						robot_list.append(deepcopy(robot))

					#for robot in unassigned_robots:
					#	robot_list.append(Robot(robot.identity,robot.end_position,'idle'))
					for shot in list(unassigned_shots):
						if shot is None:
							unassigned_shots.remove(shot)
						elif shot == 'finished':
							unassigned_shots.remove(shot)
					node_list.append(Node(robot_list+deepcopy(self.assigned_robots),unassigned_shots,self.time,self))
				return node_list

			#If there is no unassigned robots, set the robot(s) with the first shot to end as having None as their shot
			#and put them in the unassigned robots list. Set the time to the time the shot ended
			else:
				#print(len(self.assigned_robots))
				#print([[robot.given_shot,type(robot.given_shot)] for robot in self.assigned_robots])
				new_node = deepcopy(self)
				new_node.parent = self
				end_time_arr = [robot.given_shot['end_time'] for robot in new_node.assigned_robots if type(robot.given_shot) == dict]
				if len(end_time_arr) == 0:
					end_time_arr.append(0)
				min_end_time = min(end_time_arr)
				#print("given_shot")
				#for robot in self.assigned_robots:
				#	print(robot.given_shot)
				#print('allRobots:')
				#for robot in self.robots:
				#	print(robot.given_shot)
				#print("Min end time: ",min_end_time)
				
				new_node.time = min_end_time
				for robot in new_node.assigned_robots:
					if not type(robot.given_shot) == dict:
						continue
					elif robot.given_shot['end_time'] == min_end_time:
						robot.given_shot = None
						new_node.unassigned_robots.append(robot)
						new_node.assigned_robots.remove(robot)
				
				return [new_node]
				

		def __lt__(self, other):
			return self.f < other.f

		#def __repr__(self):
		#	return "Node({}{})".format([robot.__hash__() for robot in self.robots],self.time)

		def __hash__(self):
			return hash("Node({}{})".format([robot.__hash__() for robot in self.robots],self.time))
	
		def __eq__(self, other):
			return self.__hash__() == other.__hash__()

	startTime = time.process_time()

	#A* algorithm
	#Initialize lists/heaps/dictionaries and endtime
	open_list = []
	heapq.heapify(open_list)
	open_list_dict = {}
	closed_list = set(())
	end_time = solved_config['actor']['path'][-1][2]

	#Add first robot to the heap
	startRobotList = []
	for robotNumber,robot in enumerate(solved_config['robots']):
		robot['start_coord'].append(0)
		startRobotList.append(Robot(robotNumber,robot['start_coord'],None,None))

	open_list.append(Node(startRobotList,solved_config['shots'],0,None))

	#While the open_list has nodes in it, keep running
	while len(open_list) > 0:
		#print("Open List Length: ",len(open_list)) #debug
		#for node in open_list:
		#	print('unassigned robots:',len(node.unassigned_robots))
		#	for robot in node.unassigned_robots:
		#		print('robot:',robot.identity, robot.given_shot)
		#	for robot in node.assigned_robots:
		#		print('robot:',robot.identity, robot.given_shot,robot.cost_to_go)
		#Remove one node from the heap and add it to the closed_list
		current_node = heapq.heappop(open_list)
		closed_list.add(current_node)


		if current_node.time == end_time and len(current_node.available_shots) == 0: #If the node has reached the end time, then finish and return the paths
			robot_paths = []
			for robot in solved_config['robots']:
				robot_paths.append([])
			current = current_node
			added_shots = []	#Keep track of which shots have already been added to path to avoid duplicate adds
			while current is not None:
				#print('available_shots',current.available_shots)	#debug
				#for robot in current.robots:
					#print(robot.identity,robot.given_shot,current.parent)	#debug
				for robot in current.assigned_robots:
					#print(robot.identity,robot.given_shot,current.parent)
					if robot.given_shot in added_shots:
						continue
					added_shots.append(robot.given_shot)
					for position in robot.path[::-1]:
						robot_paths[robot.identity].append([position[0],position[1],position[3]])
				current = current.parent
			for element in range(0,len(robot_paths)):
				solved_config['robots'][element]['path'] = robot_paths[element][::-1]
				if solved_config['robots'][element]['path'] != [] and solved_config['robots'][element]['path'][-1][2] < end_time:
					solved_config['robots'][element]['path'].append(solved_config['robots'][element]['path'][-1][0:2] + [end_time])
				#print(solved_config['robots'][element]['path'])	#debug
			for robotNum in range(0,len(solved_config['robots'])):
				if solved_config['robots'][robotNum]['path'] == []:
					del solved_config['robots'][robotNum]['path']
			#return with solution
			print('Solution Found in {} seconds.'.format(time.process_time() - startTime))
			return solved_config
		
		for child_node in current_node.findChildren(): #For each of the children, find if it needs to be added to open_list, and do necessary
			if child_node in closed_list or child_node in open_list:
				continue
			child_node.g = current_node.g + child_node.total_ctg
			child_node.h = 0
			child_node.f = child_node.g + child_node.h
			if child_node in open_list_dict:
				if child_node.g >= open_list_dict[child_node]:
					continue
			open_list_dict[child_node] = child_node.g
			#if child.g < 1000:
			heapq.heappush(open_list, child_node)
	
	
		#for num,node in enumerate(open_list):
		#	print("node",num)
		#	for robot in node.robots:
		#	print('robot',robot.identity,robot.given_shot)


	#If no solution found print that
	print('No Solution Found, check your starting positions, speeds, shots, and robots.')
	return solved_config



#read config file
#config = yaml.safe_load(open('result_working.yaml', 'r'))
#visualize(config)
config = yaml.safe_load(open('rocky.yaml','r'))
#print(findPointOnPathAndSlope(config['actor']['path'],5))
#print(findPointOnPathAndSlope(config['actor']['path'],8))
solved_config = solve(config)
with open('errors.yaml', 'w') as yaml_file:
	yaml.dump(solved_config, yaml_file, default_flow_style=False)
#visualize(solved_config)
#config = yaml.safe_load(open('result_working.yaml', 'r'))
#visualize(config)
#config = yaml.safe_load(open('result_working2.yaml', 'r'))
visualize(solved_config,show_path=True)
#config = yaml.safe_load(open('result.yaml', 'r'))
#visualize(config,'demo.gif')