import matplotlib.pyplot as plt
import matplotlib.animation as animation
#from matplotlib import collections
import matplotlib.patches as patches
import matplotlib.path as mplpath
#from scipy.interpolate import interp1d
#from scipy.signal import savgol_filter
import os
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
def findPointOnPathAndSlope(pathArr,time,config = None):
	#find closest point time on path below given time through linear interpolation
	closeTime = pathArr.index(min(pathArr,key=lambda x: abs(x[-1]-time)))   
	if 0 < time < pathArr[closeTime][-1]:									#finds closest point below given time and uses it's timestamp
		closeTime = closeTime - 1
	if closeTime == len(pathArr) - 1:
		closeTime -= 1
	
	slope = [pathArr[closeTime+1][0]-pathArr[closeTime][0], pathArr[closeTime+1][1]-pathArr[closeTime][1]]
	if slope == [0,0]:
		slope = [1,0]
	else:
		slope = np.array(slope)/(np.array(slope)**2).sum()**.5					#Convert slope to unit vector
	
	if time - pathArr[closeTime][-1] < 0.001: 										#If requested time is in array, just return that point 
		#print('equal',time)
		if len(pathArr[0]) == 5:
			#print('actor')
			slope = [cos(pathArr[closeTime][2]),sin(pathArr[closeTime][2])]
		return [pathArr[closeTime][0],pathArr[closeTime][1],time],slope
	#If this path array is the actors, first find dubins path and linearly interpolate between points on dubins path.
	#if len(pathArr[0]) == 5:
	#	#print('actor')
	#	shortest_path = dubins.shortest_path(pathArr[closeTime][0:3],pathArr[closeTime+1][0:3], pathArr[closeTime][3]).sample_many((1.0/7)*config['solve']['resolution'])[0]
	#	# Add timesteps to dubins path
	#	time_increment = (pathArr[closeTime+1][-1] - pathArr[closeTime][-1])/len(shortest_path)
	#	for loc in range(len(shortest_path)):
	#		shortest_path[loc] = list(shortest_path[loc])
	#		shortest_path[loc].append((loc + 1) * time_increment + pathArr[closeTime][-1])
	#	#print(time_increment)
	#	intersect,slope = findPointOnPathAndSlope(shortest_path,time,config)
	#else:
		#use time to find point on line from linear interpolation
	q = (time-pathArr[closeTime][-1])/(pathArr[closeTime+1][-1]-pathArr[closeTime][-1])
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
	angle_values = np.arange(shot['angle_range'][0], shot['angle_range'][1], 1) #points every 1 degree
	#angle_values = np.arange(shot['angle_range'][0] + shot['actor_facing'], shot['angle_range'][1] + shot['actor_facing'], 1) #points every 5 degrees
	minShotVect = []
	maxShotVect = []
	for value in angle_values:
		minShotVect.insert(0,rotate([0,0],minShotDistVect,-value))
		maxShotVect.append(rotate([0,0],maxShotDistVect,-value))
	concatVect = []
	for element in maxShotVect:
		concatVect.append(element+np.array(roboLoc[0:2]))
	for element in minShotVect:
		concatVect.append(element+np.array(roboLoc[0:2]))
	return concatVect

# Find the field of view rays for the robot wrt its config. (needs to be adjusted for shot)
def findFovRays(roboSlope,robot_num,rotatePoint,config,shot = None):
	if shot is None:
		camDirectionUnit = rotate([0,0],roboSlope,config['robots'][robot_num]['camera_orientation'])
	else:
		camDirectionUnit = rotate([0,0],roboSlope,shot['actor_facing'] - 180)
		#camDirectionUnit = rotate([0,0],roboSlope,(sum(shot['angle_range'])/2) + shot['actor_facing'] - 180)
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
			actorPtOnPath,actorSlope = findPointOnPathAndSlope(config['actor']['path'],time,config)
			actorPt.set_data(actorPtOnPath[0],actorPtOnPath[1])
			#Set each robot's point and line connecting it to the actor for each 'i' timestep
			for robot in range(0,len(roboLocArr)):
				if 'path' in config['robots'][robot]:
					if not config['robots'][robot]['path'] == None:
						ptOnPath,slope = findPointOnPathAndSlope(config['robots'][robot]['path'],(1.0/config['animation']['fps'])*i,config)
						fovRays,camDirectionUnit = findFovRays(slope,robot,ptOnPath,config)
						fovFill[robot].set_xy(fovRays)
						roboLocArr[robot].set_data(ptOnPath[0],ptOnPath[1])
						segmentArr[robot].set_data([actorPtOnPath[0],ptOnPath[0]],[actorPtOnPath[1],ptOnPath[1]])
						for shotnum,shot in enumerate(config['shots']):
							if shot['start_time'] <= time <= shot['end_time']:
								#camDirectionUnit = rotate([0,0],slope,shot['actor_facing'])
								tmpfovRays, tmpCamDirUnit = findFovRays(slope,robot,ptOnPath,config,shot)
								#print(slope,tmpCamDirUnit)
								shotarr = findShotPolygon(ptOnPath,shot,tmpCamDirUnit)
								pathObject = mplpath.Path(shotarr)
								insidePoly = (pathObject.contains_points([actorPtOnPath[0:2]]))
								#print(time)
								
								if insidePoly:

									roboShot[2*(robot*len(config['shots']) + shotnum)].set_xy(shotarr)
									roboShot[2*(robot*len(config['shots']) + shotnum)+1].set_xy([[0,0]])
									#fovRays,_ = findFovRays(slope,robot,ptOnPath,config,shot)
									fovRays = tmpfovRays
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
		ax1.axis('equal')
		res1 = ax1.imshow(img1,cmap="gray",extent=[0, config['map']['width'], 0, config['map']['height']]) #gray
		res1.set_clim(0,1)
		# Draw robot path
		for identity,robot in enumerate(config['robots']):
			if 'path' in robot:
				x = np.array([pathpoint[0] for pathpoint in robot['path']])
				y = np.array([pathpoint[1] for pathpoint in robot['path']])
				ax1.plot(x, y, color = robot['color'], label= "Robot {}".format(identity))
		# Draw actor path
		#actor_path = []
		#for actor_pt in range(len(config['actor']['path'])-1):
		#	actor_path += dubins.shortest_path(config['actor']['path'][actor_pt][0:3], config['actor']['path'][actor_pt+1][0:3], config['actor']['path'][actor_pt][3]).sample_many(config['solve']['resolution'])[0]
		# Link timesteps in graph to make reading easier
		ax1.plot([x[0] for x in config['actor']['path']],[y[1] for y in config['actor']['path']], color = config['actor']['color'], label = "Actor")
		for timestep in np.linspace(config['actor']['path'][0][-1],config['actor']['path'][-1][-1],10):
			actorPt,_ = findPointOnPathAndSlope(config['actor']['path'],timestep,config)
			for robot in config['robots']:
				if hasattr(robot,'path'):
					robopt,_ = findPointOnPathAndSlope(robot['path'],timestep)
					angle = atan2(robopt[1] - actorPt[1],robopt[0] - actorPt[0])
					ax1.plot([actorPt[0],robopt[0]-cos(angle)/2.0],[actorPt[1],robopt[1]-sin(angle)/2.0],color = 'k',alpha = 0.5,ls = '--',marker = (3,0,degrees(angle) - 90),markevery = [1])
		#ax1.plot([pathpoint[0] for pathpoint in actor_path], [pathpoint[1] for pathpoint in actor_path], label= "Actor")


		ax1.legend()
	fig, ax = plt.subplots()
	ax.set(xlim=(0, config['map']['width']), ylim=(0, config['map']['height']))
	ax.axis('equal')
	res = ax.imshow(img1,cmap="gray",extent=[0, config['map']['width'], 0, config['map']['height']]) #gray
	res.set_clim(0,1)

	#Create plot points and segments for each robots. Create the point for the actor. Create the Field of View for the robots.
	roboLocArr = []
	segmentArr = []
	fovFill = []
	roboShot = []
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
	actorPt, = ax.plot(0,0,marker='o',c=config['actor']['color'],label = 'Actor')
	ax.legend(loc = 'upper right') 
	#Setup animation
	if not config['actor']['path'] == None:
		maxtime = max(config['actor']['path'],key=lambda x: x[-1])
		interval=1000.0/config['animation']['fps']
		frames = config['animation']['fps']*maxtime[-1]
		ani = animation.FuncAnimation(fig, update, frames = frames, interval=interval, blit = True)

	#run/save animation
	if not saveName == None:
		#if not config['actor']['path'] == None:
		filename, file_extension = os.path.splitext(saveName)
		if file_extension == '.gif':
			ani.save(saveName, writer='imagemagick', fps=config['animation']['fps'])	#To save gif of animation
		if file_extension == '.mp4':
			ani.save(saveName, writer='ffmpeg', fps=config['animation']['fps'])		#To save mp4 of animation
	else:
		plt.show()


def solve(inputconfig,VERBOSE = False):
	print('Solving config...')
	START_TIME = time.process_time()
	SOLVED_CONFIG = deepcopy(inputconfig)
	END_TIME = SOLVED_CONFIG['actor']['path'][-1][-1]
	SOLVE_RESOLUTION = SOLVED_CONFIG['solve']['resolution']
	ALREADY_SOLVED_PATHS = {}
	NOT_POSSIBLE_PAIRING = set(())
	IMG = imageio.imread(SOLVED_CONFIG['map']['path'])
	IMG = [[int(abs(xpoint) > 0) for xpoint in ypoint] for ypoint in IMG]

	def isValidPathToShot(path,node):
		current = node
		if len(path) == 0:
			return False,{}
		if len(path[0]) == 4 or len(path[0]) == 5:
			path = [[element[0],element[1],element[-1]] for element in path]
		while current != None:
			for robot in current.assigned_robots:
				if type(robot.given_shot) == dict and len(robot.path) > 0:
					assigned_robot_path = [[p[0],p[1],p[-1]] for p in robot.path if robot.given_shot['start_time'] < p[-1] < robot.given_shot['end_time']]
					for test_roboPos in path:
						if len(assigned_robot_path) > 0 and assigned_robot_path[0][2] < test_roboPos[2] < assigned_robot_path[-1][2]:
							planned_pathPos,theta_planned = findPointOnPathAndSlope(assigned_robot_path,test_roboPos[2],SOLVED_CONFIG)
							fovPoly = findFovRays(theta_planned,robot.identity,planned_pathPos,SOLVED_CONFIG,robot.given_shot)[0]
							insidePoly = mplpath.Path(fovPoly).contains_points([test_roboPos[0:2]])
							if insidePoly[0]:
								#print(assigned_robot_path[0][2], test_roboPos,assigned_robot_path[-1][2])
								return False,robot.given_shot
			current = current.parent
		return True,{}

	def PRM_Modified(parent_node,robot_cfg,start,shot = None,goal = None):
		# Expects input of start to be (x,y,theta,time) and goal to be (x,y,theta)
		aStar_Timeout = float(SOLVED_CONFIG['solve']['PRM_timeout']) #seconds
		#PRM_num_points = SOLVED_CONFIG['solve']['PRM_num_points']
		class astar_node():
			"""docstring for astar_node"""
			def __init__(self, position, g = 0, parent = None):
				self.position = position
				self.parent = parent
				self.g = g
				self.h = 0
				self.f = 0
			def __lt__(self, other):
				return self.f < other.f
			def __repr__(self):
				#return 'astar_node({},{},{},{},{})'.format(self.position,self.g,self.h,self.f,self.parent)
				return self.__hash__()
			def __hash__(self):
				return hash("astar_node({})".format(self.position))
		
			def __eq__(self, other):
				return self.__hash__() == other.__hash__()
	
			def findChildren(self,point_array):
				node_list_prm =[]
	
				# Don't look for node past the goal node.
				if goal is not None and tuple(self.position) == tuple(goal):
					return []
				# Add all connections from current point to every other point to our point array so we know where our current point can lead
				for dest_point in point_array:
					dubins_path = dubins.shortest_path(self.position,dest_point, robot_cfg['max_turn_rad']).sample_many(SOLVE_RESOLUTION)[0]
					# Check to make sure the dubins path doesn't pass through obstacles.
					error = 0
					for point in dubins_path:
						pointx = int(point[0] * len(IMG[0]) / float(SOLVED_CONFIG['map']['width']))
						pointy = int(point[1] * len(IMG) / float(SOLVED_CONFIG['map']['height']))
						if 0 > pointx or pointx >= len(IMG[0]) or 0 > pointy or pointy >= len(IMG):
							error = 1
							break
						#print(point,pointx,pointy,len(IMG))
						#try:
						if not IMG[-pointy][pointx]:
							error = 1
							break
						#except Exception as e:
						#	print(pointx,pointy)
							
						
					if error == 1:
						continue
					dubins_path_length = SOLVE_RESOLUTION*len(dubins_path)
					node_list_prm.append((dubins_path_length,tuple(dest_point)))

				return_list = []
				for vertex in node_list_prm:
					return_list.append(astar_node(vertex[1],vertex[0],self))
				return return_list
				
		if goal is not None and start[-1] == shot['start_time']:
			return -1
		if VERBOSE == True:
			print('Generating PRM points...')
		#Randomly placed point.
		if SOLVED_CONFIG['solve']['PRM_points'] == 'random':
			point_array = [[random.uniform(0,SOLVED_CONFIG['map']['width']),random.uniform(0,SOLVED_CONFIG['map']['height']),random.uniform(0,2*pi)] for _ in range(SOLVED_CONFIG['solve']['PRM_num_points'])]
		# Statically placed points
		else:
			#cbrt_num_points = int((PRM_num_points + 1)**(1.0/3)) # Add one to the number of PRM points because of python rounding errors
			#width_increment = float(SOLVED_CONFIG['map']['width'])/cbrt_num_points
			#height_increment = float(SOLVED_CONFIG['map']['height'])/cbrt_num_points
			#theta_increment = 2*pi/cbrt_num_points
			#point_array = []
			#for i in range(cbrt_num_points):
			#	for j in range(cbrt_num_points):
			#		for k in range(cbrt_num_points):
			#			point_array.append([i*width_increment,j*height_increment,k*theta_increment])

			point_array = []
			for i in np.arange(SOLVED_CONFIG['solve']['PRM_point_dist']/2,SOLVED_CONFIG['map']['width'],SOLVED_CONFIG['solve']['PRM_point_dist']):
				for j in np.arange(SOLVED_CONFIG['solve']['PRM_point_dist'],SOLVED_CONFIG['map']['height'],SOLVED_CONFIG['solve']['PRM_point_dist']):
					for k in np.arange(0,2*pi,radians(SOLVED_CONFIG['solve']['PRM_degrees'])):
						pointx = int(i * len(IMG[0]) / float(SOLVED_CONFIG['map']['width']))
						pointy = int(j * len(IMG) / float(SOLVED_CONFIG['map']['height']))
						if IMG[-pointy][pointx]:
							point_array.append([i,j,k])
		#print(point_array)
		if goal is not None:
			point_array.append(goal)

		#Create a dictionary of points which have as indices all the points they connect to with the path length
		#directed_graph = {}
		#for current_point in point_array:
		#	
		#	directed_graph[tuple(current_point)] = tuple(node_list_prm)
		#
		##Then do the same for the starting node.
		#node_list_prm = []
		#for dest_point in point_array:
		#	dubins_path = dubins.shortest_path(start[0:3],dest_point, robot_cfg['max_turn_rad']).sample_many(SOLVE_RESOLUTION)[0]
		#	# Check to make sure the dubins path doesn't pass through obstacles.
		#	error = 0
		#	for point in dubins_path:
		#		pointx = int(point[0] * len(IMG[0]) / float(SOLVED_CONFIG['map']['width']))
		#		pointy = int(point[1] * len(IMG) / float(SOLVED_CONFIG['map']['height']))
		#		#print(point,pointx,pointy,len(IMG[pointy]),len(IMG))
		#		if 0 >= pointx or pointx >= len(IMG[0]) or 0 >= pointy or pointy >= len(IMG):
		#				error = 1
		#				break
		#		if not IMG[-pointy][pointx]:
		#			error = 1
		#			break
		#	if error == 1:
		#		continue
		#	dubins_path_length = SOLVE_RESOLUTION*len(dubins_path)
		#	node_list_prm.append((dubins_path_length,tuple(dest_point)))
		#directed_graph[tuple(start[0:3])] = tuple(node_list_prm)
		

		if VERBOSE == True:
			print('Beginning A* Search...')
		# Run A* on the connected points to find the lowest path length that will produce a usable path
		open_list_prm = []
		closed_list_prm = set(())
		open_list_dict_prm = {}
		# Append Starting node, and run A*
		open_list_prm.append(astar_node(start[0:3]))
		timeout = time.process_time()
		while len(open_list_prm) > 0 and time.process_time() - timeout < aStar_Timeout:
			current_node_prm = heapq.heappop(open_list_prm)
			#if goal is not None:
			#if tuple(current_node_prm.position[0:3]) != tuple(goal[0:3]):
			closed_list_prm.add(current_node_prm)
			if len(open_list_dict_prm) > 0 and current_node_prm in open_list_dict_prm:
				del open_list_dict_prm[current_node_prm]
			if goal is not None and tuple(current_node_prm.position) == tuple(goal) or goal is None: #If goal position reached, then check if valid solution and return if it is.
				current = current_node_prm
				# Print out a possible array of locations to travel avoiding obsticles (next check if it avoids shots)
				if VERBOSE == True:
					path_print = []
					while current is not None:
						path_print.append(current.position)
						current = current.parent
					print('possible solution',path_print[::-1])
				
				# Connect all points in possible array with dubins path				
				path = []
				current = current_node_prm
				if current.parent is None:
					path = [current.position]
				while current.parent is not None:
					#if SOLVED_CONFIG['solve']['algorithm'] == 'dubins':
					dubins_path = dubins.shortest_path(current.parent.position, list(current.position), robot_cfg['max_turn_rad']).sample_many(SOLVE_RESOLUTION)[0]
					path += dubins_path[::-1]
					#else:
					#	reeds_shepp_path = reeds_shepp.path_sample(current.parent.position,list(current.position),robot_cfg['max_turn_rad'],SOLVE_RESOLUTION)
					#	path += reeds_shepp_path[::-1]
					current = current.parent
				# Flip path to face forwards
				path = path[::-1]
				#print(path)
				#Add timestamps for the path
				if goal is not None:
					timestep_increment = (shot['start_time']-start[-1])/len(path)
				else:
					if len(path) == 1:
						for i in np.arange(start[-1],SOLVED_CONFIG['actor']['path'][-1][-1],SOLVED_CONFIG['actor']['timestep_resolution']):
							path.append(path[0])
					timestep_increment = (SOLVED_CONFIG['actor']['path'][-1][-1] - start[-1])/len(path)
				for pos in range(len(path)):
					if goal is not None and (pos+1)*timestep_increment + start[-1] == shot['start_time']:
						del path[pos]
						continue
					else:
						path[pos] = list(path[pos])
						path[pos].append((pos+1)*timestep_increment+start[3])
				if isValidPathToShot(path,parent_node)[0]:
					if VERBOSE == True:
						print('Suceeded: ',path)
					return path
				if VERBOSE == True:
					print('Not vaild path, keep trying...')
			else: #Otherwise we are solving for trajectories to get robot out of shot
				pass

			for child_node_prm in current_node_prm.findChildren(point_array): #For each of the children, find if it needs to be added to open_list, and do necessary
				if child_node_prm in closed_list_prm and goal is not None and tuple(child_node_prm.position) != tuple(goal):
					continue
				child_node_prm.g = current_node_prm.g + child_node_prm.g
				if goal is not None:
					child_node_prm.h = SOLVE_RESOLUTION*len(dubins.shortest_path(child_node_prm.position,goal, robot_cfg['max_turn_rad']).sample_many(SOLVE_RESOLUTION)[0])
				child_node_prm.f = child_node_prm.g + child_node_prm.h
				if child_node_prm in open_list_dict_prm:
					if child_node_prm.g >= open_list_dict_prm[child_node_prm]:
						#if goal is not None or current_node_prm.position == child_node_prm.position:
						#if child_node_prm.position[0] == 30.06477879236698:
						continue
				if goal is None and current_node_prm.position == child_node_prm.position:
					continue
				#print(child_node_prm)

				open_list_dict_prm[child_node_prm] = child_node_prm.g
				heapq.heappush(open_list_prm, child_node_prm)
		#If no solution is found by A* return -1
		if time.process_time() - timeout < aStar_Timeout:
			if VERBOSE == True:
				print('No solution to PRM')
		else:
			if VERBOSE == True:
				print('PRM timeout hit')
		return -1

	def findPathToShot(current_position,shot_start_loc,robot_cfg,parent_node,shot):

		#First check if the path from current_position to shot_start_loc with robot config has already been solved
		already_solved_identifier = (tuple(current_position[0:3]), tuple(shot_start_loc[0:3]), str(robot_cfg),tuple([str(cur_shot) for cur_shot in parent_node.available_shots]))
		if already_solved_identifier in ALREADY_SOLVED_PATHS:
			return list(ALREADY_SOLVED_PATHS[already_solved_identifier][0]),ALREADY_SOLVED_PATHS[already_solved_identifier][1]

		#Check if endpoints are not in another shot
		path = [current_position,shot_start_loc]
		valid,return_shot = isValidPathToShot(path,parent_node)
		if not valid:
			for cur_shot_num,cur_shot in enumerate(SOLVED_CONFIG['shots']):
				if cur_shot == shot:
					shot1 = cur_shot_num + 1
				if cur_shot == return_shot:
					shot2 = cur_shot_num + 1
			print('Error: Shot {} overlaps with shot {}. (shots are not zero indexed)'.format(shot1,shot2))
			#error_parent = deepcopy(parent_node)
			#path_sampled = dubins.shortest_path(current_position[0:3], shot_start_loc[0:3], robot_cfg['max_turn_rad']).sample_many(SOLVE_RESOLUTION)[0]
			#
			##Add on timesteps to dubens path
			#timestep_increment = (shot['start_time']-current_position[3])/len(path_sampled)
			#for pos in range(len(path_sampled)):
			#	path_sampled[pos] = list(path_sampled[pos])
			#	path_sampled[pos].append((pos+1)*timestep_increment+current_position[3])
			#
			#for bot in error_parent.unassigned_robots:
			#	if bot.robot_cfg == robot_cfg:
			#		save_bot = deepcopy(bot)
			#		print('removed')
			#		error_parent.unassigned_robots.remove(bot)
			#save_bot.path = path_sampled
			#save_bot.given_shot = shot
			#save_bot.position = shot_start_loc
			#error_parent.assigned_robots.append(save_bot)
			#visualize(finish(error_parent))
			ALREADY_SOLVED_PATHS[already_solved_identifier] = [],-50
			return [],-50

		#Check if a direct dubins path is valid, and if it is, just return that path
		#if SOLVED_CONFIG['solve']['algorithm'] == 'dubins':
		path = dubins.shortest_path(current_position[0:3], shot_start_loc[0:3], robot_cfg['max_turn_rad'])
		path_sampled = path.sample_many(SOLVE_RESOLUTION)[0]
		#else:
		#	path_sampled = reeds_shepp.path_sample(current_position[0:3], shot_start_loc[0:3], robot_cfg['max_turn_rad'],SOLVE_RESOLUTION)

		#Add on timesteps to dubens path
		timestep_increment = (shot['start_time']-current_position[-1])/len(path_sampled)
		delete_number = 0
		for pos in range(len(path_sampled)):
			pos = pos - delete_number
			if (pos+1)*timestep_increment + current_position[-1] == shot['start_time']:
				del path_sampled[pos]
				delete_number += 1
				continue
			else:
				path_sampled[pos] = list(path_sampled[pos])
				path_sampled[pos].append((pos+1)*timestep_increment+current_position[-1])

		cost_to_go = 1*SOLVE_RESOLUTION*len(path_sampled)

		#Check to see if direct path passes over obsticles in environment png file
		error_test = 0
		for point in path_sampled:
			pointx = int(point[0] * len(IMG[0]) / float(SOLVED_CONFIG['map']['width']))
			pointy = int(point[1] * len(IMG) / float(SOLVED_CONFIG['map']['height']))
			if 0 >= pointx or pointx >= len(IMG[0]) or 0 >= pointy or pointy >= len(IMG):
				if VERBOSE == True:
					print('Outside of boundary.')
				error_test = 1
				break
			if not IMG[-pointy][pointx]:
				if VERBOSE == True:
					print('Hit object.')
				error_test = 1
				break

		valid,return_shot = isValidPathToShot(path_sampled,parent_node)
		if error_test == 0 and valid:
			pathx = [pathx[0] for pathx in path_sampled]
			pathy = [pathy[1] for pathy in path_sampled]
			patht = [patht[-1] for patht in path_sampled]
			xp = []
			yp = []
			tp = []
			for time_path in range(len(path_sampled)):
				time_index = time_path
				if time_index == len(path_sampled) - 1:
					time_index -= 1
				xp.append(pathx[(time_index + 1)] - pathx[time_index])
				yp.append(pathy[(time_index + 1)] - pathy[time_index])
				tp.append(patht[(time_index + 1)] - patht[time_index])
			xp = np.array(xp)
			yp = np.array(yp)
			tp = np.array(tp)
			meterPerSec = sqrt(xp[time_path]**2 + yp[time_path]**2)/tp[time_path]
			if meterPerSec > robot_cfg['max_speed']:
				ALREADY_SOLVED_PATHS[already_solved_identifier] = [],-1
				if VERBOSE == True:
					print('Cannot get to shot, need speed {}.'.format(meterPerSec))
				return [],-1
			else:
				ALREADY_SOLVED_PATHS[already_solved_identifier] = (tuple(path_sampled),cost_to_go)
				return path_sampled,cost_to_go
		else:
			if VERBOSE == True:
				print('Path to get to shot {} intersects with shot {}.'.format(shot,return_shot))


		#Because regular dubins didnt solve the problem, run a modified version of PRM to actually solve the problem
		if VERBOSE == True:
			print('Starting PRM for Shot Planning...')

		#print('current_position:',current_position,'shot_start_loc',shot_start_loc)
		path_sampled = PRM_Modified(parent_node,robot_cfg,current_position,shot, shot_start_loc[0:3])
		if VERBOSE == True:
			print('Finished PRM')
		#If PRM failed, just return nothing
		if path_sampled == -1:
			ALREADY_SOLVED_PATHS[already_solved_identifier] = [],-1
			return [],-1
		else:
			pathx = [pathx[0] for pathx in path_sampled]
			pathy = [pathy[1] for pathy in path_sampled]
			patht = [patht[-1] for patht in path_sampled]
			xp = []
			yp = []
			tp = []
			for time_path in range(len(path_sampled)):
				time_index = time_path
				if time_index == len(path_sampled) - 1:
					time_index -= 1
				xp.append(pathx[(time_index + 1)] - pathx[time_index])
				yp.append(pathy[(time_index + 1)] - pathy[time_index])
				tp.append(patht[(time_index + 1)] - patht[time_index])
				meterPerSec = sqrt(xp[time_path]**2 + yp[time_path]**2)/tp[time_path]
				if meterPerSec > robot_cfg['max_speed']:
					ALREADY_SOLVED_PATHS[already_solved_identifier] = [],-1
					if VERBOSE == True:
						print('Cannot get to shot, need speed {}. Robot CFG = {}'.format(meterPerSec,robot_cfg))
					return [],-1
		cost_to_go = 1*SOLVE_RESOLUTION*len(path_sampled)
		
		ALREADY_SOLVED_PATHS[already_solved_identifier] = (tuple(path_sampled),cost_to_go)
		return path_sampled,cost_to_go

	def isFollowableShot(actor_path_followable, robot_followable,shot_followable):
		# Added 'followable' to the end of most variables to keep seperate from other functions
		radius = robot_followable['max_turn_rad']
		max_speed = robot_followable['max_speed']
		start_time_followable = shot_followable['start_time']
		end_time_followable = shot_followable['end_time']
		angle_from_actor = shot_followable['actor_facing']
		dist_from_actor = sum(shot_followable['dist_range'])/2
		error_follow = ''

		x_actor = [x_actor[0] for x_actor in actor_path_followable if start_time_followable < x_actor[-1] < end_time_followable]
		y_actor = [y_actor[1] for y_actor in actor_path_followable if start_time_followable < y_actor[-1] < end_time_followable]
		t_actor = [t_actor[-1] for t_actor in actor_path_followable if start_time_followable < t_actor[-1] < end_time_followable]

		# Find derivative of actor path
		xp = []
		yp = []
		tp = []
		for time_actor in range(len(x_actor)):
			time_index = time_actor
			if time_index == len(x_actor) - 1:
				time_index -= 1
			xp.append(x_actor[(time_index + 1)] - x_actor[time_index])
			yp.append(y_actor[(time_index + 1)] - y_actor[time_index])
			tp.append(t_actor[(time_index + 1)] - t_actor[time_index])
		xp = np.array(xp)
		yp = np.array(yp)
		tp = np.array(tp)
		
		# Define q(t) = (qx,qy)
		qx = x_actor + dist_from_actor*np.cos(np.arctan2(yp,xp)+radians(angle_from_actor))
		qy = y_actor + dist_from_actor*np.sin(np.arctan2(yp,xp)+radians(angle_from_actor))
		
		# Find first derivatives of q(t)
		qxp = []
		qyp = []
		for time_actor in range(len(qx)):
			time_index = time_actor
			if time_index == len(qx) - 1:
				time_index -= 1
			qxp.append(qx[(time_index + 1)] - qx[time_index])
			qyp.append(qy[(time_index + 1)] - qy[time_index])
		qxp = np.array(qxp)
		qyp = np.array(qyp)

		# Find second derivative of q(t)
		qxpp = []
		qypp = []
		for time_actor in range(len(qxp)):
			time_index = time_actor
			if time_index == len(qxp) - 1:
				time_index -= 1
			qxpp.append(qxp[(time_index + 1)] - qxp[time_index])
			qypp.append(qyp[(time_index + 1)] - qyp[time_index])
		qxpp = np.array(qxpp)
		qypp = np.array(qypp)

		# Finally evaluate equation to check if the shot is followable by the robot with radius of curvature equation.
		for time_actor in range(len(qx)):
			if qxp[time_actor] == 0: # if the derivative of x is 0 then flip axis to check the radius that way.
				if qyp[time_actor] == 0: # If it is stationary move on to another point
					continue
				qp = qxp[time_actor]/qyp[time_actor]
				qpp = (qyp[time_actor]*qxpp[time_actor] - qxp[time_actor]*qypp[time_actor]) / qyp[time_actor]**3
			else:
				qp = qyp[time_actor]/qxp[time_actor]
				qpp = (qxp[time_actor]*qypp[time_actor] - qyp[time_actor]*qxpp[time_actor]) / qxp[time_actor]**3

			#under_radical = (abs(qp)**2)*(abs(qpp)**2) - (qp*qpp)**2
			#if under_radical < 0:
			#	under_radical = - under_radical
			#elif under_radical == 0:
			#	return 1
			#rad_of_curve = (abs(qp)**3)/(sqrt(under_radical))
			if qpp != 0:
				rad_of_curve = (abs(1+(qp**2))**(2/3))/abs(qpp)
				#print(rad_of_curve)
				if rad_of_curve < radius:
					error_follow = 'rad'
			meterPerSec = sqrt(xp[time_actor]**2 + yp[time_actor]**2)/tp[time_actor]
			if meterPerSec > max_speed:
				error_follow = 'speed'

			if error_follow != '':
				for i,tmprobot in enumerate(SOLVED_CONFIG['robots']):
					if tmprobot == robot_followable:
						for j,tmpshot in enumerate(SOLVED_CONFIG['shots']):
							if tmpshot == shot_followable:
								if (i,j) not in NOT_POSSIBLE_PAIRING:
									if error_follow == 'rad':
										print('Warning: Robot {} cannot follow shot {} because at time {} robot {} cannot turn with turning radius {}. (shots not zero indexed)'.format(i+1,j+1,t_actor[time_actor],i+1,rad_of_curve))
									elif error_follow == 'speed':
										print('Warning: Robot {} cannot follow shot {} because at time {} robot {} cannot travel at {}m/s. (shots not zero indexed)'.format(i+1,j+1,t_actor[time_actor],i+1,meterPerSec))
									NOT_POSSIBLE_PAIRING.add((i,j))
								break
						break
				return -1
		# If all points along the path are valid return true
		return 1

	def findPathAndCost(robot_cfg, current_position, shot,parent_node):
		if shot is None:
			return [],0
		elif shot == 'finished':
			return [],0

		# Find the path to follow the actor
		following_actor = []
		cur_position = np.array([])
		time_arr = np.arange(shot['start_time'],shot['end_time'],SOLVED_CONFIG['actor']['timestep_resolution'])
		#If endpoint was missed, add it
		if not time_arr[-1] == shot['end_time']:
			time_arr = np.append(time_arr,shot['end_time'])
		for time in time_arr:
			actor_xyt,actor_slope_unitVect = findPointOnPathAndSlope(SOLVED_CONFIG['actor']['path'],time,SOLVED_CONFIG)
			#cur_position = np.multiply(rotate([0,0],actor_slope_unitVect,shot['actor_facing']),sum(shot['dist_range'])/2)+actor_xyt[0:2]
			cur_position = np.multiply(rotate([0,0],actor_slope_unitVect,-(sum(shot['angle_range'])/2) + shot['actor_facing']),sum(shot['dist_range'])/2)+actor_xyt[0:2]
			cur_position = np.append(cur_position, atan2(actor_slope_unitVect[1],actor_slope_unitVect[0]))
			cur_position = np.append(cur_position, time)
			# Check if the path the robot is taking is possible with its turning constraints.

			following_actor.append(cur_position.tolist())

		# Check that following the actor is possible with the configuration on the robot
		followable = isFollowableShot(following_actor,robot_cfg,shot)
		if not followable == 1:
			return [],-1

		#calculate ctg from starting position to shot start
		actor_xyt,actor_slope_unitVect = findPointOnPathAndSlope(SOLVED_CONFIG['actor']['path'],shot['start_time'],SOLVED_CONFIG)
		#shot_start_loc = np.multiply(rotate([0,0],actor_slope_unitVect,shot['actor_facing']),sum(shot['dist_range'])/2)+actor_xyt[0:2]
		shot_start_loc = np.multiply(rotate([0,0],actor_slope_unitVect,-(sum(shot['angle_range'])/2) + shot['actor_facing']),sum(shot['dist_range'])/2)+actor_xyt[0:2]
		shot_start_loc = np.append(shot_start_loc, atan2(actor_slope_unitVect[1],actor_slope_unitVect[0]))
		shot_start_loc = np.append(shot_start_loc, shot['start_time'])
		#print(shot_start_loc)

		#Find a path not obstructed by obsticles, and not obstructed by shots to the start of the shot.
		path_sampled,cost_to_go = findPathToShot(current_position,shot_start_loc,robot_cfg,parent_node,shot)
		if cost_to_go == -1:
			return [],-1
		elif cost_to_go == -50:
			return[],-50


			#Then create the circle through the two vectors at given solve resolution
			
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
		#	window_size, poly_order = int(2.0/SOLVED_CONFIG['solve']['time_resolution'])+1, 3
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

	def finish(current_node):
		robot_paths = []
		for robot in SOLVED_CONFIG['robots']:
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
					robot_paths[robot.identity].append(position)
			current = current.parent
		for element in range(0,len(robot_paths)):
			SOLVED_CONFIG['robots'][element]['path'] = robot_paths[element][::-1]
			if SOLVED_CONFIG['robots'][element]['path'] != [] and SOLVED_CONFIG['robots'][element]['path'][-1][3] != END_TIME:
				if VERBOSE == True:
					print('Starting PRM to keep Robot out of Shots...') # Run PRM_Modified over possible robot locations with the goal of A* as the end time. (Don't care where robot ends up)
				roboavoid = PRM_Modified(current_node,robot.robot_cfg,SOLVED_CONFIG['robots'][element]['path'][-1])
				if not roboavoid == -1:
					SOLVED_CONFIG['robots'][element]['path'].extend(roboavoid)
				if VERBOSE == True:
					print('Finished PRM')
					SOLVED_CONFIG['robots'][element]['path'] = [[p[0],p[1],p[3]] for p in SOLVED_CONFIG['robots'][element]['path']]
			if SOLVED_CONFIG['robots'][element]['path'] != [] and SOLVED_CONFIG['robots'][element]['path'][-1][2] < END_TIME:
				SOLVED_CONFIG['robots'][element]['path'].append(SOLVED_CONFIG['robots'][element]['path'][-1][0:2] + [END_TIME])
				#print('adding time')
			#print(SOLVED_CONFIG['robots'][element]['path'])	#debug
		for robotNum in range(0,len(SOLVED_CONFIG['robots'])):
			if SOLVED_CONFIG['robots'][robotNum]['path'] == []:
				del SOLVED_CONFIG['robots'][robotNum]['path']
		#return with solution
		SOLVED_CONFIG['solve']['processing_time'] = time.process_time() - START_TIME
		print('Solution Found in {} seconds.'.format(SOLVED_CONFIG['solve']['processing_time']))
		return SOLVED_CONFIG

	class Robot():

		def __init__(self, identity, position ,given_shot = None,parent_node = None):
			self.position = position
			self.identity = identity
			self.robot_cfg = SOLVED_CONFIG['robots'][identity]
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
					if new_robot.cost_to_go == -50:
						return -50
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
				END_TIME_arr = [robot.given_shot['end_time'] for robot in new_node.assigned_robots if type(robot.given_shot) == dict]
				if len(END_TIME_arr) == 0:
					END_TIME_arr.append(0)
				min_END_TIME = min(END_TIME_arr)
				#print("given_shot")
				#for robot in self.assigned_robots:
				#	print(robot.given_shot)
				#print('allRobots:')
				#for robot in self.robots:
				#	print(robot.given_shot)
				#print("Min end time: ",min_END_TIME)
				
				new_node.time = min_END_TIME
				for robot in new_node.assigned_robots:
					if not type(robot.given_shot) == dict:
						continue
					elif robot.given_shot['end_time'] == min_END_TIME:
						robot.given_shot = None
						new_node.unassigned_robots.append(robot)
						new_node.assigned_robots.remove(robot)
				
				return [new_node]
				

		def __lt__(self, other):
			return self.f < other.f

		#def __repr__(self):
		#	return "Node({}{})".format([robot.__hash__() for robot in self.robots],self.time)

		def __hash__(self):
			#if self.parent != None:
			#	return hash("Node({}{}{})".format([robot.__hash__() for robot in self.robots],self.time,self.parent.__hash__()))
			#else:
			return hash("Node({}{}{})".format([robot.__hash__() for robot in self.robots],self.time,self.available_shots))
		def __eq__(self, other):
			return self.__hash__() == other.__hash__()


	#A* algorithm
	#Initialize lists/heaps/dictionaries and endtime
	open_list = []
	heapq.heapify(open_list)
	open_list_dict = {}
	closed_list = set(())
	shot_end_time = max([shot['end_time'] for shot in SOLVED_CONFIG['shots']])

	#Add first robot to the heap
	startRobotList = []
	for robotNumber,robot in enumerate(SOLVED_CONFIG['robots']):
		robot['start_coord'].append(0)
		startRobotList.append(Robot(robotNumber,robot['start_coord'],None,None))

	open_list.append(Node(startRobotList,SOLVED_CONFIG['shots'],0,None))

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
		if len(open_list_dict) > 1:
			del open_list_dict[current_node]
		if VERBOSE == True:
			print('Current robot:')
			for robot in current_node.unassigned_robots:
				print('robot:',robot.identity, robot.given_shot)
			for robot in current_node.assigned_robots:
				print('robot:',robot.identity, robot.given_shot,robot.cost_to_go)
		closed_list.add(current_node)


		if current_node.time == shot_end_time and len(current_node.available_shots) == 0: #If the node has reached the end time, then finish and return the paths
			return finish(current_node)
		
		all_children = current_node.findChildren()
		if all_children == -50:
			return -1
		for child_node in all_children: #For each of the children, find if it needs to be added to open_list, and do necessary
			if (child_node in closed_list or child_node in open_list):
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
	return -1


if __name__ == '__main__':
	#read config file
	#config = yaml.safe_load(open('result_working.yaml', 'r'))
	#visualize(config)
	config = yaml.safe_load(open('rocky.yaml','r'))
	SOLVED_CONFIG = solve(config,VERBOSE = False)
	with open('errors.yaml', 'w') as yaml_file:
		yaml.dump(SOLVED_CONFIG, yaml_file, default_flow_style=False)
	#visualize(SOLVED_CONFIG)
	#config = yaml.safe_load(open('result_working.yaml', 'r'))
	#visualize(config)
	#config = yaml.safe_load(open('result_working2.yaml', 'r'))
	visualize(SOLVED_CONFIG,show_path = True,saveName = None)
	#config = yaml.safe_load(open('result.yaml', 'r'))
	#visualize(config,'demo.gif')