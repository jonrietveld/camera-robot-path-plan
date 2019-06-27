#!/usr/bin/python3
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import collections
import matplotlib.patches as patches
import matplotlib.path as mplpath
import imageio
import numpy as np
import yaml
import decimal
from math import *
from copy import *
import heapq
import time
import dubins
import itertools

'''Shared functions'''
#######################################################################################################################
#find point on segment joining two closest points in array given. Find this point at given time.
def findPointOnPathAndSlope(pathArr,time):
	#find closest point time on path below given time
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

#Rotate a point counterclockwise by a given angle around a given origin.
def rotate(origin, point, angle):
	angle = radians(angle)
	ox, oy = origin
	px, py = point
	qx = ox + cos(angle) * (px - ox) - sin(angle) * (py - oy)
	qy = oy + sin(angle) * (px - ox) + cos(angle) * (py - oy)
	return qx, qy


#Find the polygon for the shot of the rpobot
def findShotPolygon(roboLoc,shot,camDir_unit_vect):
	minShotDistVect = np.array(camDir_unit_vect) * shot['dist_range'][0]
	maxShotDistVect = np.array(camDir_unit_vect) * shot['dist_range'][1]
	angle_values = np.arange(shot['angle_range'][0], shot['angle_range'][1], 5)
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
#######################################################################################################################

def visualize(config, saveName=None):
	def findFovRays(slope,robot,rotatePoint):
		camDirectionUnit = rotate([0,0],slope,config['robots'][robot]['camera_orientation'])
		camDirection = (np.array(camDirectionUnit) / (np.array(camDirectionUnit)**2).sum()**0.5)*(config['map']['height']+config['map']['width'])**2 #Make unit vector longer
		fovSlopes = [rotate([0,0],camDirection,config['robots'][robot]['fov']/2),rotate([0,0],camDirection,-config['robots'][robot]['fov']/2)]
		
		fov = config['robots'][robot]['fov']
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
	#read image and create plot
	img1 = imageio.imread(config['map']['path'])
	fig, ax = plt.subplots()
	ax.set(xlim=(0, config['map']['width']), ylim=(0, config['map']['height']))
	ax.imshow(img1,cmap="binary",extent=[0, config['map']['width'], 0, config['map']['height']]) #gray

	#Create plot points and segments for each robots. Create the point for the actor. Create the Field of View for the robots.
	roboLocArr = []
	segmentArr = []
	fovFill = []
	roboShot = []
	for robot in config['robots']:
		segment, = ax.plot([],[],c=(0,0,0,.5))
		pt, = ax.plot([],[],marker='o',c=robot['color'])
		fovFill.append(patches.Polygon([[0,0],[0,0],[0,0]],closed=True, alpha=.25, fc=robot['color'], ec='None'))
		ax.add_patch(fovFill[len(fovFill)-1])
		for configShot in config['shots']:
			roboShot.append(patches.Polygon([[0,0],[1,1],[2,2],[5,5]],closed=True, alpha=.25, fc=config['animation']['in_shot'], ec='None'))
			ax.add_patch(roboShot[len(roboShot)-1])
			roboShot.append(patches.Polygon([[0,0],[1,1],[2,2],[5,5]],closed=True, alpha=.25, fc=config['animation']['out_shot'], ec='None'))
			ax.add_patch(roboShot[len(roboShot)-1])
		roboLocArr.append(pt)
		segmentArr.append(segment)
	actorPt, = ax.plot(0,0,marker='o',c=config['actor']['color'])

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

def solve_old(inputconfig):
	solved_config = deepcopy(inputconfig)
	class Node():
		"""A node class for A* Pathfinding"""
	
		def __init__(self, parent=None, position=None):
			self.parent = parent
			self.position = position							#(x,y,theta,time)
	
			self.g = 0
			self.h = 0
			self.f = 0
	
		def __eq__(self, other):
			return self.position == other.position

		def __repr__(self):
			return "Node(position={})".format(self.position)

		def __hash__(self):
			return hash(self.__repr__())

		def __lt__(self, other):
			return self.f < other.f
	


	

	#Account for velocity, acceleration, turn radius, walls, actor location, and other robots paths. Use solved_config
	def findPossiblePositions(node,solved_config,maze): 
		# Make sure walkable terrain
		maxAngle = 120
		posArr = []
		one_timestep = solved_config['solve']['time_resolution']
		for xpoint in np.linspace(-1.75,1.75,1.75/solved_config['solve']['map_resolution'],endpoint=False):
			for ypoint in np.linspace(-1.75,1.75,1.75/solved_config['solve']['map_resolution'],endpoint=False):
				#print(xpoint,ypoint)
				xpoint_map = int((xpoint + node.position[0]) * (len(maze[0])/solved_config['map']['width']))
				ypoint_map = int((ypoint + node.position[1]) * (len(maze)/solved_config['map']['height']))
				if xpoint == ypoint == 0:
						continue
				def findtheta(xdist,ydist):
					if xdist == 0 and ydist > 0:
						theta = pi/2
					elif xdist ==0 and ydist < 0:
						theta = 3*pi/2
					else:
						theta = atan(ydist/xdist)
					return theta
				if hasattr(node,'position') and hasattr(node.parent,'position'):
					theta_dif = abs(findtheta(xpoint,ypoint) - findtheta(node.parent.position[0],node.parent.position[1]))
					if theta_dif > radians(maxAngle):
						#print(theta_dif)
						continue
				#if isnan(theta):
				#	continue
				if xpoint_map >= len(maze[0]) or ypoint_map >= len(maze):
					continue
				if maze[ypoint_map,xpoint_map]: #if the point is in an open space then use it 
					#print(xpoint,ypoint)
					posArr.append([xpoint + node.position[0],ypoint + node.position[1],[xpoint,ypoint],node.position[3] + solved_config['solve']['time_resolution']])
		return posArr

	#Account for euclidean distance of node from actor, using 0 for distances within distance range of either shot. Use solved_config for actor
	def hueristic(node,solved_config,end_time,scaling): #dist from actor
		node_xy = node.position[0:2]
		node_time = node.position[3]
		actor_xy,slope = findPointOnPathAndSlope(solved_config['actor']['path'],node_time)
		dist_from_actor = hypot(node_xy[0] - actor_xy[0],node_xy[1] - actor_xy[1])
		time_to_end = end_time - node.position[3]
		if dist_from_actor<=2:
			dist_from_actor = 0
		#print(dist_from_actor,time_to_end)
		return (time_to_end*2 + dist_from_actor*scaling[0] + solved_config['solve']['time_resolution']) #Now non-Admissable

	#Account for distance from obsticles, keeping actor in center of viewing area, possibly maintaining constant acceleration.
		#Use solved_config for actor location and viewing area.
	def lossFunction(node,solved_config,scaling):
		not_catching_shot_loss = 1000

		loss = solved_config['solve']['time_resolution'] #loss to move 1 positon in one time step

		x,y = round(node.position[0],10),round(node.position[1],10)
		actor_xy,actor_slope = findPointOnPathAndSlope(solved_config['actor']['path'],node.position[3])
		node_vect_theta = node.position[2] #[cos(node.position[2]),sin(node.position[2])]
		node_cam_dir = rotate([0,0],node_vect_theta,solved_config['robots'][0]['camera_orientation'])			#change for more than one robot
		length = hypot(node_cam_dir[0],node_cam_dir[1])
		node_cam_dir = [node_cam_dir[0]/length,node_cam_dir[1]/length]

		#Add loss for not staying constant accel
		parent_node_length = hypot(node.parent.position[2][0],node.parent.position[2][1])
		difference = abs(parent_node_length - length) #Camera length is same as robot length
		loss += difference



		#Add loss for actor outside of viewing area of robot
		for shot in solved_config['shots']:
			if shot['start_time'] <= node.position[3] <= shot['end_time']:
				shotarr = findShotPolygon([x,y],shot,node_cam_dir)
				pathObject = mplpath.Path(shotarr)
				insidePoly = pathObject.contains_points([actor_xy[0:2]])
				if insidePoly:
					center_polygon = [node_cam_dir[0]*(shot['dist_range'][1] - shot['dist_range'][0]),node_cam_dir[1]*(shot['dist_range'][1] - shot['dist_range'][0])]
					center_polygon = [center_polygon[0] + x, center_polygon[1] + x]
					dist_from_center = hypot(center_polygon[0] - actor_xy[0], center_polygon[1] - actor_xy[1])
					#print(loss)
					#if not dist_from_actor < (.1 * (shot['dist_range'][1] - shot['dist_range'][0])):
					loss += dist_from_center*scaling[0] + solved_config['solve']['time_resolution']
					#print(loss)																			#change , magic number
				#	length = hypot(node.position[0] - node.parent.position[0],node.position[1] - node.parent.position[1])
				#	print(node_cam_dir,[(node.position[0]-node.parent.position[0])/length,(node.position[1]-node.parent.position[1])/length])
				if not insidePoly:
					loss += not_catching_shot_loss
					#print("not Inside")
		#print(node.position[3],loss)
		#if loss < 0:
		#	print(loss)
		return loss


	def astar(maze, start, solved_config):
		"""Returns a list of tuples as a path from the given start to the given end in the given maze"""
	
		# Create start and end node
		start_node = Node(None, start)
		start_node.g = start_node.h = start_node.f = 0
		#end_node = Node(None, end)
		#end_node.g = end_node.h = end_node.f = 0
	
		# Initialize both open and closed list
		open_list = []
		heapq.heapify(open_list)
		open_list_dict = {}

		closed_list = set(())
		end_time = solved_config['actor']['path'][len(solved_config['actor']['path'])-1][2]
	
		# Add the start node
		#open_list.append(start_node)
		#open_list.push(start_node)
		heapq.heappush(open_list, start_node)
		open_list_dict[start_node] = []
		heapq.heappush(open_list_dict[start_node],start_node.g)
		#open_list_set.add(start_node)
	
		# Loop until you find the end
		told = time.time()
		itts = 0
		scaling = [(1/hypot(solved_config['map']['height'],solved_config['map']['width'])),end_time/(end_time+1)]
		while len(open_list) > 0:
	
			# Get the current node
			current_node = heapq.heappop(open_list)
			open_list_dict.pop(current_node,None)

			if tuple([current_node.position[0],current_node.position[1],tuple(current_node.position[2]),current_node.position[3]]) in closed_list:
				continue
			current_index = 0
			#tmpNode = open_list_set.
			#for index, item in enumerate(open_list):
			#	#print(item.f)
			#	if item.f < current_node.f:
			#		current_node = item
			#		current_index = index
			#print(current_index)
			tnew = time.time()
			itts +=1
			if tnew-told > .5:
				print("x = {:<15}y = {:<15}theta = {:>15},{:<15}time = {:<15}g = {:<15}h = {:<15}f = {:<15}itt = {}".format(current_node.position[0],current_node.position[1],current_node.position[2][0],current_node.position[2][1],current_node.position[3], current_node.g, current_node.h,current_node.f,itts))
				told = tnew
				itts = 0
			# Pop current off open list, add to closed list
			#open_list.pop(current_index)
			#open_list_set.remove(current_node)
			closed_list.add(tuple([current_node.position[0],current_node.position[1],tuple(current_node.position[2]),current_node.position[3]]))

			
	
			# Found the goal - the time has ended
			if current_node.position[3] >= end_time:	#changed
				path = []
				current = current_node
				while current is not None:
					#print(type(current.position[0]))
					path.append([float(current.position[0]),float(current.position[1]),float(current.position[3])])
					current = current.parent
				return path[::-1] # Return reversed path
	
			# Generate children
			children = []
			allPositions = findPossiblePositions(current_node,solved_config,maze)
			#print(allPositions)
			#input()
			for new_position in allPositions: # Adjacent squares	 #changed
	
				# Get node position
				#node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1], current_node.position[2] + new_position[3],current_node.position[3] + new_position[3])		#changed
				node_position = new_position
				# Make sure within range - handled by findPossiblePositions()
				#if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
				#	continue
	
				# Create new node
				new_node = Node(current_node, node_position)
	
				# Append
				children.append(new_node)
	
			# Loop through children
			for child in children:
	
				# Child is on the closed list
				#for closed_child in closed_list:
				#	if child == closed_child:
				#		continue
				if tuple([child.position[0],child.position[1],tuple(child.position[2]),child.position[3]]) in closed_list:
					continue


				child.position = [round(child.position[0],10),round(child.position[1],10),child.position[2],round(child.position[3],10)]
				# Create the f, g, and h values
				child.g = round(current_node.g + lossFunction(child,solved_config,scaling),10)									#changed
				child.h = round(hueristic(child,solved_config,end_time,scaling),10)												#changed
				child.f = round(child.g + child.h,10)
				if child.f < 0 or child.g < 0 or child.h < 0:
					print(child.g, child.h, child.f)
				#if child.g > 10000:																						#debug
				#	child.g = 10000
	
					# Child is already in the open list
					#for open_node in open_list:
					#	if child == open_node and child.g > open_node.g:
					#		continue

				if child in open_list_dict:
					if child.g >= open_list_dict[child]:
						continue
				else:
					open_list_dict[child] = []

					#if child in open_list_set:
					#	pass
	
				# Add the child to the open list
					#index = binarySearch(open_list,child.f)
					#open_list.insert(index[1],child)

				open_list_dict[child] = child.g
				if child.g < 1000:
					heapq.heappush(open_list, child)
				

				#print(open_list_dict)
		#If no solution found print that
		print('No Solution Found')

	maze = imageio.imread(config['map']['path'])

	print(maze[50])
	start = [5, 7, [1,0], 0]

	#path = astar(maze, start, solved_config)
	solved_config['robots'][0]['path'] = astar(maze, start, solved_config)
	#print(path)
	#for element in solved_config['robots']:
	#	element['path'] = [[54,237,0],[85,185,5],[100,85,7],[95,65,8]]
	#solved_config['robots'][0]['path'] = [[5,24,0],[6,20,1],[10,18,2],[12,15,3],[16,14,3.5],[19,5,5],[22,7,6],[20,10,7],[17,11,7.78],[17,13,8],[21,15,9],[29,14,9.78],[30,13,10],[35,5,11]]
	#solved_config['robots'][1]['path'] = [[6,24,0],[5,22,1.1],[9,18,1],[9,15,2],[16,14,3],[17,5,4],[22,5,5],[22,9,6],[17,12,7],[18,15,8],[28,14,9],[33,7,10],[34,7,12]]
	return solved_config


def solve(inputconfig):
	solved_config = deepcopy(inputconfig)
	solve_resolution = solved_config['solve']['map_resolution']
	
	def findPathAndCost(robot_cfg, current_position, shot):
		#calculate ctg from starting position to shot start
		if shot is None:
			return [],0
		elif shot == 'idle':
			return [],0

		actor_xyt,actor_slope_unitVect = findPointOnPathAndSlope(solved_config['actor']['path'],shot['start_time'])
		shot_start_loc = np.multiply(rotate([0,0],actor_slope_unitVect,(sum(shot['angle_range'])/2)),sum(shot['dist_range'])/2)+actor_xyt[0:2]
		shot_start_loc = np.append(shot_start_loc, atan2(actor_slope_unitVect[1],actor_slope_unitVect[0]))
		
		path = dubins.shortest_path(current_position[0:3], shot_start_loc, robot_cfg['max_turn_rad'])
		path_sampled = path.sample_many(solve_resolution)
		path_sampled = path_sampled[0]
		cost_to_go = 1*solve_resolution*len(path_sampled)

		#Add on timesteps to dubens path
		timestep_increment = (shot['start_time']-current_position[3])/len(path_sampled)
		#print(shot['end_time'],shot['start_time'])
		#print(len(path_sampled))
		for pos in range(len(path_sampled)):
			if (pos+1)*timestep_increment + current_position[3] == shot['start_time']:
				del path_sampled[pos]
				continue
			else:
				path_sampled[pos] = list(path_sampled[pos])
				path_sampled[pos].append((pos+1)*timestep_increment+current_position[3])
			#print(path_sampled[pos])

		#Add cost to go for traveling too fast on dubins path
		if len(path_sampled)>1:
			speed = hypot(path_sampled[0][0] - path_sampled[1][0],path_sampled[0][1] - path_sampled[1][1])/timestep_increment
			if speed > robot_cfg['speed']:
				cost_to_go += 1000000			#Change fix please
			
		
		#calculate ctg following actor
		following_actor = []
		current_position = np.array([])
		#If endpoint was missed, add it
		time_arr = np.arange(shot['start_time'],shot['end_time'],solved_config['solve']['time_resolution'])
		if not time_arr[-1] == shot['end_time']:
			time_arr = np.append(time_arr,shot['end_time'])
		for time in time_arr:
			actor_xyt,actor_slope_unitVect = findPointOnPathAndSlope(solved_config['actor']['path'],time)
			current_position = np.multiply(rotate([0,0],actor_slope_unitVect,(sum(shot['angle_range'])/2)),sum(shot['dist_range'])/2)+actor_xyt[0:2]
			current_position = np.append(current_position, atan2(actor_slope_unitVect[1],actor_slope_unitVect[0]))
			current_position = np.append(current_position, time)
			following_actor.append(current_position.tolist())
			#Add cost for being oustide of shot polygon
			robot_cam_dir = rotate([0,0],actor_slope_unitVect,robot_cfg['camera_orientation'])
			shot_poly = findShotPolygon(current_position,shot,robot_cam_dir)
			pathObject = mplpath.Path(shot_poly)
			insidePoly = pathObject.contains_points([actor_xyt[0:2]])
			if not insidePoly:
				cost_to_go += 500	#fix please

		#Add cost to go for each segment in the path next to the actor in a given shot
		#for loc in range(1,len(following_actor)):
		#	cost_to_go += hypot(following_actor[loc][0] - following_actor[loc-1][0],following_actor[loc][1] - following_actor[loc-1][1])

		#return total path, and cost to go
		#print(path_sampled)
		total_path = path_sampled + following_actor
		return total_path,cost_to_go

	class Robot():

		def __init__(self, identity, position ,given_shot = None):
			self.position = position
			self.identity = identity
			self.robot_cfg = solved_config['robots'][identity]
			self.given_shot = given_shot
			self.path, self.cost_to_go = findPathAndCost(self.robot_cfg,position,given_shot)
			if given_shot is None:
				self.end_position = position
			elif given_shot == 'idle':
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

		def findChildren(self):


			node_list = []
			
			#If there are unassigned robots, then find all permutations of shots with robots possible
			if len(self.unassigned_robots) > 0:
				#If there are more or equal unassigned robots than available shots, find all permutations of robots to assign shots too.
				available_shots = deepcopy(self.available_shots)
				for robot in self.unassigned_robots:
					available_shots.append('idle')

				shotPermutations = list(itertools.permutations(available_shots,len(self.unassigned_robots)))
				#print(shotPermutations)
				for perm in shotPermutations:	# shotPermutations looks like ((shot1,shot2),(shot1,shot3),(shot2,shot1),(shot2,shot3)...)
					robot_list = []
					unassigned_shots = deepcopy(available_shots)
					unassigned_robots = deepcopy(self.unassigned_robots)
					for shot in perm:
						robot = unassigned_robots[0]
						if type(shot) == dict and robot.end_position[3] > shot['start_time']:
							robot_list.append(Robot(robot.identity,robot.end_position,'idle'))
						else:
							robot_list.append(Robot(robot.identity,robot.end_position,shot))
							unassigned_shots.remove(shot)
						unassigned_robots.remove(robot)

					#for robot in unassigned_robots:
					#	robot_list.append(Robot(robot.identity,robot.end_position,'idle'))
					for shot in list(unassigned_shots):
						if shot == 'idle':
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
					if robot.given_shot['end_time'] == min_end_time:
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

	#return findPathAndCost(solved_config['robots'][0],solved_config['robots'][0]['path'][0][0:2] + [0, solved_config['robots'][0]['path'][0][2]],solved_config['shots'][0])
	
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
		startRobotList.append(Robot(robotNumber,robot['start_coord'],None))

	open_list.append(Node(startRobotList,solved_config['shots'],0,None))

	#While the open_list has nodes in it, keep running
	while len(open_list) > 0:
		#print("Open List Length: ",len(open_list)) #debug
		#for node in open_list:
		#	print(len(node.unassigned_robots))
		#	for robot in node.unassigned_robots:
		#		print(robot.identity, robot.given_shot)
		#Remove one node from the heap and add it to the closed_list
		current_node = heapq.heappop(open_list)
		closed_list.add(current_node)


		if current_node.time == end_time and len(current_node.available_shots) == 0: #and current_node.f<10000000: #If the node has reached the end time, then finish and return the paths
			print('Solution Found')
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
			return solved_config
		
		for child_node in current_node.findChildren(): #For each of the children, find if it needs to be added to open_list, and do necessary
			if child_node in closed_list:
				continue
			child_node.g = current_node.g + child_node.total_ctg		#fix please
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
#t0 = time.time()
solved_config = solve(config)
#tmpvar = [element[0:2] + [element[3]] for element in tmpvar]
#tmpvar.sort(key=lambda position: position[2])
#config['robots'][0]['path'] = tmpvar
#config['robots'][1]['path'] = None
#print(config['robots'][0]['path'])
#t1 = time.time()
#print("Total time: {}".format(t1-t0))
#with open('result.yaml', 'w') as yaml_file:
#	yaml.dump(solved_config, yaml_file, default_flow_style=False)
#visualize(solved_config)
#config = yaml.safe_load(open('result_working.yaml', 'r'))
#visualize(config)
#config = yaml.safe_load(open('result_working2.yaml', 'r'))
visualize(solved_config)
#config = yaml.safe_load(open('result.yaml', 'r'))
#visualize(config,'demo.gif')