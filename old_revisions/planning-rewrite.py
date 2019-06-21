#!/usr/bin/python3

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import collections
import imageio
import numpy as np
import yaml


print("JASH = 1.75")

def visualize(yamlFile,roboArr = None, saveName=None):
	def LinePlaneCollision(planeNormal, planePoint, rayDirection, rayPoint, epsilon=1e-6):
		ndotu = planeNormal.dot(rayDirection)
		if abs(ndotu) < epsilon:
			raise RuntimeError("no intersection or line is within plane")
 	
		w = rayPoint - planePoint
		si = -planeNormal.dot(w) / ndotu
		Psi = w + si * rayDirection + planePoint
		return Psi
	#find point on segment joining two closest points in array given. Find this point at given time.
	def findPointOnSegment(someArr,time):
		#find closest point in time
		#someArr.sort(key=lambda x: x[2])
			#remove duplicate values here
		closeTime = someArr.index(min(someArr,key=lambda x: abs(x[2]-time)))
		if time - someArr[closeTime][2] < 0:				#finds point below given time
			closeTime = closeTime - 1
		elif someArr[closeTime][2] == time:
			return [someArr[closeTime][0],someArr[closeTime][1]]


		planeNormal = np.array([0,0,1])
		planePoint = np.array([0, 0, time]) #Any point on the plane
 		 			
		#Define ray
		rayDirection = np.array([someArr[closeTime+1][0]-someArr[closeTime][0],someArr[closeTime+1][1]-someArr[closeTime][1],someArr[closeTime+1][2]-someArr[closeTime][2]])
		rayPoint = np.array(someArr[closeTime]) #Any point along the ray
		intersect = LinePlaneCollision(planeNormal, planePoint, rayDirection, rayPoint)
		return intersect
		


	def update(i):
		#print(i)
		if not roboArr == None:
			tmpPt1 = findPointOnSegment(roboArr,(1.0/config['animation']['fps'])*i) #findPointOnPath
			pt2.set_data(tmpPt1[0],tmpPt1[1])
			tmpPt2 = findPointOnSegment(config['actorPointArr'],(1.0/config['animation']['fps'])*i)
			pt1.set_data(tmpPt2[0],tmpPt2[1])
			segment.set_data([tmpPt1[0],tmpPt2[0]],[tmpPt1[1],tmpPt2[1]])
			return segment,pt1,pt2
		else:
			tmpPt = findPointOnSegment(config['actorPointArr'],(1.0/config['animation']['fps'])*i)
			pt1.set_data(tmpPt[0],tmpPt[1])
			return pt1,

	
	#read config file
	read = open(yamlFile, 'r')
	config = yaml.safe_load(read)
	
	#read image and create plot
	img1 = imageio.imread(config['map']['path'])
	#plt.axes([0, config['map']['width'], 0, config['map']['height']])
	fig, ax = plt.subplots() #1,1,figsize=(config['map']['width'],config['map']['height']))
	ax.set(xlim=(0, config['map']['width']), ylim=(0, config['map']['height']))
	ax.imshow(img1,cmap="gray",extent=[0, config['map']['width'], 0, config['map']['height']])
	if not roboArr == None:
		segment, = ax.plot([0,0],[0,0],c=(0,0,0,.5))
		pt2, = ax.plot(0,0,marker='o',c='blue')
	pt1, = ax.plot(0,0,marker='o',c='red')
	#print(findPointOnSegment(config['actorPointArr'],6.5))
	maxtime = max(config['actorPointArr'],key=lambda x: x[2])
	
	if not config['actorPointArr']==None:
		interval=1000.0/config['animation']['fps']
		frames = config['animation']['fps']*maxtime[2]
		print("frames: " + str(frames))
		print("interval: "+ str(interval))
		ani = animation.FuncAnimation(fig, update, frames = frames, interval=interval,blit = True)

	#run/save animation
	if not saveName == None:
		ani.save(saveName, writer='imagemagick', fps=config['animation']['fps']) #To save gif of animation
		#Writer = animation.writers['ffmpeg']
		#writer = Writer(fps=config['animation']['fps'], metadata=dict(artist='Me'), bitrate=1800)
		#ani.save(saveName, writer=writer)
	else:
		plt.show()


visualize('document.yaml')
visualize('document.yaml',[[54,237,0],[85,185,5],[100,85,7],[95,65,8]])