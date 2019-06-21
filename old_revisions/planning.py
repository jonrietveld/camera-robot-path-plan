#!/usr/bin/python3

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import collections
import imageio
import numpy as np
import yaml
		

def visualize(imgLocation,segments=None,fps=3,saveName=None):
	#Change these colors for lines 1,2,.... in RGBA notation
	lineColors = [(1,0,0,1),(0,1,1,1)]
	#Change color of connecting lines in RGBA notation
	connectColor = (0,0,0,.5)
	#Change Animation speed in frames per second
	def update(i):
		col = []
		tmpSegment = []
		for x in range(len(segments)):
			col.append(segments[x][0:i+1])
		if not len(segments)==1:
			for x in range(len(segments)-1):
				for y in range(i+1):
					if len(segments[x+1])>y:
						tmpSegment.append(segments[x][y])
						tmpSegment.append(segments[x+1][y])
						col.append((segments[x][y],segments[x+1][y]))
			scat.set_offsets(tmpSegment)
			lineCol.set_paths(col)
			return lineCol,scat
		else:
			for x in range(i+1):
				tmpSegment.append(segments[0][x])
			scat.set_offsets(tmpSegment)
			lineCol.set_paths(col)
			return lineCol,scat
	#Import the image
	img1 = imageio.imread(imgLocation)
	#print(img.shape) #Debug
	

	fig, ax = plt.subplots()
	ax.imshow(img1,cmap="gray",extent=[0, img1.shape[1], 0, img1.shape[0]])
	if not segments==None:
		c = lineColors
		for x in range(len(segments[0])):
			c.append(connectColor)
		lineCol = collections.LineCollection([], linewidths=2,colors=c)
		ax.add_collection(lineCol)
		scat = ax.scatter([], [],c='white',edgecolors='black')
		#scat.edgecolors('white')
		ani = animation.FuncAnimation(fig, update, frames = len(segments[0]), interval=1000/fps, blit=True)
	if not saveName == None:
		ani.save(saveName, writer='imagemagick', fps=fps) #To save gif of animation
	else:
		plt.show()

	
#img = imageio.imread(imgLocation)
#img[img==255] = 1

lineSegArr = [[(70,234),(97,177),(107,77),(99,60)],[(54,237),(85,185),(100,85),(95,65)]]

#print(lineSegArr[0][0])


read = file('document.yaml', 'r')
mydata = yaml.safe_load(read)
#print(mydata['actorPointArr'])
visualize('simple_rooms.png',lineSegArr,fps = 3)