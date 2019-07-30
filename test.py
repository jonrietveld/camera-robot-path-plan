import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import dubins

fig, ax = plt.subplots()
ax.axis('equal')

distance = 2
theta = np.pi/2
path = dubins.shortest_path([0,0,np.pi/2],[5,2,np.pi], 1).sample_many(.1)[0]
t = np.arange(0,2*np.pi,.05)
#x = np.cos(t)
#y = np.sin(t)
x = [x3[0] for x3 in path]
y = [y3[1] for y3 in path]

xp = []
yp = []
for time in range(len(x)):
	time_index = time
	if time_index == len(x) - 1:
		time_index -= 1
	xp.append(x[(time_index + 1)] - x[time_index])
	yp.append(y[(time_index + 1)] - y[time_index])
xp = np.array(xp)
yp = np.array(yp)

x1 = x + distance*np.cos(np.arctan2(yp,xp)+theta)
y1 = y + distance*np.sin(np.arctan2(yp,xp)+theta)


xpp = []
ypp = []
for time in range(len(xp)):
	time_index = time
	if time_index == len(xp) - 1:
		time_index -= 1
	xpp.append(xp[(time_index + 1)] - xp[time_index])
	ypp.append(yp[(time_index + 1)] - yp[time_index])
xpp = np.array(xpp)
ypp = np.array(ypp)

#derivative = ((2*yp*xp*xpp)/((yp**2+xp**2)**2))-((2*yp*xp*ypp)/((yp**2+xp**2)**2))
derivative = (-yp/(xp**2+yp**2))*xpp + (xp/(xp**2 + yp**2))*ypp
y1s = yp + derivative*distance*np.cos(np.arctan2(yp,xp)+theta)
x1s = xp - derivative*distance*np.sin(np.arctan2(yp,xp)+theta)

x2 = x1 + np.cos(np.arctan2(y1s,x1s))
y2 = y1 + np.sin(np.arctan2(y1s,x1s)) 

eq1, = ax.plot(x, y)
eq2, = ax.plot(x1, y1)
#eq3, = ax.plot(x1s, y1s)
eq4, = ax.plot(x2, y2)
arrow, = ax.plot([0],[0],marker = 'o')
arrow2, = ax.plot([0],[0],marker = 'o')

def update(i):
	#eq1.set_data(x[0:i],y[0:i])
	eq2.set_data(x1[0:i],y1[0:i])
	eq4.set_data(x2[0:i],y2[0:i])
	arrow.set_data([x[i],x1[i]],[y[i],y1[i]])
	arrow2.set_data([x1[i],x2[i]],[y1[i],y2[i]])
	return eq2,eq4,arrow,arrow2

ani = animation.FuncAnimation(fig, update, frames = len(x)-2, interval=1000/30, repeat_delay = 3000, blit = True)
#ani.save('non-differentiable2.gif', writer='imagemagick', fps=30)
#ani.save('non-differentiable2.mp4', writer='ffmpeg', fps=30)
for element in range(int(len(x)/5)-1):
	print('actor:',np.sqrt((y[element + 1] - y[element])**2 + (x[element + 1] - x[element])**2))
	print('robot:',np.sqrt((y1[element + 1] - y1[element])**2 + (x1[element + 1] - x1[element])**2))

plt.show()