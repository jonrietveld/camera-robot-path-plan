		def LinePlaneCollision(planeNormal, planePoint, rayDirection, rayPoint, epsilon=1e-6):
		ndotu = planeNormal.dot(rayDirection)
		if abs(ndotu) < epsilon:
			raise RuntimeError("no intersection or line is within plane")
 	
		w = rayPoint - planePoint
		si = -planeNormal.dot(w) / ndotu
		Psi = w + si * rayDirection + planePoint
		return Psi


		planeNormal = np.array([0,0,1])
		planePoint = np.array([0, 0, time]) #Any point on the plane of given time		
		rayDirection = np.array([pathArr[closeTime+1][0]-pathArr[closeTime][0],pathArr[closeTime+1][1]-pathArr[closeTime][1],pathArr[closeTime+1][2]-pathArr[closeTime][2]]) #Define ray
		rayPoint = np.array(pathArr[closeTime]) #Any point along the ray
		intersect = LinePlaneCollision(planeNormal, planePoint, rayDirection, rayPoint)