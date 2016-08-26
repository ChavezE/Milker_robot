import cv2
import numpy as np
import math

# bodyCoords should be a bidimentional list that in each of its elements contains 
# a coordinate of a square inside the body square
# Equations are got from this webpage: 
# http://portales.puj.edu.co/objetosdeaprendizaje/Online/OA10/capitulo3/3.6.htm
def ajusteDeCurvas(bodyCoords):
	# These lists and variables are used to calculate A and B
	# to get a polynomial of 1st grade: y = Ax + B
	Xi = []
	Yi = []
	XiYi = []
	Xi2 = []
	n = len(bodyCoords)	
	# This cycle is to fill up all the lists in order to compute A and B
	for coord in bodyCoords:
		xT = coord[0]
		yT = coord[1]
		Xi.append(xT)
		Yi.append(yT)
		XiYi.append(xT*yT)
		Xi2.append(pow(xT,2))
	# The reason to create lists is to get the sum of all its elements 
	# so it will be computed the sum of each of the lists
	sXi = sum(Xi)
	sYi = sum(Yi)
	sXiYi = sum(XiYi)
	sXi2 = sum(Xi2)
	# Now its time to compute A and B
	A = ((n*sXiYi)-(sXi*sYi))/((n*sXi2)-(pow(sXi,2)))
	B = ((sXi2*sYi)-(sXiYi*sXi))/((n*sXi2)-(pow(sXi,2)))
	theta = math.tan(A)
	theta = 180/math.pi
	return theta,A,B


pruebita = [[1,1],[2,2],[3,3]]
theta,m,b = ajusteDeCurvas(pruebita)
print theta