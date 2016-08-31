import cv2
import time
import math
import numpy as np
import LARC1 as rb
import random
import serial



filename = 'image11.jpg'
binValue = 100  # parameter for the threshold
cap = cv2.VideoCapture(0)
ser = serial.Serial('/dev/ttyACM0',9600, timeout = 3)
# CLUSTERING CLASS


class Cluster:
	"""Clustering mothfkc points"""
	# atributes
	center = []
	new_points = []
	old_points = []
	# methods
	def __init__(self, init_center):
		self.center = init_center
		self.new_points = []
		self.old_points = []
	def calculate_center(self):
		x_sum = 0
		y_sum = 0
		for i in range (len(self.new_points)):
			x_sum = x_sum + self.new_points[i][0]
			y_sum = y_sum + self.new_points[i][1]
		if(len(self.new_points) > 0):
			x_center = int(x_sum/len(self.new_points))
			y_center = int(y_sum/len(self.new_points))
			self.center = (x_center,y_center)
	def add_point(self,new_p):
		self.new_points.append(new_p)
		""" 
		making this not visible

		"""
	def set_center(self,point):
		self. center = point
		""" 
		making this not visible

		"""
	#def set_points(self):
	def distance(self,p):
		x = p[0]
		y = p[1]
		xDif = abs(x - self.center[0])
		yDif = abs(y - self.center[1])
		return math.sqrt((xDif*xDif) + (yDif*yDif))
		""" 
		making this not visible

		"""
	def update_points(self):
		self.old_points = list(self.new_points)
		self.new_points = []
	def get_center(self):
		return self.center
	def get_new_points(self):
		return list(self.new_points)
	def get_old_points(self):
		return list(self.old_points)


# This function is to implement the clustering algorithm 
# PARAMETERS: number of clusters to search for, list of coordinates of the cow
# and number of iterations before returning the final clusters
def findClusters(num_clusters, cowRectangles,iterations):
   # First, the list clusters is initialized with n number of Clusters
	clusters = []
	for iA in range(num_clusters):	
		cluster = Cluster([0,0])
		clusters.append(cluster)
	if num_clusters == 3:
		clusters[0].set_center([320,150])
		clusters[1].set_center([0,480])
		clusters[2].set_center([640,480])
	elif num_clusters == 4:
		clusters[0].set_center([640,0])
		clusters[1].set_center([0,480])
		clusters[2].set_center([640,480])
		clusters[3].set_center([0,0])
	for cluster in clusters:
		print cluster.get_center()

	for dummy_iterator in range(iterations):
		for rect in cowRectangles:
			dist = 100000000
			index = 0
			for i in range(len(clusters)):
				if(clusters[i].distance(rect) < dist):
					dist = clusters[i].distance(rect)
					index = i

			clusters[index].add_point(rect)
		for cluster in clusters:
			cluster.calculate_center()
			cluster.update_points()

	return clusters

# loads image as imgOriginal
# This will have to change to taking snap everytime needed
# in order to do this we must initialice cv2.videoCapture in the future
# and remove parameter filename
def loadImage(capName):
	
	imgOriginal = cv2.imread(capName)
	imgOriginal = cv2.resize(imgOriginal, (720 ,480))
	return imgOriginal

imgOriginal = loadImage(filename)

def neighboors(cowSquares):
	radius = 150
	neigh = []
	for i in range (len(cowSquares)):
		xi = cowSquares[i][4]
		yi = cowSquares[i][5]
		count = 0
		for j in range (len(cowSquares)):
			if (i != j):
				xj = cowSquares[j][4]
				yj = cowSquares[j][5]
				if ((abs(xi-xj) < radius) and (abs(yi-yj) < radius)):
					count = count + 1
		if (count > 2):
			cv2.circle(imgOriginal,(xi,yi),10,(0,255,0),2)
			neigh.append([xi,yi])

	return neigh


def loop():

	while (1):
		incom = ''
		while incom != 'R':
			ser.flush()
			incom = ser.read(1)

		if incom == 'R':
			_, imgOriginal = cap.read()
			filteredImage = rb.clearImage(imgOriginal)
			thresImage = rb.doThresHold(filteredImage,binValue)
			cv2.imshow('T',thresImage)
			contours = rb.findContours(thresImage)
			cv2.drawContours(imgOriginal,contours,-1,(0,0,255),1)
			cowRectangles = rb.getGoodSquares(contours,imgOriginal)
			# We have this order 
			# [area,extent,w,h,x,y]
			n = neighboors(cowRectangles)
			# getting body here !!!! 
			clusters = findClusters(3,n,100)

			# for i in range(len(clusters)):
			# 	b = int ( random.uniform(0,255))
			# 	g = int ( random.uniform(0,255))
			# 	r = int ( random.uniform(0,255))
			# 	list_1 = clusters[i].get_old_points()
			# 	xc,yc = clusters[i].get_center()
			# 	cv2.circle(imgOriginal,(xc,yc),15,(b,g,r),3)
			# 	for j in range (len(list_1)):
			# 		x = list_1[j][0]
			# 		y = list_1[j][1]
			# 		cv2.circle(imgOriginal,(x,y),5,(b,g,r),-1)


			for cluster in clusters:
				print "---------"
				print cluster.get_old_points()
			theta,m,b = rb.ajusteDeCurvas(clusters[0].get_old_points())
			#theta,m,b = rb.ajusteDeCurvas(n,len(n))
			cv2.line(imgOriginal,(100,int(100*m+b)),(600,int(600*m+b)),(255,0,0),3)
			print "angulo ",theta
			print "ordenada al origen", b
			cv2.imshow('i',imgOriginal)
			c = 0
			while c != 27:
				c = cv2.waitKey(50)
				break
			
			cv2.destroyAllWindows()

			theta = abs(int(theta))
			theta = str(theta)
			ser.write(theta)
			echo = ser.read(len(theta))
			print "Este es el echo: ", echo

			if echo == theta:
				print 'jalando'
			else:
				print 'no jalo'

			

	
	






loop()

