# ------ LIBRARIES ----------
import cv2
import time
import math
import numpy as np
import random
import LARC1 as rb
# ---------------------------

filename = 'image5.jpg'
binValue = 110	 # parameter for the threshold
cap = cv2.VideoCapture(0)

# loads image as imgOriginal
# This will have to change to taking snap everytime needed
# in order to do this we must initialice cv2.videoCapture in the future
# and remove parameter filename
def loadImage(capName):
	
	imgOriginal = cv2.imread(capName)
	return imgOriginal

imgOriginal = loadImage(filename)
''' 	IN THIS AREA WE TEST POSIBLE FUNCTIONS OR CLASSES TO BE INSERTED 
		IN LARC1 PERMANENTLY
'''


def neighboors(cowSquares, minNeig):
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
		if (count > minNeig):
			cv2.circle(imgOriginal,(xi,yi),10,(0,255,0),2)
			area = cowSquares[i][0]
			neigh.append([xi,yi,area])

	return neigh

# this function should return at most 3 vaules fo the cow:
# - Angle of rotation
# - Aprox Distance
# - Center missmatch 
def basicProcess(imgOriginal):
	filteredImage = rb.clearImage(imgOriginal)	
	thresImage = rb.doThresHold(filteredImage,binValue)
	cv2.imshow('T',thresImage)
	contours = rb.findContours(thresImage)
	cv2.drawContours(imgOriginal,contours,-1,(0,0,255),1)
	cowRectangles = rb.getGoodSquares(contours,imgOriginal)
	# We have this order 
	# [area,extent,w,h,x,y]
	# Good processing here, still have to make sure no cows are overlaping

	n = neighboors(cowRectangles,2)
	for nt in n:
                print nt
	coordClusters = []
	coordClusters.append([600,180])
	coordClusters.append([425,180])
	coordClusters.append([319,180])
	coordClusters.append([213,180])
        coordClusters.append([40,180])	
	
	clusters = rb.findClusters(n,1,coordClusters)
	for i in range(len(clusters)):
		b = int ( random.uniform(50,255))
		g = int ( random.uniform(50,255))
		r = int ( random.uniform(50,255))
		list_1 = clusters[i].get_old_points()
		x,y = clusters[i].get_center()
		cv2.circle(imgOriginal,(x,y),10,(b,g,r),4)
		for j in range (len(list_1)):
			x = list_1[j][0]
			y = list_1[j][1]
			cv2.circle(imgOriginal,(x,y),5,(b,g,r),-1)
        return 0,0
        '''
	# Interpretation of cow properties below #
	theta,m,b = rb.ajusteDeCurvas(clusters[1].get_old_points()) 
	print theta,m,b
	if len(clusters[1].get_old_points()) < 3:
		return "not enough data"
	else:
		font = cv2.FONT_HERSHEY_SIMPLEX
		cv2.putText(imgOriginal,("angle: " + str(theta)),(300,50), font, 0.7,(0,255,0),1,cv2.LINE_AA)
     
	cv2.line(imgOriginal,(100,int(100*m+b)),(600,int(600*m+b)),(255,0,0),3)
	# print "Angulo :", theta
	# Center of image here
	# x = 720 / 2 
	# y = 480 / 2
	# cv2. circle(imgOriginal, (x,y),5,(0,0,255),-1)
	cv2.imshow(filename,imgOriginal)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

	return theta,clusters
	'''
# This funtion will recieve as a paramates a list with 3 objects from the class 'Cluster'.
# The density of points inside each cluster will be counted and depending on it the next
# movement will be decided. The best case will be to have the gratest number of clusters in the center.
# If not, the rotation will be calculated depending on the cluster's center that has the greatest number 
# of clusters. If two clusters have the same number of squares, the one that has the greatest mean 
# area is the one that will pass.
#def defineNextMovement(clusters):

''' FINISHES HERE '''
################################################
############### MAIN LOOP ######################
################################################
def loop():
	letter = 'd'
	while(letter != 'f'):

		# Takes a picture
		for i in range(4):
			cap.grab()
		goodFrm, imgOriginal = cap.read()

		# If the image is in good form, analyze it
		if goodFrm:
			angle,clusters = basicProcess(imgOriginal)
			#print "Angulo de rotacion : ", angle
			cv2.imshow('img',imgOriginal)
			cv2.waitKey(0)
			cv2.destroyAllWindows()
			
		letter = raw_input('Letter: ')



def test():
	img = loadImage(filename)
	img = rb.clearImage(img)
	print img.shape
        cv2.imshow('img',img)
        cv2.waitKey(0)
	
loop()

cv2.destroyAllWindows()

