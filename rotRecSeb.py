# ------ LIBRARIES ----------
import cv2
import time
import math
import numpy as np
import random
import LARC1 as rb
# ---------------------------

filename = 'image5.jpg'
binValue = 80	 # parameter for the threshold
cap = cv2.VideoCapture(0)

if not cap.isOpened():
	raise IOError("Cannot open webcam")

# When testing, setup the threshold value
binValue = raw_input('Define threshold value: ')
binValue = float(binValue)
# loads image as imgOriginal
# This will have to change to taking snap everytime needed
# in order to do this we must initialice cv2.videoCapture in the future
# and remove parameter filename
def loadImage(capName):
	imgOriginal = cv2.imread(capName)
	return imgOriginal

# imgOriginal = loadImage(filename)
''' 	IN THIS AREA WE TEST POSIBLE FUNCTIONS OR CLASSES TO BE INSERTED 
		IN LARC1 PERMANENTLY
'''

def drawClusters(clusters,img):
	for i in range(len(clusters)):
		b = int ( random.uniform(50,255))
		g = int ( random.uniform(50,255))
		r = int ( random.uniform(50,255))
		list_1 = clusters[i].get_old_points()
		x,y = clusters[i].get_center()
		cv2.circle(img,(x,y),10,(b,g,r),4)
		for j in range (len(list_1)):
			x = list_1[j][0]
			y = list_1[j][1]
			cv2.circle(img,(x,y),5,(b,g,r),-1)
	return img
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

def histEqualisationYUV(img):
	img_yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
	img_yuv[:,:,0] = cv2.equalizeHist(img_yuv[:,:,0])
	img_output = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)
	return img_output

def loop():
	
	letter = 'd'
	while(letter != 'f'):

		# Takes a picture
		for i in range(4):
			cap.grab()
		goodFrm, frame = cap.read()
		cv2.imshow('f',frame)
		# rows, cols = frame.shape[:2]
		# gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		"""
		# If the image is in good form, analyze it
		if goodFrm:
			# angle,clusters = basicProcess(frame)
			frame = histEqualisationYUV(frame)
			cv2.imshow('f',frame)
			frame = rb.clearImage(frame)
			thresImage = rb.doThresHold(frame,60)
			cv2.imshow('T',thresImage)
			#print "Angulo de rotacion : ", angle
		"""
		cv2.waitKey(0)
		cv2.destroyAllWindows()

		# LAST STEP... Ask if the user wants to take another picture
		letter = raw_input('Letter: ')
	
	
# Analyze a frame and tell whether there is enough information to analyze or not
def isThereACow():	
	# Take the picture
	for i in range(4):
		cap.grab()
	goodFrm, mainFrame = cap.read()
	# If the frame isn't corrupted, then analyze it
	if goodFrm:
		filteredFrame = rb.clearImage(mainFrame)	# Clear the image with a GaussianBlur
		thresFrame = rb.doThresHold(filteredFrame, binValue) # Thresholds the image and erodes it
		contours = rb.findContours(thresFrame) # Finds all the contours inside the image
		cowRectangles, mainFrame = rb.getGoodSquares(contours,mainFrame) # From contours, extract possile cow squares
		cowRectangles = rb.neighboors(cowRectangles,2,mainFrame) # Find squares that have at least to neighboors

		# Cluster the rectangles in order to obtain the center of the cow 
		coordClusters = []	# List to sotre the centers' coordinates 
		coordClusters.append([160,260])	# Left cluster's center
		coordClusters.append([320,180])	# Center cluter's center
		coordClusters.append([480,260])	# Right cluster's center
		clusters = rb.findClusters(cowRectangles,5,coordClusters)	# Make 5 iterations to determine the clusters
		mainFrame = drawClusters(clusters, mainFrame)	# Draw each cluster in a different color
		cv2.imshow('f',mainFrame)
		cv2.waitKey(0)
		cv2.destroyAllWindows()

isThereACow()
cap.release()

