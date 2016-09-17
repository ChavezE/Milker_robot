##-----------LIBRARIES-----------##
import cv2
import time
import math
import numpy as np
import LARC1 as rb
import random
import serial
##-------------------------------##


##-----------SETUP-----------##
binValue = 100  # parameter for the threshold
cap = cv2.VideoCapture(0)
# Check that the connection with the camera is open
if not cap.isOpened():
	raise IOError("Cannot open webcam")
# arduino = serial.Serial('/dev/ttyACM0',9600, timeout = 3)

# When testing, setup the threshold value
binValue = raw_input('Define threshold value: ')
##---------------------------##


##-----------FUNCTIONS-----------##
def takePicture():
   for i in range(4):
      cap.grab()
   goodFrm, img = cap.read()
   return goodFrm, img

def main():
	
	while (1):
		# checkArduinoIsAlive()	# Send something to Arduino and recieve it back
		# confirmTerrineZone()	# Check out that the robot is ready to search for the terrine
		# findTerrine()
		# grabTerrine()
		# goodFrm, imgOriginal = rb.takePicture()
		# analyzeEnvironment()	# Search for the motherfucker cow
		# positionInFrontCowLateral()
		isThereACow()


# This function is for the 1st case of the Arduino. The position of the terrines 
# should detect a wall with the left and back distance sensors.
def confirmTerrineZone():
	arduino.write('1')	# Execute the 1st state of the arduino	

	# Now we need to wait for an acknowledgement from the arduino
	# 0 - OK, -1 - Not OK
	incom = ' '
	initTime = time.time()
	counTimeOut = time.time() - initTime
	while incom != '0' and incom != '-1' and counTimeOut < 15:
		incom = arduino.read(1)
		# Count the seconds the loop has runned
		counTimeOut = time.time() - initTime	

	# Check incom and counTimeOur in order to determine success
	# if counTimeOut < 15 and incom == '0':
		# Proceed to next function
	# else:
		# Determine a routine to handle the error

def findTerrine():
	arduino.write('2')

def grabTerrine():
	arduino.write('3')

# This function will make the robot walk over the border of the corral zone. The terrine must be grabbed 
# before this function is called
def analyzeEnvironment():
	orientation = 'W'	# The first orientation of the robot after grabbing the terrine

# Analyze a frame and tell whether there is enough information to analyze or not
def isThereACow():	
	# Take the picture
	goodFrm, mainFrame = takePicture()
	
	# If the frame isn't corrupted, then analyze it
	if goodFrm:
		filteredFrame = rb.clearImage(mainFrame)	# Clear the image with a GaussianBlur
		thresFrame = rb.doThresHold(filteredFrame, binValue) # Thresholds the image and erodes it
		contours = rb.findContours(thresFrame) # Finds all the contours inside the image
		cowRectangles = rb.getGoodSquares(contours,mainFrame) # From contours, extract possile cow squares
		cowRectangles = neighboors(cowRectangles,2) # Find squares that have at least to neighboors

		# Cluster the rectangles in order to obtain the center of the cow 
		coordClusters = []	# List to sotre the centers' coordinates 
		coordClusters.append([160,260])	# Left cluster's center
		coordClusters.append([320,180])	# Center cluter's center
		coordClusters.append([480,260])	# Right cluster's center
		clusters = rb.findClusters(cowRectangles,5,coordClusters)	# Make 5 iterations to determine the clusters
		mainFrame = rb.drawClusters(clusters, mainFrame)	# Draw each cluster in a different color
		cv2.imshow('f',mainFrame)
		cv2.waitKey(0)
		cv2.destroyAllWindows()

##-------------------------------##

##-----------MAIN FUNCTIONS-----------##	
main()
cap.release()










# Code that may be used in the futue...
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

"""
TEST COMMUNICATION WITH THE ARDUNIO
"""
# This code was for testing communication 
# with the Arduino Microcontroller

# incom = ''
# while incom != 'R':
# 	arduino.flush()
# 	incom = arduino.read(1)

# if incom == 'R':
# 	for i in range(4):
# 		cap.grab()
# 	_, imgOriginal = cap.read()
# 	filteredImage = rb.clearImage(imgOriginal)
# 	thresImage = rb.doThresHold(filteredImage,binValue)
# 	cv2.imshow('T',thresImage)
# 	contours = rb.findContours(thresImage)
# 	cv2.drawContours(imgOriginal,contours,-1,(0,0,255),1)
# 	cowRectangles = rb.getGoodSquares(contours,imgOriginal)
# 	# We have this order 
# 	# [area,extent,w,h,x,y]
# 	n = neighboors(cowRectangles)
# 	# getting body here !!!! 
# 	clusters = findClusters(3,n,100)

	


# 	for cluster in clusters:
# 		print "---------"
# 		print cluster.get_old_points()
# 	theta,m,b = rb.ajusteDeCurvas(clusters[0].get_old_points())
# 	#theta,m,b = rb.ajusteDeCurvas(n,len(n))
# 	cv2.line(imgOriginal,(100,int(100*m+b)),(600,int(600*m+b)),(255,0,0),3)
# 	print "angulo ",theta
# 	print "ordenada al origen", b
# 	cv2.imshow('i',imgOriginal)
# 	c = 0
# 	while c != 27:
# 		c = cv2.waitKey(50)
# 		break
	
# 	cv2.destroyAllWindows()

# 	theta = abs(int(theta))
# 	theta = str(theta)
# 	arduino.write(theta)
# 	echo = arduino.read(len(theta))
# 	print "Este es el echo: ", echo

# 	if echo == theta:
# 		print 'jalando'
# 	else:
# 		print 'no jalo'
