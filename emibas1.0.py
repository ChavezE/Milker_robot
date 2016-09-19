##-----------LIBRARIES-----------##
import cv2
import time
import math
import numpy as np
import LARC1 as rb
import random
import serial
##-------------------------------##

##-----------GLOBAL VARIABLES-----------##
binValue = 85  # parameter for the threshold
headingWall = "N"	# GLOBAL DIRECTION VARIABLE
mainFrame = cv2.imread('image5.jpg')	# Initialize global variable for image
##--------------------------------------##

##-----------SETUP-----------##
cap = cv2.VideoCapture(0)
# Check that the connection with the camera is open
if not cap.isOpened():
        cap.release()
	raise IOError("Cannot open webcam")
arduino = serial.Serial('/dev/ttyACM0',9600, timeout = 3)
# When testing, setup the threshold value
# binValue = raw_input('Define threshold value: ')
##---------------------------##


##-----------FUNCTIONS-----------##
def takePicture():
   for i in range(4):
      cap.grab()
   goodFrm, img = cap.read()
   mainFrame = img 	# Update global image variable
  
   return goodFrm

# This function is for the 1st case of the Arduino. The position of the terrines 
# should detect a wall with the left and back distance sensors.
def confirmTerrineZone():
        arduino.flushInput()
        arduino.flushOutput()
	arduino.write("1")	# Execute the 1st state of the arduino
	while (arduino.inWaiting() <= 0):
                pass

        res = arduino.read(2)
        print "Res: ",res
        if (res == "0"):
                return True
        else:
                arduino.write("1")

	# Now we need to wait for an acknowledgement from the arduino
	# 0 - OK, -1 - Not OK
	# incom = ' '
	# initTime = time.time()
	# counTimeOut = time.time() - initTime
	# while incom != '0' and incom != '-1' and counTimeOut < 15:
		# incom = arduino.read(1)
		# Count the seconds the loop has runned
		# counTimeOut = time.time() - initTime	

	# Check incom and counTimeOur in order to determine success
	# if counTimeOut < 15 and incom == '0':
		# Proceed to next function
	# else:
		# Determine a routine to handle the error

def findTerrine():
	arduino.write('2')

def grabTerrine():
	arduino.write('3')

# Analyze a frame and tell whether there is enough information to analyze or not
def isThereACow():	
	# Take the picture
	goodFrm = takePicture()
        cv2.imshow('m',mainFrame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
	# If the frame isn't corrupted, then analyze it.
	if goodFrm:
		filteredFrame = rb.clearImage(mainFrame)	# Clear the image with a GaussianBlur
		thresFrame = rb.doThresHold(filteredFrame, binValue) # Thresholds the image and erodes it
		# cv2.imshow('t',thresFrame)
		contours = rb.findContours(thresFrame) # Finds all the contours inside the image
		cowRectangles = rb.getGoodSquares(contours,mainFrame) # From contours, extract possile cow squares
		cowNeighboors = rb.neighboors(cowRectangles) # Find squares that have at least to neighboors
		print "Len CowNeighs: ", len(cowNeighboors)


		# If there are enough data, run the clustering algorithm
		if len(cowNeighboors) > 8:
			# Cluster the rectangles in order to obtain the center of the cow 
			coordClusters = []	# List to sotre the centers' coordinates 
			coordClusters.append([160,260])	# Left cluster's center
			coordClusters.append([320,180])	# Center cluter's center
			coordClusters.append([480,260])	# Right cluster's center
			clusters = rb.findClusters(cowNeighboors,5,coordClusters)	# Make 5 iterations to determine the clusters
			# mainFrame = drawClusters(clusters, mainFrame)	# Draw each cluster in a different color

			# Now its time to analyze the clusters
			if clustersNotEmpty(clusters):	# The 3 clusters must have more than 1 element
				print "Cow is close to the center of the camera"
				return True
			# elif len(clusters[0].get_old_points()) > 1 and len(clusters[1].get_old_points()) > 1:
			# 	print "Rob must turn left in order to enter below the cow"
			# elif len(clusters[2].get_old_points()) > 1 and len(clusters[1].get_old_points()) > 1:
			# 	print "Rob must turn right in order to enter below the cow"
			else:
				print "No cow found"				
				return False	
		else:
			print "No cow found"
			return False
		# Show main frame 
		# cv2.imshow('f',mainFrame)
		# cv2.waitKey(0)
		# cv2.destroyAllWindows()

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

def clustersNotEmpty(clusters):
	for cluster in clusters:
		if len(cluster.get_old_points()) <= 1:
			return False
	return True	

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

# advice the arduino microcontroler that we are on line
# sending signal and wait for aknowledgement 
def checkForArduino():
        arduino.flushInput()
        arduino.flushOutput()
	arduino.write("b") # sending the ping
	ans = arduino.read()
	print "Ans: ", ans
	while(ans == ""):
                print "Wait for 1 sec"
                time.sleep(1)
                arduino.write("b")
                ans = arduino.read()
##	waiting = arduino.inWaiting() 
##	print "InW: ",waiting
##	while (waiting < 0):
##                waiting = arduino.inWaiting()
##		print "InW: ",waiting
##	print "InW: ",waiting
##	res = arduino.read()
##	print "Res: ",res
	if(ans == "b"): # arduino is alive and ready
		return True
	else:
		return False

def moveBot(cm):
	if cm == "forward":
		cm = "50"
	elif cm == "backward":
		cm = "-50"

	arduino.write("4")
	arduino.write("0")
	arduino.write(cm)
	while(arduino.inWaiting() <= 0):
		pass
	return(arduino.read(2))

def turnBot(degrees):
        
        arduino.flushInput()
        arduino.flushOutput()
	if degrees == "right":
                print "Turn Right"
		degrees = "90"
	elif degrees == "left":
                print "Turn Left"
		degrees = "-90"
        
	arduino.write("4")
	arduino.write("1")
	arduino.write(degrees)
        
	while(arduino.inWaiting() <= 0):
                pass
        ans = arduino.read(2)
	print "Ans: ",ans
	return(ans)

def updateDirection(lasDir):
	if lasDir == "N":
		return "E"
	elif lasDir == "S":
		return "W"
	elif lasDir == "E":
		return "S"
	else:
		return "N"


# Explore the arena to find the desired misterious liquid (milk)
# The coordenates of the arena will be taken this way:
# This approach starts after picking up the terrine
'''
	NOMENCLATURE:
	1. Empty Terrines zone
	2. Drop Terrines zone

			 EAST
	 	 _____________
   		|			  | 
NORTH 	|			  |	SOUTH
   		|			  |
   		-2----	-----1-
   			 ***

   			 WEST
'''

def analizeEnviroment():
	checkCorner = False
	cowFound = False
	while 1:
                print "Beginning to anaEnv"
		if checkCorner:
			turnBot("45")
			cowFound = isThereACow()
			if cowFound:
				break

			turnBot("-45")
			checkCorner = False
		else:
			# face center of arena and look for cow
			turnBot("right")
			
			cowFound = isThereACow() # if COW IS FOUND, BRAKE THIS STATE AND GO TO NEXT
			if cowFound:
                                print "Cow found"
				break
			print "Not found cow"
			turnBot("left") 

		cowFound = isThereACow() # to see if can keep moving on that direction
		if cowFound:
			break

		res = moveBot("forward")
		if res == "0":
			pass # we havent arrived to the wall
		elif res == "-1":
			pass # something went wrong 
		elif res == "1":
			# most probable you're at the corner
			# check the corner
			checkCorner = True
			turnBot("right")
			headingWall = updateDirection(headingWall) # update global 

# After we milked cow, it's time to go to the gate
def goAlamus():

	arduino.write("9")
	# Get to the wall you were before
	if headingWall == "E":
		arduino.write("0")
	elif headingWall == "S":
		arduino.write("90")
	elif headingWall == "W":
		arduino.write("180")
	while(arduino.inWaiting()<0):
		pass
	res = arduino.read(1)
	if res == "0":
		arduino.write("11")
	else:
	# Something went wrong
		pass

def milk():
	arduino.write("7")
	while (arduino.inWaiting):
		pass
	if(arduino.read(1) == "1"):
		# It entered below the cow
		return True
	else:
		# Something went wrong
		return False
##-------------------------------##

##-----------MAIN FUNCTIONS-----------##
def main():
	
	while (1):
                print "Checking for Arduino"
                
		if(checkForArduino()):	# Send something to Arduino and recieve it 
			print "Arduino is alive"
			if(confirmTerrineZone()):	# Check out that the robot is ready to search for the terrine
				print "We are in Terrine Zone"
##				# findTerrine()
##				# grabTerrine()
				print "Next, analyze environment"
				analizeEnviroment()	# Search for the motherfucker cow
##				# positionInFrontCowLateral()
##				print "Cow Found"
				if(milk()):
					goAlamus()
		time.sleep(5)

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
