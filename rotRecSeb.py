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
##-----------LIBRARIES-----------##
import cv2
import time
import math
import numpy as np
import LARC1 as rb
import random
import serial
import statistics
##-------------------------------##

##-----------GLOBAL VARIABLES-----------##
binValue = 110  # parameter for the threshold
headingWall = "N"	# GLOBAL DIRECTION VARIABLE
mainFrame = []	# Initialize global variable for image
##--------------------------------------##

##-----------SETUP-----------##
# cap = cv2.VideoCapture(0)
# # Check that the connection with the camera is open
# if not cap.isOpened():
# 	cap.release()
# 	raise IOError("Cannot open webcam")
		
# arduino = serial.Serial('/dev/ttyACM0',9600, timeout = 1)
# # When testing, setup the threshold value
# binValue = raw_input('Define threshold value: ')
##---------------------------##


##-----------MAIN FUNCTIONS-----------##
def takePicture():
	global mainFrame
	for i in range(4):
		cap.grab()
	goodFrm, mainFrame = cap.read()

	return goodFrm

def analizeEnviroment():
	global headingWall
	checkCorner = False
	cowFound = False
	while 1:
		#print "Beginning to anaEnv"
		if checkCorner:
			turnBot("45")
			#print "Turn Left 45"
			cowFound = isThereACow()
			if cowFound:
				break

			turnBot("-45")
			#print "Turn Left 45"
			checkCorner = False
		else:
			# face center of arena and look for cow
			#print "Turn Right"
			turnBot("right")
			
			cowFound = isThereACow() # if COW IS FOUND, BRAKE THIS STATE AND GO TO NEXT
			if cowFound:
				#print "Cow found"
				break
			#print "Not found cow"
			turnBot("left") 
			#print "Turn Left"

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
			#print "Turn Right"
			headingWall = updateDirection(headingWall) # update global 

def isThereACow():	
	# Analyze a frame and tell whether there is enough information to analyze or not
	global mainFrame
	goodFrm = takePicture()
	
	# If the frame isn't corrupted, then analyze it.
	if goodFrm:
		filteredFrame = rb.clearImage(mainFrame)	
		thresFrame = rb.doThresHold(filteredFrame, binValue) 
		# cv2.imshow('t',thresFrame)
		contours = rb.findContours(thresFrame) 
		cowRectangles = rb.getGoodSquares(contours,mainFrame) 
		greatesTissue = rb.makeTissue(cowRectangles,[],30,0,[0,0],0)
		print greatesTissue
		'''
			# theta,A,B = rb.ajusteDeCurvas(clusters[1].get_old_points())
			
			# Now its time to analyze the clusters
			if clustersNotEmpty(clusters):	# The 3 clusters must have more than 1 element
				print "Cow is close to the center of the camera"
				return True
			elif len(clusters[0].get_old_points()) > 1 and len(clusters[1].get_old_points()) > 1:
				return False
				print "Rob must turn left in order to enter below the cow"
			elif len(clusters[2].get_old_points()) > 1 and len(clusters[1].get_old_points()) > 1:
				return False
				print "Rob must turn right in order to enter below the cow"
			else:
				print "No cow found"				
				return False	
		else:
			print "No cow found"
			return False
		'''

def findTank():
	good = takePicture()
	if good:
		hsv = cv2.cvtColor(mainFrame,cv2.COLOR_BGR2HSV)

		# Define 'red' in HSV colorspace
		lower = np.array([0,100,50])
		upper = np.array([0,100,255])

		# Threshold the HSV image to get only red color
		mask = cv2.inRange(hsv, lower, upper)

		# Bitwise-AND mask and original image
		res = cv2.bitwise_and(mainFrame,mainFrame,mask=mask)
		res = cv2.medianBlur(res, 5)

		cv2.imshow('m',mainFrame)
		cv2.imshow('Color Detector', res)

		cv2.waitKey(0)
		cv2.destroyAllWindows()

def mode(list):
	actMax = []
	
def findMaxLevel(tissue):
	maxLevel = []
	levels = []
	for t in tissue:
		levels.append(t.getLevel())
	modeLevels = statistics.mode(levels)
	for i in range(len(tissue)):
		t = tissue[i]
		if t.getLevel() == modeLevels:
			while t.getLevel() == modeLevels:
				maxLevel.append(t)
				i += 1
				if i < len(tissue):
					t = tissue[i]
				else:
					return maxLevel
			return maxLevel

def testImage():
	global mainFrame
	mainFrame = cv2.imread('image4.jpg')
	filteredFrame = rb.clearImage(mainFrame)	# Clear the image with a GaussianBlur
	thresFrame = rb.doThresHold(filteredFrame, binValue) # Thresholds the image and erodes it
	cv2.imshow('t',thresFrame)
	# cv2.waitKey(0)
	# cv2.destroyAllWindows()
	
	contours = rb.findContours(thresFrame) # Finds all the contours inside the image
	cowRectangles = rb.getGoodSquares(contours,mainFrame) # From contours, extract possile cow squares

	newCowRectangles = sorted(cowRectangles, key=lambda x:x.getY(), reverse=True)
	
	greatestTissue = rb.makeTissue(newCowRectangles,[],20,0,[0,0],0)
	listMaxLevel = findMaxLevel(greatestTissue)
	
	drawGreatestTissue(greatestTissue)
	theta,A,B = rb.ajusteDeCurvas(listMaxLevel)
	drawSlope(A,B)
	print "Theta: ", theta
	cv2.imshow('m',mainFrame)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def main():
	while (1):
		#print "Checking for Arduino"
		if(checkForArduino()):	# Send something to Arduino and recieve it 
			#print "Arduino is alive"
			if(confirmTerrineZone()):	# Check out that the robot is ready to search for the terrine
				#print "We are in Terrine Zone"
				##				# findTerrine()
				##				# grabTerrine()
				#print "Next, analyze environment"
				analizeEnviroment()	# Search for the motherfucker cow
				##				# positionInFrontCowLateral()
				##				#print "Cow Found"
				if(milk()):
					goAlamus()
				time.sleep(5)
##------------------------------------##

##-----------SECONDARY FUNCTIONS-----------##
def drawClusters(clusters):
	global mainFrame
	for i in range(len(clusters)):
		b = int ( random.uniform(50,255))
		g = int ( random.uniform(50,255))
		r = int ( random.uniform(50,255))
		list_1 = clusters[i].get_old_points()
		x,y = clusters[i].get_center()
		cv2.circle(mainFrame,(x,y),10,(b,g,r),4)
		for j in range (len(list_1)):
			x = list_1[j][0]
			y = list_1[j][1]
			cv2.circle(mainFrame,(x,y),5,(b,g,r),-1)

def clustersNotEmpty(clusters):
	for cluster in clusters:
		if len(cluster.get_old_points()) <= 1:
			return False
	return True	

def updateDirection(lasDir):
	if lasDir == "N":
		return "E"
	elif lasDir == "S":
		return "W"
	elif lasDir == "E":
		return "S"
	else:
		return "N"

def drawGreatestTissue(greatestTissue):
	global mainFrame
	font = cv2.FONT_HERSHEY_SIMPLEX
	for c in greatestTissue:
		b = int ( random.uniform(50,255))
		g = int ( random.uniform(50,255))
		r = int ( random.uniform(50,255))
		x = c.getX()
		y = c.getY()
		cv2.circle(mainFrame,(x,y),10,(b,g,r),4)
		cv2.putText(mainFrame,(str(c.getLevel())),(x,y+10), font, 0.5,(0,0,255),1,cv2.LINE_AA)

def drawSlope(A,B):
	x1 = 0
	x2 = 600
	y1 = int(A*x1 + B)
	y2 = int(A*x2 + B)
	cv2.line(mainFrame,(x1,y1),(x2,y2),(0,0,255),3)
##-----------------------------------------##

##-----------Arduino Interaction-----------##
def checkForArduino():
	# Send a 'b' to tell the Arduino the RaspberryPi is alive
	ans = " "
	while ans != "b":
		arduino.write("b")
		time.sleep(1)
		ans = arduino.read()	

	arduino.flushInput()	
	arduino.flushOutput()
	
	# Arduino is alive if the RaspberryPi recieved a 'b'
	if(ans == "b"): 
		return True
	else:
		raise IOError("Can't connect with arduino")

def confirmTerrineZone():
	# This function is for the 1st case of the Arduino. The position of the terrines 
	# should detect a wall with the left and back distance sensors.
	arduino.flushInput()	
	arduino.flushOutput()
	arduino.write("1")	# Execute the 1st state of the arduino
	res = " "
	while res != "0":
		time.sleep(1.5)		# Wait for 1.5 seconds before reading a answer from Arduino
		res = arduino.read(2)
		# #print "Res: ",res
		if (res == "0"):
			return True
		else:
			arduino.write("1")

def findTerrine():
	arduino.write('2')

def grabTerrine():
	arduino.write('3')

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
	res = arduino.read(2)
	if res == 0:
		return True

def turnBot(degrees):
	arduino.flushInput()
	arduino.flushOutput()
	if degrees == "right":
		#print "Turn Right"
		degrees = "90"
	elif degrees == "left":
		#print "Turn Left"
		degrees = "-90"
        
	arduino.write("4")
	arduino.write("1")
	arduino.write(degrees)
        
	while(arduino.inWaiting() <= 0):
		pass

	ans = arduino.read(2)
	return(ans)

def milk():
	arduino.write("7")
	while (arduino.inWaiting <= 0):
		pass
	if(arduino.read(1) == "1"):
		# It entered below the cow
		return True
	else:
		# Something went wrong
		return False

def goAlamus():
	# After we milked cow, it's time to go to the gate
	arduino.write("9")
	# Get to the wall you were before
	if headingWall == "E":
		arduino.write("0")
	elif headingWall == "S":
		arduino.write("90")
	elif headingWall == "W":
		arduino.write("180")
	while(arduino.inWaiting()<=0):
		pass
	res = arduino.read(1)
	if res == "0":
		arduino.write("11")
	else:
	# Something went wrong
		pass
##--------------------------------------##

##-----------LOOP-----------##
testImage()
##--------------------------##