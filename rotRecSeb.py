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
binValue = 100  # parameter for the threshold
headingWall = "N"	# GLOBAL DIRECTION VARIABLE
mainFrame = []	# Initialize global variable for image
##--------------------------------------##

##-----------SETUP-----------##
cap = cv2.VideoCapture(0)
# Check that the connection with the camera is open
if not cap.isOpened():
	cap.release()
	raise IOError("Cannot open webcam")
		
arduino = serial.Serial('/dev/ttyACM0',9600, timeout = 1)
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
		#print "Turn Right"
		if checkCorner == False:
			turnBot("right")
			cowFound = isThereACow() # if COW IS FOUND, BRAKE THIS STATE AND GO TO NEXT
			if cowFound:
				break
			turnBot("-45") 
			cowFound = isThereACow()
			if cowFound:
				break
			turnBot("-45") 
			cowFound = isThereACow()
			if cowFound:
				break
		else:
			turnBot("45") 
			cowFound = isThereACow()
			if cowFound:
				break
			turnBot("-45") 
			cowFound = isThereACow()
			if cowFound:
				break
			checkCorner = False
		

		res = moveBot("forward")
		if res == "0":
			pass # we havent arrived to the wall
		elif res == "-1":
			pass # something went wrong 
		elif res == "1":
			checkCorner = True
			turnBot("right")
			headingWall = updateDirection(headingWall) # update global 
	
def findTank():
	good = takePicture()
	if good:
		hsv = cv2.cvtColor(mainFrame,cv2.COLOR_BGR2HSV)

		# Define 'red' in HSV colorspace
		lower_range = np.array([169,100,100], dtype=np.uint8)
		upper_range = np.array([189,255,255], dtype=np.uint8)

		# Threshold the HSV image to get only red color
		mask = cv2.inRange(hsv, lower_range, upper_range)

		# Bitwise-AND mask and original image
		res = cv2.bitwise_and(mainFrame,mainFrame,mask=mask)
		res = cv2.medianBlur(res, 5)

		cv2.imshow('m',mainFrame)
		cv2.imshow('Color Detector', res)
		cv2.waitKey(0)
		cv2.destroyAllWindows()
	
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

def distanceToCow(x):
	y = 29.69*math.exp(0.0044*x)
	if y > 55 or y < 45:
		moveBot(str(int(y - 50)))
	return y

def testImage():
	global mainFrame
	maxLenT = [] # maximumLenghtTissue
	# mainFrame = cv2.imread('image1.jpg')
	gF = takePicture()
	if gF:
		filteredFrame = rb.clearImage(mainFrame)	# Clear the image with a GaussianBlur
		for binValue in range(30,101,10):
			thresFrame = rb.doThresHold(filteredFrame, binValue) # Thresholds the image and erodes it
			# cv2.imshow('t',thresFrame)
			# cv2.waitKey(0)
			# cv2.destroyAllWindows()
			
			contours = rb.findContours(thresFrame) # Finds all the contours inside the image
			cowRectangles = rb.getGoodSquares(contours,mainFrame) # From contours, extract possile cow squares

			newCowRectangles = sorted(cowRectangles, key=lambda x:x.getY(), reverse=True)
			if len(newCowRectangles) > 1:
				greatestTissue = rb.makeTissue(newCowRectangles,[],20,0,[0,0],0)
				if len(greatestTissue) > len(maxLenT):
					maxLenT = greatestTissue
		if len(maxLenT) != 0:
			listMaxLevel = findMaxLevel(maxLenT)
			drawGreatestTissue(maxLenT)
			theta,A,B = rb.ajusteDeCurvas(listMaxLevel)
			# print "tehta",theta
			drawSlope(A,B)
			listMaxLevel = sorted(listMaxLevel, key=lambda x:x.getX(), reverse = False)
			limitRight = listMaxLevel[len(listMaxLevel)-1].getTopRightC()[0]
			limitLeft = listMaxLevel[0].getX()
			maxLenT = sorted(maxLenT, key=lambda x:x.getY(), reverse = False)
			minY = maxLenT[0].getY()
			dToCow = distanceToCow(minY)
			print dToCow
			if theta > 0:
				turnBot(str(90 - theta * 2))
				moveBot("50")
				turnBot(str(-90))
			else:
				turnBot(str(-(90 + theta * 2)))
				moveBot("50")
				turnBot(str(90))
			drawLimits(limitLeft,limitRight,minY)
			print "Theta: ", theta
			cv2.imshow('mF',mainFrame)
			cv2.waitKey(0)
			cv2.destroyAllWindows()

def isThereACow():	
	global mainFrame
	maxLenT = []
	# mainFrame = cv2.imread('image1.jpg')
	gF = takePicture()
	if gF:
		filteredFrame = rb.clearImage(mainFrame)	# Clear the image with a GaussianBlur
		for binValue in range(30,151,5):
			thresFrame = rb.doThresHold(filteredFrame, binValue) # Thresholds the image and erodes it
			# cv2.imshow('t',thresFrame)
			# cv2.waitKey(0)
			# cv2.destroyAllWindows()
			
			contours = rb.findContours(thresFrame) # Finds all the contours inside the image
			cowRectangles = rb.getGoodSquares(contours,mainFrame) # From contours, extract possile cow squares

			newCowRectangles = sorted(cowRectangles, key=lambda x:x.getY(), reverse=True)
			if len(newCowRectangles) > 1:
				greatestTissue = rb.makeTissue(newCowRectangles,[],20,0,[0,0],0)
				if len(greatestTissue) > len(maxLenT):
					maxLenT = greatestTissue
		if len(maxLenT) > 1:
			listMaxLevel = findMaxLevel(maxLenT)
			# drawGreatestTissue(maxLenT)
			theta,A,B = rb.ajusteDeCurvas(listMaxLevel)
			# drawSlope(A,B)
			# huntingCow(theta,listMaxLevel)
			print "Theta: ", theta
			cv2.imshow('mF',mainFrame)
			cv2.waitKey(0)
			cv2.destroyAllWindows()
		return False

# def huntingCow(theta, maxLevel):
	# maxLevel = sorted(maxLevel, key=lambda x:x.getX(), reverse=False)
	# xLeft = maxLevel[0]
	# xRight = 640 - maxLevel[len(maxLevel)-1]
	# if len(maxLevel) > 3:
	# 	if abs(xLeft - xRight) < 40:
	# 		if theta > 0 and theta < 10:

def main():
	while (1):
		if(checkForArduino()):
			if(confirmTerrineZone()):	
				# findTerrine()
				# grabTerrine()
				analizeEnviroment()	# Search for the motherfucker cow
				# positionInFrontCowLateral()
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
	areaT = 0
	for c in greatestTissue:
		areaT += c.getArea()
		b = int ( random.uniform(50,255))
		g = int ( random.uniform(50,255))
		r = int ( random.uniform(50,255))
		x = c.getX()
		y = c.getY()
		cv2.circle(mainFrame,(x,y),5,(b,g,r),-1)
		# cv2.putText(mainFrame,(str(c.getArea())),(x,y+10), font, 0.5,(0,0,255),1,cv2.LINE_AA)
	# areaT /= len(greatestTissue)
	# cv2.putText(mainFrame,(str(areaT)),(50,50), font, 1,(0,0,255),1,cv2.LINE_AA)

def drawSlope(A,B):
	x1 = 0
	x2 = 600
	y1 = int(A*x1 + B)
	y2 = int(A*x2 + B)
	cv2.line(mainFrame,(x1,y1),(x2,y2),(0,0,255),3)

def drawLimits(left,right,y):
	font = cv2.FONT_HERSHEY_SIMPLEX
	cv2.line(mainFrame,(left,0),(left,480),(255,0,0),3)
	cv2.line(mainFrame,(right,0),(right,480),(255,0,0),3)
	cv2.line(mainFrame,(0,y),(640,y),(255,0,0),3)
	cv2.putText(mainFrame,("diff L: " + str(left)),(30,20), font, 0.8,(0,0,255),1,cv2.LINE_AA)
	cv2.putText(mainFrame,("diff R: " + str(640-right)),(30,50), font, 0.8,(0,0,255),1,cv2.LINE_AA)
	cv2.putText(mainFrame,("diff Top: " + str(y)),(30,80), font, 0.8,(0,0,255),1,cv2.LINE_AA)

def readFromFile(fileName):
	num = open('begin.txt','r').read()
	print num
	num = str(1+int(num))
	file = open('begin.txt','w')
	file.write(num)
	file.close()
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
		arduino.flushInput()	
		arduino.flushOutput()
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
	print res

def turnBot(degrees):
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
	print ans
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
checkForArduino()
print "Arduino OK..."
while 1:
	testImage()




##--------------------------##

##-----------Final Instructions-----------##
cap.release()
##----------------------------------------##