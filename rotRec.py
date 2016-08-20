import cv2
import time
import math
import numpy as np
import LARC1 as rb
import random


filename = 'image32.jpg'
binValue = 75 # parameter for the threshold


# loads image as imgOriginal
# This will have to change to taking snap everytime needed
# in order to do this we must initialice cv2.videoCapture in the future
# and remove parameter filename
def loadImage(capName):
	
	imgOriginal = cv2.imread(capName)
	imgOriginal = cv2.resize(imgOriginal, (720 ,480))
	return imgOriginal

imgOriginal = loadImage(filename)
''' 	IN THIS AREA WE TEST POSIBLE FUNCTIONS OR CLASSES TO BE INSERTED 
		IN LARC1 PERMANENTLY
'''

def existsInVertical(lined,key):
	for i in range (len(lined)):
		for j in range(len(lined[i])):
			if lined[i][j] == key:
				return True
	return False
# cowSquares is an emibas list, epsilon is tolerance 
# this function compares squares in X axis to find vertical aligned
# squared and stores them sublists that belong to linedSquares
# NOTE:  linedSquares has 3 dimensions 
def getVerticalSqrs(cowSquares, xTol, minSqrs):
	linedSquares = []
	tempSquares = []
	for i in range(len(cowSquares)):
		xi = cowSquares[i][4]
		yi = cowSquares[i][5]
		areai = cowSquares[i][0]
		temCount = 0
		foundOthers = False
		if not(existsInVertical(linedSquares,(xi,yi,areai))):
			for j in range(i + 1,len(cowSquares)):
				xj = cowSquares[j][4]
				yj = cowSquares[j][5]
				areaj = cowSquares[j][0]
				if abs(xj - xi) < xTol:
					tempSquares.append((xj,yj,areaj))
					temCount = temCount + 1
					foundOthers = True
 		if foundOthers:
 			temCount = temCount + 1
		if temCount > minSqrs:
			tempSquares.append((xi,yi,areai))
			stList = []
			for x in range(len(tempSquares)):
				stList.append(tempSquares[x][2])
			stDeviation = np.std(stList)
			print "Standar Deviation ", stDeviation
			if(stDeviation < 500):
				linedSquares.append(tempSquares)
		tempSquares = []

	linedSquares = sorted(linedSquares, key=lambda x: x[0],reverse=False)

	for x in range (len(linedSquares)):
		linedSquares[x] =  sorted(linedSquares[x], key=lambda x: x[1],reverse=False)

	return linedSquares

def getHorizontalSqrs(cowRectangles, myLegs):
	# trying to find a pair square for the top lef leg
	epsilon = 20
	cowRows = []
	temCowRows = []
	for i in range (len(cowRectangles)):
		x0 = cowRectangles[i][4]
		y0 = cowRectangles[i][5]
		temCowRows = []
		temCowRows.append((x0,y0))
		if (betweenLegs(myLegs,x0) and not (existsInVertical(cowRows,(x0,y0)))):
			for j in range (i + 1, len(cowRectangles)):
				xt = cowRectangles[j][4]
				yt = cowRectangles[j][5]
				if ( betweenLegs(myLegs,xt) and abs(yt - y0) <= epsilon):
					temCowRows.append((xt,yt))
					x0 = xt
					y0 = yt
		b = int ( random.uniform(50,255))
		g = int ( random.uniform(50,255))
		r = int ( random.uniform(50,255))
		if( len(temCowRows) >= 4):
			cowRows.append(temCowRows)
			for rT in temCowRows:
				x = rT[0]
				y = rT[1]
				cv2.circle(imgOriginal,(x,y),4,(b,g,r),-1)

# this function compares a leg
# and returns T if exists else F
def existInAllLegs(allLegs,key):
	for i in range (len(allLegs)):
			if allLegs[i] == key:
				return True
	return False

def getMyLegs(cowSquares, xTol, minSqrs):
	allLegs = []
	tempSquares = []
	xTol = 2
	for i in range(len(cowSquares)):
		# x,y coord of individual sqr
		xi = cowSquares[i][4]
		yi = cowSquares[i][5]
		areai = cowSquares[i][0]
		found = False
		# comparing with rest of Sqrs
		for j in range(len(cowSquares)):
			if(i != j):
				xj = cowSquares[j][4]
				yj = cowSquares[j][5]
				areaj = cowSquares[j][0]
				if abs(xj - xi) < xTol:
					tempSquares.append([xj,yj,areaj])
					found = True		
	 			# adding first sqr
	 			if found:
	 				tempSquares.append([xi,yi,areai])

		print tempSquares
		# sorting tempSquares to compare in allSquares
		tempSquares = sorted(tempSquares, key=lambda x: x[0],reverse=False)
		# minimum sqrs and repetition comparation	
		print len(tempSquares)
		if len(tempSquares) >= minSqrs and not existInAllLegs(allLegs,tempSquares):
			# size RECTIFICATOR comparation
			stList = []
			for x in range(len(tempSquares)):
				stList.append(tempSquares[x][2])
			stDeviation = np.std(stList)
			# ADJUST THIS IF EXPERIMENTALLY TO WORK EFFICIENT
			if(stDeviation < 500):
				allLegs.append(tempSquares)
			
		# clean temporal vairables
		tempSquares = []

	for x in range (len(allLegs)):
		allLegs[x] =  sorted(allLegs[x], key=lambda x: x[1],reverse=False)

	return allLegs

def printLegs(linedSqrs):

	for i in range (0, len(linedSqrs)):
		print linedSqrs[i]
		b = int ( random.uniform(50,255))
		g = int ( random.uniform(50,255))
		r = int ( random.uniform(50,255))
		for j in range(0,len(linedSqrs[i])):
			x = int(linedSqrs[i][j][0])
			y = int(linedSqrs[i][j][1])
			cv2.circle(imgOriginal,(x,y),10,(b,g,r),-1)

# returns True is X coord is between legs of cow
def betweenLegs(myLegs,xt):
	if ( xt >= myLegs[0][0][0] and xt <= myLegs[len(myLegs) - 1][0][0] ):
		return True
	else:
		return False

def legsCenter(myLegs):
	# X and Y of massCenter
	xCenter = 0
	yCenter = 0
	xTemp = 0
	yTemp = 0
	count = 0

	# Getting averages
	for i in range (len(myLegs)):
		for j in range(len(myLegs[i])):
			xTemp = xTemp + myLegs[i][j][0]
			yTemp = yTemp +  myLegs[i][j][1]

		xTemp = xTemp / len(myLegs[i])
		yTemp = yTemp / len(myLegs[i])
		xCenter = xCenter + xTemp
		yCenter = yCenter + yTemp
		xTemp = 0
		yTemp = 0

	xCenter = xCenter / len(myLegs)
	yCenter = yCenter / len(myLegs)

	return (xCenter,yCenter)

''' FINISHES HERE '''
################################################
############### MAIN LOOP ######################
################################################
def loop():

	
	filteredImage = rb.clearImage(imgOriginal)
	thresImage = rb.doThresHold(filteredImage,binValue)
	cv2.imshow('T',thresImage)
	contours = rb.findContours(thresImage)
	cv2.drawContours(imgOriginal,contours,-1,(0,0,255),1)

	cowRectangles = rb.getGoodSquares(contours,imgOriginal)
	# We have this order 
	# [area,extent,w,h,x,y]
	cowRectangles = rb.sortList(5,cowRectangles)
	

	# getHorizontalSqrs(cowRectangles, myLegs)

	myLegs = getMyLegs(cowRectangles,15 ,10) # epsilon and >minSqrs
	cowRectangles = rb.sortList(4,cowRectangles)
	printLegs(myLegs)
	print "numero de patas :", len(myLegs)


	# x, y  = legsCenter(myLegs)
	# cv2. circle(imgOriginal, (x,y),5,(0,255,0),-1)
	
	x = 720 / 2 
	y = 480 / 2
	cv2. circle(imgOriginal, (x,y),5,(0,0,255),-1)




	cv2.imshow(filename,imgOriginal)
	cv2.waitKey(0)

loop()
cv2.destroyAllWindows()

