import cv2
import time
import math
import numpy as np
import LARC1 as rb
import random
import ajusteDeCurvas as aj


filename = 'image3.jpg'
binValue = 90  # parameter for the threshold


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

def getHorizontalSqrs(cowRectangles, myLegs):
	# trying to find a pair square for the top lef leg
	epsilon = 3
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
				cv2.circle(imgOriginal,(x,y),10,(b,g,r),-1)
	return cowRows

# this function compares a leg
# and returns T if exists else F
def existInAllLegs(allLegs,key):
	for i in range (len(allLegs)):
			if allLegs[i] == key:
				return True
	return False
# Recibes a list of lists with (x,y,area) = atom
# return list with less Standar Deviation in areas
def getMyLegs(cowSquares, xTol, minSqrs):
   allLegs = []
   for i in range(len(cowSquares)):
      # x,y coord of individual sqr
      xi = cowSquares[i][4]
      yi = cowSquares[i][5]
      areai = cowSquares[i][0]
      tempSquares = []  # List to store neighbours from 'i'
      tempSquares.append([xi,yi,areai])   # adding first square
      
      # comparing with rest of Sqrs
      for j in range(len(cowSquares)):
         if(i != j):    # Check that it is not comparing with itself
            xj = cowSquares[j][4]
            yj = cowSquares[j][5]
            areaj = cowSquares[j][0]
            # If both are close in 'x' plane insert to the temporal list
            if abs(xj - xi) < xTol: 
               tempSquares.append([xj,yj,areaj])    

      # sorting tempSquares to compare in allSquares
      tempSquares = sorted(tempSquares, key=lambda x: x[2],reverse=False)
      tempSquares = sorted(tempSquares, key=lambda x: x[0],reverse=False)

      if len(tempSquares) >= minSqrs and not existInAllLegs(allLegs,tempSquares):
         # size RECTIFICATOR comparation
         stList = []
         for x in range(len(tempSquares)):
            stList.append(tempSquares[x][2])
         stDeviation = np.std(stList)
         #print "Std Deviation: ", stDeviation
         # ADJUST THIS IF EXPERIMENTALLY TO WORK EFFICIENT
         if(stDeviation < 500):
            allLegs.append(tempSquares)
   
   tempAllLegs = []
   for leg in allLegs:
      commonSqrLegs = [] 
      commonSqrLegs.append(leg)
      for legCompare in allLegs:
         if allLegs.index(leg) != allLegs.index(legCompare) and haveCommonSquares(leg,legCompare):
            commonSqrLegs.append(legCompare)
      
      lT = minStdDev(commonSqrLegs)
      if not existInAllLegs(tempAllLegs,lT):
         tempAllLegs.append(lT)


   allLegs = tempAllLegs
   for x in range (len(allLegs)):
      allLegs[x] =  sorted(allLegs[x], key=lambda x: x[1],reverse=False)
   
   return allLegs

def haveCommonSquares(list1,list2):
   for squareAct in list1:
      if list2.count(squareAct) != 0:
         return True
   return False
# Recibes a list of lists with (x,y,area) = atom
# return list with less Standar Deviation in areas
def minStdDev(allLegs):
   minDev = -1
   stdDev = 100000
   for i in range(len(allLegs)):
      stList = []
      for x in range(len(allLegs[i])):
         stList.append(allLegs[i][x][2])
      tempDev = np.std(stList)
      if tempDev < stdDev:
         stdDev = tempDev
         minDev = i
   
   return allLegs[minDev]

def compareStdDeviation(list1,list2):
   std1 = 0
   std2 = 0
   for sqr in list1:
      std1 += sqr[2]
   std1 = np.std(std1)

   for sqr in list2:
      std2 += sqr[2]
   std2 = np.std(std2)

   if std1 < std2:
      return list1
   else: 
      return list2

def printLegs(linedSqrs):

	for i in range (0, len(linedSqrs)):
		#print linedSqrs[i]
		b = int ( random.uniform(50,255))
		g = int ( random.uniform(50,255))
		r = int ( random.uniform(50,255))
		for j in range(0,len(linedSqrs[i])):
			x = int(linedSqrs[i][j][0])
			y = int(linedSqrs[i][j][1])
			cv2.circle(imgOriginal,(x,y),5,(b,g,r),-1)

# returns True is X coord is between legs of cow
def betweenLegs(myLegs,xt):
	if ( xt >= myLegs[0][0][0] and xt <= myLegs[len(myLegs) - 1][0][0] ):
		return True
	else:
		return False

# returns average of all legs
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

# returns a list with indivual mass averages of legs
def getLegsMassCenter(myLegs):

	legsMassCenter = []
	xCenter = 0
	yCenter = 0
	xTemp = 0
	yTemp = 0
	for leg in myLegs:
		for i in range(len(leg)):
			xTemp = xTemp + leg[i][0]
			yTemp = yTemp + leg[i][1]

		xCenter = xTemp / len(leg)
		yCenter = yTemp / len(leg)
		legsMassCenter.append((xCenter,yCenter))
		xTemp = 0
		yTemp = 0

	return legsMassCenter

def printLegsMassCenter(legsMassCenter):

	for i in range (len(legsMassCenter)):
		x = legsMassCenter[i][0]
		y = legsMassCenter[i][1]
		cv2.circle(imgOriginal,(x,y),7,(255,0,0),-1)

#def internalDistEquality(line):

''' currently working on this one '''
def getHorizontalLines(cowSquares):
	# soting by X
	tempSquares = sorted(cowSquares, key=lambda x: x[4],reverse=False)
	# sorting by Y
	tempSquares = sorted(cowSquares, key=lambda x: x[5],reverse=False)
	allLines = []
	tempLine = []
	for i in range (len(tempSquares)):
		ari,exi,wi,hi,xi,yi = tempSquares[i]
		tempLine = []
		tempLine.append((xi,yi,ari))
		for j in range (i, len(tempSquares)):
			xj = tempSquares[j][4]
			yj = tempSquares[j][5]
			arj = tempSquares[j][0]
			'''
			define this epsilon and dont change it any more
			each cow will vary on this
			'''
			eps = 10
			if (abs(yi-yj) < eps):
				tempLine.append((xj,yj,arj))
			else:
				break


		if (len(tempLine)>3):
			stList = []
			for x in range(len(tempLine)):
				stList.append(tempSquares[x][2])
				stDeviation = np.std(stList)
         	#print "Std Deviation: ", stDeviation
			# ADJUST THIS IF EXPERIMENTALLY TO WORK EFFICIENT
			if(stDeviation < 50):
				allLines.append(tempLine)

	'''
	after this cicle we must have our candidates
	time to apply size filters, and internal distance filters
	we may have lines inside other lines
	'''

	return allLines

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
			neigh.append(cowSquares[i])

	return neigh


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
	



	myLegs = getMyLegs(cowRectangles,10,3) # epsilon and >=minSqrs
	if len(myLegs) >= 2:
		cowRows = getHorizontalSqrs(cowRectangles, myLegs)
		# if len(cowRows > 2):
		# 	print "si hay patas"
		x, y  = legsCenter(myLegs)
		cv2. circle(imgOriginal, (x,y),5,(0,255,0),-1)

	cowRectangles = rb.sortList(4,cowRectangles)
	#printLegs(myLegs)
	print "numero de patas :", len(myLegs)
	legsMassCenter = getLegsMassCenter(myLegs)
	printLegsMassCenter(legsMassCenter)
	print "Centros de Masa :", legsMassCenter

	print "Todas las lineas a continuacion"
	n = neighboors(cowRectangles)
	cowRows = getHorizontalSqrs(n,myLegs)
	print "lineas :", cowRows
	# l = getHorizontalLines(cowRectangles)
	# print l

	# for i in l:
	# 	for j in i:
	# 		x = int(j[0])
	# 		y = int(j[1])
	# 		cv2.circle(imgOriginal,(x,y),5, (255,0,0), -1 )

	# for i in l:
	# 	xi = i[0][0]
	# 	yi = i[0][1]
	# 	xj = i[len(i)-1][0]
	# 	yj = i[len(i)-1][1]
	# 	cv2.line(imgOriginal,(xi,yi),(xj,yj),(255,255,255),5)


	# print "total :", len(l)
		


	
	x = 720 / 2 
	y = 480 / 2
	cv2. circle(imgOriginal, (x,y),5,(0,0,255),-1)
	theta,m,b = aj.ajusteDeCurvas(n,len(n))
	x1 = 50
	x2 = 600
	y1 = int(x1*m+b)
	y2 = int(x2*m+b)
	cv2.line(imgOriginal,(x1,y1),(x2,y2),(255,255,0),3)
	cv2.imshow(filename,imgOriginal)
	cv2.waitKey(0)


	


loop()
cv2.destroyAllWindows()

