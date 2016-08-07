import cv2
import time
import math
import numpy as np
import LARC1 as rb

filename = 'image23.jpg'
binValue = 65 # parameter for the threshold


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
# cowSquares is an emibas list, epsilon is tolerance 
# this function compares squares in X axis
def getVerticalSqrs(cowSquares, epsilon):
	linedSquares = []
	for i in range(0,len(cowSquares) - 1):
		xi = cowSquares[i][4]
		yi = cowSquares[i][5]
		foundOthers = False
		for j in range(i + 1,len(cowSquares) - 1):
			xj = cowSquares[j][4]
			yj = cowSquares[j][5]
			if abs(xj - xi) < epsilon:
				# if already in list do not insert
				if (linedSquares.count([xj,yj])) < 1:
					linedSquares.append([xj,yj])
					foundOthers = True
		if foundOthers: # inserting the first one that found others
			linedSquares.append([xi,yi])

	return linedSquares
# This class would store rectangles
# and sort them to be used as vertical
# indicators
class Mylegs:
	""" squares here """


	def __init__(self, listaCuadros):

		# variables
		self.leftLeg = [] # here we store coordenates of left leg
		self.contLeftLeg = 0 # This will tell us the size of the list 'leftLeg'
		self.rightLeg = [] # coord for right leg
		self.contRightLeg = 0 # This will tell us the size of the list 'rightLeg'
		self.epsilon = 20 # This is the range of error when finding the legs
		#

		# ---Filling up leftLeg---

		# listaCuadros is sorted with the upperLeftCorner in the index '0'
		listaCuadros = self.upperLeftCorner(listaCuadros)
		mostLeft = listaCuadros[0]
		# insert the mostLeft coordinate as the first element of the left leg
		self.leftLeg.insert(0,mostLeft)
		# insert all of the other coords that build up the left leg
		self.findLeftLeg(listaCuadros)
		# At last, the leftLeg coords are sorted from top to bottom
		self.leftLeg = sorted(self.leftLeg, key=lambda x: x[1],reverse=False)
		self.contLeftLeg = len(self.leftLeg)

		# ---Filling up leftLeg---

		# is pretty much the same process as filling up leftLeg
		listaCuadros = self.upperRightCorner(listaCuadros)
		mostRight = listaCuadros[0]
		self.rightLeg.insert(0, mostRight)
		self.findRightLeg(listaCuadros)
		self.rightLeg = sorted(self.rightLeg, key=lambda x: x[1],reverse=False)
		self.contRightLeg = len(self.rightLeg)


	def findLeftLeg(self, listaCuadros):
		# Looking for the coordinates that have the same 'x' coordinate
		for i in range(1, len(listaCuadros)):
			if(abs(listaCuadros[i][0] - self.leftLeg[0][0]) < self.epsilon):
				self.leftLeg.insert(len(self.leftLeg),listaCuadros[i])
			else:
				break

	def upperLeftCorner(self,list1):

		# The function "sorted" sorts a multidimensional list. Depending on x[0]
		# is the index that you use to compare
		list1 = sorted(list1, key=lambda x: x[0], reverse=False)
		# Now that we have the list ordered from left to right we need to find the
		# coordinate that is closest to the origin (0,0)
		for i in range(0, len(list1)):
			if abs(list1[0][0] - list1[i][0]) <= self.epsilon:
				if list1[0][1] > list1[i][1]:
					temp = list1[0]
					list1[0]=list1[i]
					list1[i]=temp
				else:
					break
		return list1

	def findRightLeg(self, listaCuadros):
		# Define 'epsilon' which is the tolerance in px to find the other
		# coordinates that form the right Leg
		epsilon = 10
		# Looking for the coordinates that have the same 'x' coordinate
		for i in range(1, len(listaCuadros)):
			if(abs(listaCuadros[i][0] - self.rightLeg[0][0]) < self.epsilon):
				self.rightLeg.insert(len(self.rightLeg),listaCuadros[i])
			else:
				break

	def upperRightCorner(self,list1):
		# 'epsilon' is the error range in pixels
		epsilon = 10
		# The function "sorted" sorts a multidimensional list. Depending on x[0]
		# is the index that you use to compare
		list1 = sorted(list1, key=lambda x: x[0], reverse=True)
		# Now that we have the list ordered from right to left we need to find the
		# coordinate that is the top of the right leg
		for i in range(0, len(list1)):
			if abs(list1[0][0] - list1[i][0]) <= self.epsilon:
				if list1[0][1] > list1[i][1]:
					temp = list1[0]
					list1[0]=list1[i]
					list1[i]=temp
				else:
					break
		return list1
	




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
	# Getting Contours here (and other thing we're not using)
	# image, contours, hierarchy = cv2.findContours(thresImage,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

	cowRectangles = rb.getGoodSquares(contours,imgOriginal)
	# We have this order 
	# [area,extent,w,h,x,y]
	cowRectangles = rb.sortList(5,cowRectangles)
	
	font = cv2.FONT_HERSHEY_SIMPLEX # This line defines the font 
	# cv2.drawContours(imgOriginal,allRect,-1,(0,255,0),1)
	# second parameter is number of cow lines
	# third parameter is epsilon between same lines in pxls
	myBody= rb.getBody(cowRectangles,imgOriginal,3,15)
	linedSquares = getVerticalSqrs(cowRectangles,5 )


	'''for i in range(0,3):
		print myBody[i]

	tam = len(myBody[0])
	Yangle1 = float(myBody[0][0][1])
	Yangle2 = float(myBody[0][tam - 1][1])
	Xangle1 = float(myBody[0][0][0])
	Xangle2 = float(myBody[0][tam - 1][0])
	if(Xangle2 - Xangle1) != 0:

		param = float((Yangle2 - Yangle1) / (Xangle2 - Xangle1))
		print param
		print "%d %d %d %d"% (Yangle2, Yangle1, Xangle2 , Xangle1)
		angle = math.degrees(math.atan(param))
		print angle
		text1 = "Angle : " + str(angle)
		cv2.putText(imgOriginal,text1,(50,50), font, 0.5,(0,0,255),1  ,cv2.LINE_AA)
	'''
	for i in range(0,len(cowRectangles)):
		print (cowRectangles[i])
	if len(linedSquares) != 0:
	# showing squares
		for i in range(0,len(linedSquares)):
			x = linedSquares[i][0]
			y = linedSquares[i][1]
			print (x,y)
			cv2.circle(imgOriginal,(x,y), 5 , (0,0,255),-1)

		patas = Mylegs(linedSquares)
		pAux = patas.contLeftLeg
		xi = patas.leftLeg[0][0]
		yi = patas.leftLeg[0][1]
		xd = patas.rightLeg[0][0]
		yd = patas.rightLeg[0][1]
		print patas.leftLeg
		cv2.circle(imgOriginal,(xi,yi),5,(0,255,0),-1)
		cv2.circle(imgOriginal,(xd,yd),5,(0,255,0),-1)


	cv2.imshow(filename,imgOriginal)
	cv2.waitKey(0)

loop()
cv2.destroyAllWindows()

