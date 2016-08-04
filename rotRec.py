import cv2
import time
import math
import numpy as np
import LARC1 as rb

filename = 'image22 .jpg'
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

################################################
############### MAIN LOOP ######################
################################################
def loop():

	
	filteredImage = rb.clearImage(imgOriginal)
	thresImage = rb.doThresHold(filteredImage,binValue)
	cv2.imshow('T',thresImage)

	contours = rb.findContours(thresImage)
	# Getting Contours here (and other thing we're not using)
	# image, contours, hierarchy = cv2.findContours(thresImage,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

	cowRectangles = rb.getGoodSquares(contours,imgOriginal)
	# We have this order 
	# [area,extent,w,h,x,y]
	rb.sortList(5,cowRectangles)
	
	font = cv2.FONT_HERSHEY_SIMPLEX # This line defines the font 
	# cv2.drawContours(imgOriginal,allRect,-1,(0,255,0),1)
	# second parameter is number of cow lines
	# third parameter is epsilon between same lines in pxls
	myBody= rb.getBody(cowRectangles,imgOriginal,3,15)

	for i in range(0,3):
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

	cv2.imshow(filename,imgOriginal)
	cv2.waitKey(0)

loop()
cv2.destroyAllWindows()

