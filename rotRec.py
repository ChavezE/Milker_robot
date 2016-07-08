import cv2
import time
import numpy as np

filename = 'C5.jpg'
imgOriginal = cv2.imread(filename)
imgOriginal = cv2.resize(imgOriginal, (720 ,480))

# This method will recibe all countours in image and
# will make all necesary filtering in order to achive
# countors with high probability of being cow rectangles
def getGoodSquares(contours):
	allR = [] # temporal for storing 
	for cnt in contours:
		area = cv2.contourArea(cnt)
		rect = cv2.minAreaRect(cnt)
		#x = int(rect[0][0])
		#y = int(rect[0][1])
		w = int(rect[1][0])
		h = int(rect[1][1])
		#angle = rect [2] 
		box = cv2.boxPoints(rect)
		box = np.int0(box)
		rect_area = w*h
		if(rect_area>0):
			extent = float(area/rect_area)
			if (extent >= 0.7):   # tolerance
				allR.insert(0,cnt)

	# ADD HERE MEDIAN SIZE FILTERING
	# ADD HERE MEDIAN SIZE FILTERING
	# ADD HERE MEDIAN SIZE FILTERING

	# ADD ANY OTHER FILTER
	# ADD ANY OTHER FILTER
	# ADD ANY OTHER FILTER

	return allR
	
# This method will sort countours respect to Y or X coord of bounding Rectangle
# Critieria defines whether X o Y is used DEFAULT IS Y
# This is always from lef to right of from top to bottom --> Reverse
# Most of this is from here :	http://www.pyimagesearch.com/2015/04/20/sorting-contours-using-python-and-opencv/
def boundingRectSort(allRect,criteria):

	i = 1
	if criteria == 'X' or criteria ==  'x':
		i = 0
	boundingBoxes = [cv2.boundingRect(c) for c in allRect]
	(allRect, boundingBoxes) = zip(*sorted(zip(allRect, boundingBoxes),
		key=lambda b:b[1][i], reverse=False))
 
	return (allRect, boundingBoxes)

################################################
############### MAIN LOOP ######################
################################################
def loop():

	binValue = 65 # parameter for the threshold
	##########################################################################################
	imGray = cv2.cvtColor(imgOriginal, cv2.COLOR_BGR2GRAY)
	imGray = cv2.GaussianBlur(imGray, (3,3), 2)
	_, thres1 = cv2.threshold(imGray,binValue,255,cv2.THRESH_BINARY_INV)
	thres1 = cv2.erode(thres1,np.ones((5,5),np.uint8), iterations=2)
	cv2.imshow('t',thres1)
	image, contours, hierarchy = cv2.findContours(thres1,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
	##########################################################################################

	allRect = getGoodSquares(contours)
	boundingBoxes = [] # will be used in parallel with allRect

	""" At this point we have all posible squares filtered and stored  AS COUNTOURS in allRect """
	allRect, boundingBoxes = boundingRectSort(allRect,'y')

	font = cv2.FONT_HERSHEY_SIMPLEX

	for cnt in range(0,len(allRect)):
		x = boundingBoxes[cnt][0]
		y = boundingBoxes[cnt][1]
		im = cv2.drawContours(imgOriginal,allRect,cnt,(0,255,0),3)
		im = cv2.circle(imgOriginal,(x,y), 3, (255,0,0), -1)
		text = str(cnt)
		cv2.putText(imgOriginal,text,(x,y), font, 1,(0,0,255),1,cv2.LINE_AA)





	cv2.imshow('im',imgOriginal)
	cv2.waitKey(0)

loop()
cv2.destroyAllWindows()