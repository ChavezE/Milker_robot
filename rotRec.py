import cv2
import time
import numpy as np

filename = 'image2.jpg'
imgOriginal = cv2.imread(filename)
imgOriginal = cv2.resize(imgOriginal, (720 ,480))

# This method will recibe all countours in image and
# will make all necesary filtering in order to achive
# countors with high probability of being cow rectangles
def getGoodSquares(contours):

	allR = [] # temporal for storing
	cowSqrs = [] 
	for cnt in contours:
		area = cv2.contourArea(cnt)
		rect = cv2.minAreaRect(cnt)
		#x = int(rect[0][0])
		#y = int(rect[0][1])
		w = int(rect[1][0])
		h = int(rect[1][1])
		#angle = rect [2] 
		#box = cv2.boxPoints(rect)
		#box = np.int0(box)
		rect_area = w*h
		if(rect_area>0): # sometimes this value is found
			extent = float(area/rect_area)
			if (extent >= 0.7):   # tolerance
				allR.insert(0,cnt)
				cowSqrs.insert(0,[area,extent])
        cv2.drawContours(imgOriginal, allR, -1, (0,255,0))
	# FILTER SQUARES FOR ITS AREA
	(allR, cowSqrs) = zip(*sorted(zip(allR, cowSqrs),
		key=lambda b:b[1][0], reverse=False))
	
	medianArea = 0

	# HERE WE CALCULATE THE MEDIAN VALUE OF AREA FOR ALL THE CONTOURS IN allR
	if (len(cowSqrs) % 2) != 0:     # The length of the list is odd
		medianArea = cowSqrs[len(cowSqrs)/2][0]
	# The lenght of the list is even
	else:
		medianArea = (cowSqrs[len(cowSqrs)/2][0] + cowSqrs[len(cowSqrs)/2 + 1][0])/2

        for a in cowSqrs:
                print a
        print medianArea

        tempAllR = []
	for cnt in allR:
		areaTemp = cv2.contourArea(cnt)
		if not((areaTemp > medianArea * 1.75) or (areaTemp < medianArea * 0.25)):
                       tempAllR.insert(0, cnt) 
        allR = tempAllR
        cv2.drawContours(imgOriginal, allR, -1, (255,0,0), 4)

        


	# FILTER squares for its POSITION on the image
	# STILL NOT WORKING
	'''
	for cnt in allR:
		x,y,w,h = cv2.boundingRect(cnt)
		# First we assume that 'cnt' has no neighbours neither in x nor in y axis
		xNeig = False
		yNeig = False
		for cntT in allR:
			xT,yT,wT,hT = cv2.boundingRect(cntT)
			# Before serching for neighbours, we need to be sure we are analyzing 2 different contours
			if(x != xT and y != yT and w != wT and h != hT): 
				# Check if the contours has a neighbour 
				if (abs(y - yT) < h):
				# Has neighbour in y axis
					yNeig = True
				if (abs(x - xT) < w):
				# Has neighbour in x axis
					xNeig = True

		# If no neihgbour is found, the contour is deleted from the list
		if not yNeig or not xNeig:
			i = findIndex(cnt, allR)
			allR.pop(i)
	'''
	

                                                
	return allR

# This function is still on progress, hope to return well ordered body of cow
# in a convenient way 
def getBoddy(allRect,boundingBoxes,bodyLines,epsilon):

	i = 0
	# Testing Y coord 
	while abs(boundingBoxes[i][1] - boundingBoxes[i+1][1]) < epsilon and i < len(boundingBoxes):
		x = boundingBoxes[i][0]
		y = boundingBoxes[i][1]
		cv2.circle(imgOriginal,(x,y),3,(255,0,0),-1)
		i += 1

	# drawing last one
	x = boundingBoxes[i][0]
	y = boundingBoxes[i][1]
	cv2.circle(imgOriginal,(x,y),3,(255,0,0),-1)

# This method will sort countours respect to Y or X coord of bounding Rectangle
# Critieria defines whether X o Y is used DEFAULT IS y
# This is always from lef to right of from top to bottom --> Reverse = False
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
	allRect, boundingBoxes = boundingRectSort(allRect,'y')
	""" At this point we have all posible squares filtered and stored  AS COUNTOURS in allRect """

	font = cv2.FONT_HERSHEY_SIMPLEX # This line defines the font 

        cv2.drawContours(imgOriginal,allRect,-1,(0,255,0),1)
        """
	for cnt in range(0,len(allRect)):
		x = boundingBoxes[cnt][0]
		y = boundingBoxes[cnt][1]
		im = cv2.drawContours(imgOriginal,allRect,cnt,(0,255,0),3)
		im = cv2.circle(imgOriginal,(x,y), 3, (255,0,0), -1)
		#text = str(cnt)
		#cv2.putText(imgOriginal,text,(x,y), font, 1,(0,0,255),1,cv2.LINE_AA)
	"""	
	cv2.imshow('im',imgOriginal)
	cv2.waitKey(0)

loop()
cv2.destroyAllWindows()
