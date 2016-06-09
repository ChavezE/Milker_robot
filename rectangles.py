# Emibas.py
# Here we go...
import cv2
import numpy as np


filename = 'image.jpg'

imgOriginal = cv2.imread(filename)

# Here the capture is binarized and filtred
# we also find all Contours and they are stored
# in contours
#
# open to changes
binValue = 50 # parameter for the threshold
##########################################################################################
imGray = cv2.cvtColor(imgOriginal, cv2.COLOR_BGR2GRAY)
imGray = cv2.GaussianBlur(imGray, (3,3), 2)
_, thres1 = cv2.threshold(imGray,binValue,255,cv2.THRESH_BINARY_INV)
thres1 = cv2.erode(thres1,np.ones((5,5),np.uint8))
image, contours, hierarchy = cv2.findContours(thres1,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
#cv2.imshow('-',imGray)
##########################################################################################

total = len(contours)
allSquares = []

# Here we analized contours and obtain rectangles
for cnt in range(0,total):
	area = cv2.contourArea(contours[cnt])
	x,y,w,h = cv2.boundingRect(contours[cnt])
	rect_area = w*h
	extent = float(area)/rect_area
	if (extent >= 0.8):   # tolerance
		img = cv2.drawContours(imgOriginal, contours, cnt, (0,255,0), 2)
		allSquares.insert(0,contours[cnt])


total = len(allSquares)
print total
linedSquares = []
index = 0

######### VERTICAL SQUARES ##################
# all vertical aligned squares are stored in linedSquares
# criteria of comparaison is the center of rectangle
# upper left coodenates are stored in lined squares as subArrays
for i in range(0,total):
	xi,yi,wi,hi = cv2.boundingRect(allSquares[i])
	centeri = (xi + wi) / 2   # getting 1st center
	foundOthers = False
	for j in range(i+1,total):
		xj,yj,wj,hj = cv2.boundingRect(allSquares[j])
		centerj = (xj + wj)/2 	# finding 2nd center
		if abs(centerj - centeri) < 5:
			if (linedSquares.count([xj,yj])) < 1: # if already in list do not insert
				linedSquares.insert(0,[xj,yj])
				index = index + 1
				foundOthers = True
	if foundOthers: # inserting the first one that found others
		linedSquares.insert(0,[xi,yi])
		index = index + 1

# showing squares
for i in range(0,index):
	x = linedSquares[i][0]
	y = linedSquares[i][1]
	img = cv2.circle(img,(x,y), 5 , (0,0,255),-1)

cv2.imshow('o',img)
cv2.waitKey(0)
print linedSquares
print index
print len(linedSquares)
print ''
cv2.destroyAllWindows()
