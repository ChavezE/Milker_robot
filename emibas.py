import cv2
import numpy as np


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
		#

		# looking for left-est square

		listaCuadros = self.upperLeftCorner(listaCuadros) # listaCuadros is sorted in x
		mostLeft = listaCuadros[0]
		self.leftLeg.insert(0,mostLeft)
		self.findLeftLeg(listaCuadros)
		self.leftLeg = sorted(self.leftLeg, key=lambda x: x[1],reverse=False)
		self.contLeftLeg = len(self.leftLeg)


	def findLeftLeg(self, listaCuadros):
		# Define 'epsilon' which is the tolerance in px to find the other
		# coordinates that form the left Leg
		epsilon = 10
		# Looking for the coordinates that have the same 'x' coordinate
		for i in range(1, len(listaCuadros)):
			if(abs(listaCuadros[i][0] - self.leftLeg[0][0]) < epsilon):
				self.leftLeg.insert(len(self.leftLeg),listaCuadros[i])
			else:
				break

	def upperLeftCorner(self,list1):
		# 'epsilon' is the error range in pixels
		epsilon = 10
		# The function "sorted" sorts a multidimensional list. Depending on x[0]
		# is the index that you use to compare
		list1 = sorted(list1, key=lambda x: x[0], reverse=False)
		# Now that we have the list ordered from left to right we need to find the
		# coordinate that is closest to the origin (0,0)
		for i in range(0, len(list1)):
			if abs(list1[0][0] - list1[i][0]) <= epsilon:
				if list1[0][1] > list1[i][1]:
					temp = list1[0]
					list1[0]=list1[i]
					list1[i]=temp
				else:
					break
		return list1




####################################
# Emibas.py
# Here we go...
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
linedSquares = []
index = 0

######### VERTICAL SQUARES ##################
""" all vertical aligned squares are stored in linedSquares
# criteria of comparaison is the center of rectangle
upper left coodenates are stored in lined squares as subArrays """
for i in range(0,total):
	xi,yi,wi,hi = cv2.boundingRect(allSquares[i])
	centeri = (xi + wi) / 2   # getting 1st center
	foundOthers = False
	for j in range(i+1,total):
		xj,yj,wj,hj = cv2.boundingRect(allSquares[j])
		centerj = (xj + wj)/2 	# finding 2nd center
		if abs(centerj - centeri) < 5:
			# if already in list do not insert
			if (linedSquares.count([xj,yj])) < 1:
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

patas = Mylegs(linedSquares)
pAux = patas.contLeftLeg
x = patas.leftLeg[0][0]
y = patas.leftLeg[0][1]
print patas.leftLeg
img = cv2.circle(img,(x,y),5,(0,255,0),-1)

cv2.imshow('o',img)
cv2.waitKey(0)
print linedSquares
print index
print len(linedSquares)
print ''
cv2.destroyAllWindows()
