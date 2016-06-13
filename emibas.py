import time
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



##############################################################################
# emibas.py
# Here we go...

# For analysing a single image
filename = 'image.jpg'
#imgOriginal = cv2.imread(filename)
# Video capture
cap = cv2.VideoCapture(0)

for counter in range(0,20):
	_, imgOriginal = cap.read()

while(True):
	_, imgOriginal = cap.read()

	"""Here the capture is binarized and filtred. We also find all Contours
		and they are stored in contours[]. 				"""
	# open to changes
	binValue = 80 # parameter for the threshold
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
		# centeri = (xi + wi) / 2    getting 1st center
		foundOthers = False
		for j in range(i+1,total):
			xj,yj,wj,hj = cv2.boundingRect(allSquares[j])
			# centerj = (xj + wj)/2 	# finding 2nd center
			if abs(xj - xi) < 5:
				# if already in list do not insert
				if (linedSquares.count([xj,yj])) < 1:
					linedSquares.insert(0,[xj,yj])
					index = index + 1
					foundOthers = True
		if foundOthers: # inserting the first one that found others
			linedSquares.insert(0,[xi,yi])
			index = index + 1

	if len(linedSquares) != 0:
	# showing squares
		for i in range(0,index):
			x = linedSquares[i][0]
			y = linedSquares[i][1]
			img = cv2.circle(img,(x,y), 5 , (0,0,255),-1)

		patas = Mylegs(linedSquares)
		pAux = patas.contLeftLeg
		xi = patas.leftLeg[0][0]
		yi = patas.leftLeg[0][1]
		xd = patas.rightLeg[0][0]
		yd = patas.rightLeg[0][1]
		print patas.leftLeg
		img = cv2.circle(img,(xi,yi),5,(0,255,0),-1)
		img = cv2.circle(img,(xd,yd),5,(0,255,0),-1)

		cv2.imshow('o',img)
	else:
		print "No squares found"

	k = cv2.waitKey(5) & 0xFF
	if k == 27:
		break
	#cv2.waitKey(0)
	#print linedSquares
	#print index
	#print len(linedSquares)
	#print ''
cv2.destroyAllWindows()
cap.release()
