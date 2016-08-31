'''
This is version one of LARC2016 Module of Mexican Team RoBorregos
here there are stored most of functions used by the artificial vision
squad. All rights reserved by Instituo Tecnologico de Monterrey

'''
import cv2
import time
import math
import numpy as np

######	 	LIST OF FUNCTIONS 	#######
#######################################



# Does some guassian filtering to remove noise
def clearImage(imgOriginal):
	imGray = cv2.cvtColor(imgOriginal, cv2.COLOR_BGR2GRAY)
	imGray = cv2.GaussianBlur(imGray, (3,3), 2)

	return imGray

def doThresHold(filteredImage,tVal):
	_, thres1 = cv2.threshold(filteredImage,tVal,255,cv2.THRESH_BINARY_INV)
	thres1 = cv2.erode(thres1,np.ones((5,5),np.uint8), iterations=2)
	return thres1

def centerOfContour(contour): # Returns the coordinates of the contour's center
   M = cv2.moments(contour)
   cx = int(M['m10']/M['m00'])
   cy = int(M['m01']/M['m00'])
   return cx, cy

def printContourCoords(cx,cy,x,y):
   print "Center: %d, %d   LeftTopCorner: %d, %d" % (cx,cy,x,y)

# INPUT: List 
def calculateAverages(cowSquares):
   avgExtent = 0
   avgArea = 0
   avgArea2 = 0

   if(len(cowSquares) != 0):
      for i in range(0, len(cowSquares)):
         avgExtent += cowSquares[i][4]
         avgArea += cowSquares[i][5]
         avgArea2 += cowSquares[i][2] * cowSquares[i][3]
         
      avgExtent /= len(cowSquares)
      avgArea /= len(cowSquares)
      avgArea2 /= len(cowSquares)
   return (avgExtent,avgArea,avgArea2)

def findContours(img):
   _, contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, 
      cv2.CHAIN_APPROX_SIMPLE)
   return contours

def sortList(index,l): # This functions sorts a multivaraible list depending on the index specified
   l = sorted(l, key=lambda x: x[index],reverse=False) 
   return l

def findMedian(index,l):
   if (len(l) % 2) != 0:     # The length of the list is odd
      return l[len(l)/2][index]
   else:              # The lenght of the list is even
      return (l[len(l)/2][index] + l[len(l)/2 + 1][index])/2

def printCowSquares(imgOriginal,G,B,R,sqrs):
   for sqr in sqrs:
      w = sqr[2]
      h = sqr[3]
      x = sqr[4]
      y = sqr[5]
      cv2.rectangle(imgOriginal,(x,y),(x+w,y+h),(G,B,R),2)

# This method will recibe all countours in image and
# will make all necesary filtering in order to achive
# countors with high probability of being cow rectangles
# it returns a mutidimensional list like this: 
#     [][area,extent,w,h,x,y]
# should be changed to [][x,y,w,h,area,extent] 
# list is sorted with respect to WHAT?
def getGoodSquares(contours,imgOriginal):

   # ----VARIABLES----
   cowSqrs = [] # This list is the one that is going to being returned
   tempCowSqrs = []
   # -----------------

   # 1st - Search through the list to find the candidates to be cow squares
   # Data is stored in lists "allR" and "cowSqrs"
   for cnt in contours:
      area = cv2.contourArea(cnt)
      rect = cv2.minAreaRect(cnt)
      #x = int(rect[0][0])
      #y = int(rect[0][1])
      w = int(rect[1][0])
      h = int(rect[1][1])
      rect_area = w * h
      if(rect_area > 0): # sometimes this value is found
         extent = float(area / rect_area)
         if (extent >= 0.7 and area >= 50):   # tolerance
            x,y,w,h = cv2.boundingRect(cnt)
            aspect_ratio = float(w)/h
            if aspect_ratio > 0.2 and aspect_ratio < 2:
               cowSqrs.append([area,extent,w,h,x,y])

   # Print elements in cowSqrs
   for sqr in cowSqrs:
      w = sqr[2]
      h = sqr[3]
      x = sqr[4]
      y = sqr[5] 
      #cv2.circle(imgOriginal,(x,y),3,(255,255,0),-1)
      cv2.rectangle(imgOriginal,(x,y),(x+w,y+h),(255,255,255),3)
   
   # AT THE END, RETURN THE LIST THAT BELONG TO THE COW                                     
   return cowSqrs

# This function is still on progress, hope to return well ordered body of cow
# in a convenient way 
def getBody(cowRectangles,imgOriginal,totLines,epsilon):

	bodyLines = []
	singleLine = []
	yTest = cowRectangles[0][5]
	i = 0
	b = g = r = 0

	for count in range(0,totLines):
		if count == 0:
			b = 255
		elif count == 1:
			g = 255
		elif count == 2:
			r = 255
		while(abs(cowRectangles[i][5] - yTest) < epsilon):
			x = cowRectangles[i][4]
			y = cowRectangles[i][5]
			#cv2.circle(imgOriginal,(x,y),5,(b,g,r),-1)
			#bodyLines.insert(0,allRect[i])
			singleLine.append((x,y))
			i += 1
		bodyLines.append(singleLine)
		singleLine = []
		print count
		yTest = cowRectangles[i][5]

	return bodyLines


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

def ajusteDeCurvas(bodyCoords):
   # These lists and variables are used to calculate A and B
   # to get a polynomial of 1st grade: y = Ax + B
   n = len(bodyCoords) 
   Xi = []
   Yi = []
   XiYi = []
   Xi2 = []
   # This cycle is to fill up all the lists in order to compute A and B
   for coord in bodyCoords:
      xT = coord[0]
      yT = coord[1]
      Xi.append(xT)
      Yi.append(yT)
      XiYi.append(xT*yT)
      Xi2.append(pow(xT,2))
   # The reason to create lists is to get the sum of all its elements 
   # so it will be computed the sum of each of the lists
   sXi = sum(Xi)
   sYi = sum(Yi)
   sXiYi = sum(XiYi)
   sXi2 = sum(Xi2)
   # Now its time to compute A and B
   A = float((n*sXiYi)-(sXi*sYi))/((n*sXi2)-(pow(sXi,2)))
   B = float(((sXi2*sYi)-(sXiYi*sXi))/((n*sXi2)-(pow(sXi,2))))
   theta = float(math.atan(A))
   theta = float(theta*180/math.pi)
   return theta,A,B

# This function is to implement the clustering algorithm 
# PARAMETERS: number of clusters to search for, list of coordinates of the cow
# and number of iterations before returning the final clusters
def findClusters(num_clusters, cowRectangles,iterations):
   # First, the list clusters is initialized with n number of Clusters
   clusters = []
   for iA in range(num_clusters):
      tList = []
      cluster = Cluster([0,0],tList,tList)
      clusters.append(cluster)
   clusters[0].set_center([320,0])
   clusters[1].set_center([0,480])
   clusters[2].set_center([640,480])
   for dummy_iterator in range(iterations):
      for rect in cowRectangles:
         dist = float('inf')
         index = 0
         for i in range(len(clusters)):
            actClust = clusters[i]
            if(actClust.distance(rect) < dist):
               dist = actClust.distance(rect)
               index = i
         clusters[index].add_point(rect)
      for cluster in clusters:
         cluster.update_points()
         cluster.calculate_center()

   return clusters





