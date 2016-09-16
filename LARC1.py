
"""
   This is version 1.0 of LARC 2016 Module of Mexican Team RoBorregos
   here there are stored most of functions used by the artificial vision
   squad. All rights reserved by Instituo Tecnologico de Monterrey.

   AUTHORS:    Emilio Chavez Madero
            Sebastian Rivera Gonzalez

"""

# ------ LIBRARIES ----------
import cv2
import time
import math
import numpy as np
import random
# ---------------------------

######	 	LIST OF FUNCTIONS 	#######
#######################################

# CLUSTERING CLASS that works along with the 'findClusters' function
class Cluster:
   """Clustering modafoca points"""
   # atributes
   center = []
   new_points = []
   old_points = []
   # methods
   def __init__(self, init_center):
      self.center = init_center
      self.new_points = []
      self.old_points = []
   def calculate_center(self):
      x_sum = 0
      y_sum = 0
      for i in range (len(self.new_points)):
         x_sum = x_sum + self.new_points[i][0]
         y_sum = y_sum + self.new_points[i][1]
      if(len(self.new_points) > 0):
         x_center = int(x_sum/len(self.new_points))
         y_center = int(y_sum/len(self.new_points))
         self.center = (x_center,y_center)
   def add_point(self,new_p):
      self.new_points.append(new_p)
      """ 
      making this not visible

      """
   def set_center(self,point):
      self. center = point
      """ 
      making this not visible

      """
   #def set_points(self):
   def distance(self,p):
      x = p[0]
      y = p[1]
      xDif = abs(x - self.center[0])
      yDif = abs(y - self.center[1])
      return math.sqrt((xDif*xDif) + (yDif*yDif))
      """ 
      making this not visible

      """
   def update_points(self):
      self.old_points = list(self.new_points)
      self.new_points = []
   def get_center(self):
      return self.center
   def get_new_points(self):
      return list(self.new_points)
   def get_old_points(self):
      return list(self.old_points)
      
# Does some guassian filtering to remove noise and converts image to gray scale :)
def clearImage(imgOriginal):

   imgOriginal[480*0.75:480,0:640] = [255,255,255]
   imGray = cv2.cvtColor(imgOriginal, cv2.COLOR_BGR2GRAY)
   imGray = cv2.GaussianBlur(imGray, (3,3), 2)

   return imGray

def doThresHold(filteredImage,tVal):
	_, thres1 = cv2.threshold(filteredImage,tVal,255,cv2.THRESH_BINARY_INV)
	thres1 = cv2.erode(thres1,np.ones((3,3),np.uint8), iterations=5)
	return thres1

def centerOfContour(contour): # Returns the coordinates of the contour's center
   M = cv2.moments(contour)
   cx = int(M['m10']/M['m00'])
   cy = int(M['m01']/M['m00'])
   return cx, cy

def printContourCoords(cx,cy,x,y):
   print "Center: %d, %d   LeftTopCorner: %d, %d" % (cx,cy,x,y)

# Finds contours in the image and returns them as a numpy list. Recieves the image
# as a parameter.
def findContours(img):
   _, contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, 
      cv2.CHAIN_APPROX_SIMPLE)
   return contours

def sortList(index,l): # This functions sorts a multivaraible list depending on the index specified
   l = sorted(l, key=lambda x: x[index],reverse=False) 
   return l

def findMedian(index,l):
   # The length of the list is odd
   if (len(l) % 2) != 0:     
      return l[len(l)/2][index]
   # The lenght of the list is even
   else:              
      return (l[len(l)/2][index] + l[len(l)/2 + 1][index])/2

# Receives the list 'sqrs' which contains the data in this order: [area,extent,w,h,x,y]
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
#     [k][area,extent,w,h,x,y]
def getGoodSquares(contours,imgOriginal):

   # ----VARIABLES----
   cowSqrs = [] # This list is the one that is going to being returned
   tempCowSqrs = []
   # -----------------

   for cnt in contours:
      area = cv2.contourArea(cnt)
      rect = cv2.minAreaRect(cnt)
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
   printCowSquares(imgOriginal,255,255,255,cowSqrs)
   
   # AT THE END, RETURN THE LIST OF SQUARES THAT BELONG TO THE COW                                     
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

# This function fits 
def ajusteDeCurvas(bodyCoords):
   # bodyCoords is a multidimensional list: [[x1,y1],[x2,y2],...,[xN,yN]]
   # These lists and variables are used to calculate A and B
   # to get a polynomial of 1st grade: y = Ax + B
   theta = 0
   A = 0
   B = 0
   n = len(bodyCoords) # N is the lenght of the list body coords
   if n > 0:
      Xi = [] # [x1,x2,...,xN]
      Yi = [] # [y1,y2,...,yN]
      XiYi = [] #[x1*y1,x2*y2,...,xN*yN]
      Xi2 = [] #[x1^2, x2^2,...,xN^2]

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
def findClusters(cowRectangles,iterations,coordClusters):
   # First, the list clusters is initialized with n number of Clusters
   clusters = []
   for iA in range(len(coordClusters)):
      cluster = Cluster(coordClusters[iA])
      clusters.append(cluster)
    
   for dummy_iterator in range(iterations):
      for rect in cowRectangles:
         dist = float('inf')
         index = 0
         for i in range(len(clusters)):
            #actClust = clusters[i]
            if(clusters[i].distance(rect) < dist):
               dist = clusters[i].distance(rect)
               index = i
         clusters[index].add_point(rect)
      for cluster in clusters:
         #cluster.update_points()
         cluster.calculate_center()
         cluster.update_points()
   
   return clusters

def takePicture():
   cap = cv2.VideoCapture(0)
   for i in range(4):
         cap.grab()
   goodFrm, img = cap.read()
   return goodFrm, img






