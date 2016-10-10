## This is version 1.0 of LARC 2016 Module of Mexican Team RoBorregos
## which stores functions used by the computer vision squad. 
## All rights reserved by Instituo Tecnologico de Monterrey.

## AUTHORS:    Emilio Chavez Madero
##             Sebastian Rivera Gonzalez
##             Diego Garza Rodriguez

## CREATION DATE: October 2016

##-----------LIBRARIES-----------
import cv2
import time
import math
import numpy as np
import LARC1 as rb
import random
import serial
import statistics
from copy import deepcopy
##-------------------------------

##-----------CLASSES-----------
class cowSquare:
   ##-----------ATRIBUTES-----------##
   x = y = w = h = area = level = 0
   topLeftC = topRightC = botLeftC = botRightC = []
   ##-------------------------------##

   ##----------METHODS-----------##
   def __init__(self,xT,yT,wT,hT,areaT):
      self.x = xT
      self.y = yT
      self.w = wT
      self.h = hT
      self.area = areaT
      self.topLeftC = [xT,yT]
      self.topRightC = [xT+wT,yT]
      self.botLeftC = [xT,yT+hT]
      self.botRightC = [xT+wT,yT+hT]
   ##----------------------------##

   ##-----GETs-----##
   def getX(self):
      return self.x
   def getY(self):
      return self.y
   def getW(self):
      return self.w
   def getH(self):
      return self.h
   def getArea(self):
      return self.area
   def getTopLeftC(self):
      return self.topLeftC
   def getTopRightC(self):
      return self.topRightC
   def getBotLeftC(self):
      return self.botLeftC
   def getBotRightC(self):
      return self.botRightC
   def getLevel(self):
      return self.level
   ##--------------##

   ##-----SETs-----##
   def setX(self,xT):
      self.x = xT
   def setY(self,yT):
      self.y = yT
   def setW(self,wT):
      self.w = wT
   def setH(self,hT):
      self.h = hT
   def setArea(self,areaT):
      self.area = areaT
   def setLevel(self,lev):
      self.level = lev
   ##--------------##

   ##-----END of Class 'cowSquare'-----

class Cluster:
   ##-----------ATRIBUTES-----------##
   center = []
   new_points = []
   old_points = []
   ##-------------------------------##

   ##----------METHODS-----------##
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
  
   def set_center(self,point):
      self. center = point

   def distance(self,p):
      x = p[0]
      y = p[1]
      xDif = abs(x - self.center[0])
      yDif = abs(y - self.center[1])
      return math.sqrt((xDif*xDif) + (yDif*yDif))

   def update_points(self):
      self.old_points = list(self.new_points)
      self.new_points = []

   def get_center(self):
      return self.center

   def get_new_points(self):
      return list(self.new_points)

   def get_old_points(self):
      return list(self.old_points)
   ##----------------------------##
   ##-----END of Class 'Cluster'-----##
##-----------------------------

##-----------PRINCIPAL FUNCTIONS-----------
def clearImage(imgOriginal):
   # Does some guassian filtering to remove noise and converts image to gray scale
   # imgOriginal[480*0.65:480,0:640] = [255,255,255]
   imGray = cv2.cvtColor(imgOriginal, cv2.COLOR_BGR2GRAY)
   imGray = cv2.GaussianBlur(imGray, (3,3), 2)
   # imGray = cv2.fastNlMeansDenoisingColored(imgOriginal,None,10,10,7,21)

   return imGray

def doThresHold(filteredImage,tVal,k,i):
   _, thres1 = cv2.threshold(filteredImage,tVal,255,cv2.THRESH_BINARY_INV)
   thres1 = cv2.erode(thres1,np.ones((k,k),np.uint8), iterations=i)
   
   return thres1

def findContours(img):
   # Finds contours in the image and returns them as a numpy list. 
   # Recieves the image as a parameter.
   _, contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, 
      cv2.CHAIN_APPROX_SIMPLE)
   return contours

def getGoodSquares(contours,imgOriginal,thres):
   # Processes the contours found in order to find squares in the image

   # ----VARIABLES----
   cowSquares = []   # This list is the one that is going to being returned
   # -----------------

   for cnt in contours:
      area = cv2.contourArea(cnt)
      rect = cv2.minAreaRect(cnt)
      w = int(rect[1][0])
      h = int(rect[1][1])
      rect_area = w * h
      if(rect_area > 0): # sometimes this value is found
         extent = float(area / rect_area)
         if (extent >= 0.75 and area >= 400 and area <= 12500):   # tolerance
            x,y,w,h = cv2.boundingRect(cnt)
            if thres[y + h*0.5,x + w*0.5] == 1 and w/h > 0.1:
               tempCowSquare = cowSquare(x,y,w,h,area)    # Create an objet from the 'cowSquare' class
               cowSquares.append(tempCowSquare) # Insert object 'cowSquare' into a list  

   # Print elements in cowSqrs
   # b = int ( random.uniform(50,255))
   # g = int ( random.uniform(50,255))
   # r = int ( random.uniform(50,255))
   # printCowSquares(imgOriginal,b,g,r,cowSquares)
                                       
   return cowSquares

def distance(x1,y1,x2,y2):
  return math.sqrt(pow(x2 - x1,2) + pow(y2 - y1,2))

def makeTissue(tCowSquares, tissue, epsilon, checkWith, corner, curLevel):

   if checkWith == 1:   # corner => topLeft
      found = False     # Flag to check if the tissue found a neighboor
      for i in range(len(tCowSquares)):    # Go over all the elements in 'tCowSquares' from 0 to len(cowSquares)-1
         cowSquare= tCowSquares[i]        # temporal square
         compareCoord = cowSquare.getBotRightC()   # use the corner of interest depending on the attribute 'checkWith'

         # Find the distance between the two corners in order to find adjacent ones
         if(distance(corner[0],corner[1],compareCoord[0],compareCoord[1]) < epsilon and compareCoord[1] - 30 <= corner[1]):
            cowSquare.setLevel(curLevel + 1)   # Set the level of the square
            tissue.append(cowSquare)  # Add 'cowSquare' to the list 'tissue'
            found = True
            tCowSquares.pop(i)
            break
      if found:
         tempElement = tissue[len(tissue)-1]
         makeTissue(tCowSquares, tissue, epsilon, 1, tempElement.getTopLeftC(), curLevel + 1)
         makeTissue(tCowSquares, tissue, epsilon, 2, tempElement.getTopRightC(), curLevel + 1)
         makeTissue(tCowSquares, tissue, epsilon, 3, tempElement.getBotLeftC(), curLevel + 1)

   elif checkWith == 2:   # corner => topRight
      found = False
      for i in range(len(tCowSquares)):
         cowSquare= tCowSquares[i]
         compareCoord = cowSquare.getBotLeftC()

         # Find the distance between the two corners in order to find adjacent ones
         if(distance(corner[0],corner[1],compareCoord[0],compareCoord[1]) < epsilon and compareCoord[1] - 30<= corner[1]):
            cowSquare.setLevel(curLevel + 1)   # Set the level of the square
            tissue.append(cowSquare)  # Add 'cowSquare' to the list 'tissue'
            found = True
            tCowSquares.pop(i)
            break
      if found:
         tempElement = tissue[len(tissue)-1]
         makeTissue(tCowSquares, tissue, epsilon, 1, tempElement.getTopLeftC(), curLevel + 1)
         makeTissue(tCowSquares, tissue, epsilon, 2, tempElement.getTopRightC(), curLevel + 1)
         makeTissue(tCowSquares, tissue, epsilon, 4, tempElement.getBotRightC(), curLevel + 1)

   elif checkWith == 3:   # corner => botLeft
      found = False
      for i in range(len(tCowSquares)):
         cowSquare= tCowSquares[i]
         compareCoord = cowSquare.getTopRightC()

         # Find the distance between the two corners in order to find adjacent ones
         if(distance(corner[0],corner[1],compareCoord[0],compareCoord[1]) < epsilon and compareCoord[1] + 30 >= corner[1]):
            cowSquare.setLevel(curLevel - 1)   # Set the level of the square
            tissue.append(cowSquare)  # Add 'cowSquare' to the list 'tissue'
            found = True
            tCowSquares.pop(i)
            break
      if found:
         tempElement = tissue[len(tissue)-1]
         makeTissue(tCowSquares, tissue, epsilon, 1, tempElement.getTopLeftC(), curLevel - 1)
         makeTissue(tCowSquares, tissue, epsilon, 3, tempElement.getBotLeftC(), curLevel - 1)
         makeTissue(tCowSquares, tissue, epsilon, 4, tempElement.getBotRightC(), curLevel - 1)

   elif checkWith == 4:   # corner => botRight
      found = False
      for i in range(len(tCowSquares)):
         cowSquare= tCowSquares[i]
         compareCoord = cowSquare.getTopLeftC()

         # Find the distance between the two corners in order to find adjacent ones
         if(distance(corner[0],corner[1],compareCoord[0],compareCoord[1]) < epsilon and compareCoord[1] + 30 >= corner[1]):
            cowSquare.setLevel(curLevel - 1)   # Set the level of the square
            tissue.append(cowSquare)  # Add 'cowSquare' to the list 'tissue'
            found = True
            tCowSquares.pop(i)
            break
      if found:
         tempElement = tissue[len(tissue)-1]
         makeTissue(tCowSquares, tissue, epsilon, 2, tempElement.getTopRightC(), curLevel - 1)
         makeTissue(tCowSquares, tissue, epsilon, 3, tempElement.getBotLeftC(), curLevel - 1)
         makeTissue(tCowSquares, tissue, epsilon, 4, tempElement.getBotRightC(), curLevel - 1)
   
   elif checkWith == 0:   # First call to the function
      greatestTissue = []
      tissue = []

      while len(tCowSquares) > 0:
         actSquare = tCowSquares.pop(0)
         actSquare.setLevel(0)
         tissue.append(actSquare)
         makeTissue(tCowSquares, tissue, epsilon, 1, actSquare.getTopLeftC(), 0)
         makeTissue(tCowSquares, tissue, epsilon, 2, actSquare.getTopRightC(), 0)
         makeTissue(tCowSquares, tissue, epsilon, 3, actSquare.getBotLeftC(), 0)
         makeTissue(tCowSquares, tissue, epsilon, 4, actSquare.getBotRightC(), 0)

         if len(tissue) > len(greatestTissue):
            greatestTissue = deepcopy(tissue)
         tissue[:] = []

      greatestTissue = sorted(greatestTissue, key=lambda x:x.getLevel(), reverse=True)
      # iA = 0
      # Remove those squares who are unique at a specific level
      # while iA < len(greatestTissue) and len(greatestTissue) > 1:
      #    if(iA == len(greatestTissue) - 1):
      #       if(greatestTissue[iA-1].getLevel() != greatestTissue[iA].getLevel()):
      #          greatestTissue.pop(iA)
      #       else:
      #          iA += 1
      #    elif(iA == 0):
      #       if(greatestTissue[iA+1].getLevel() != greatestTissue[iA].getLevel()):
      #          greatestTissue.pop(iA)
      #       else:
      #          iA += 1
      #    else:
      #       if(greatestTissue[iA-1].getLevel() != greatestTissue[iA].getLevel() and greatestTissue[iA+1].getLevel() != greatestTissue[iA].getLevel()):
      #          greatestTissue.pop(iA)
      #       else:
      #          iA += 1
         
      return greatestTissue

def ajusteDeCurvas(cSquares):
   # bodyCoords is a multidimensional list: [[x1,y1],[x2,y2],...,[xN,yN]]
   # These lists and variables are used to calculate A and B
   # to get a polynomial of 1st grade: y = Ax + B
   theta = 0
   A = 0
   B = 0
   n = len(cSquares)
   if n > 1:
      Xi = [] # [x1,x2,...,xN]
      Yi = [] # [y1,y2,...,yN]
      XiYi = [] #[x1*y1,x2*y2,...,xN*yN]
      Xi2 = [] #[x1^2, x2^2,...,xN^2]

      # This cycle is to fill up all the lists in order to compute A and B
      for c in cSquares:
         Xi.append(c.getX())
         Yi.append(c.getY())
         XiYi.append(c.getX() * c.getY())
         Xi2.append(pow(c.getX(),2))

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
##-----------------------------------------

##-----------SECONDARY FUNCTIONS-----------
def sortList(index,l): 
   # This functions sorts a multivaraible list depending on the index specified
   l = sorted(l, key=lambda x:x[index], reverse=False) 
   
   return l

def printCowSquares(imgOriginal,G,B,R,sqrs):
   for sqr in sqrs:
      x = sqr.getX()
      y = sqr.getY()
      w = sqr.getW()
      h = sqr.getH()
      cv2.rectangle(imgOriginal,(x,y),(x+w,y+h),(G,B,R),2)
   cv2.imshow('m',imgOriginal) 

def findClusters(cowRectangles,iterations,coordClusters):
   # This function is to implement the clustering algorithm 
   # PARAMETERS: number of clusters to search for, list of coordinates of the cow
   # and number of iterations before returning the final clusters
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

def neighboors(cowSquares):
   # This function returnsa list of coords with all those
   # that have more than 2 neihbors in a specific radius
   radius = 100
   neighboors = []
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
         neigh.append([xi,yi])
   return neigh
##-----------------------------------------

##-----------EXTRA FUNCTIONS-----------
def printContourCoords(cx,cy,x,y):
   print "Center: %d, %d   LeftTopCorner: %d, %d" % (cx,cy,x,y)

def findMedian(index,l):
   # The length of the list is odd
   if (len(l) % 2) != 0:     
      return l[len(l)/2][index]
   # The lenght of the list is even
   else:              
      return (l[len(l)/2][index] + l[len(l)/2 + 1][index])/2

def getBody(cowRectangles,imgOriginal,totLines,epsilon):
   # This function is still on progress, hope to return well 
   # ordered body of cow in a convenient way 
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

def boundingRectSort(allRect,criteria):
   # This method will sort countours respect to Y or X coord of bounding Rectangle
   # Critieria defines whether X o Y is used DEFAULT IS y
   # This is always from lef to right of from top to bottom --> Reverse = False
   # Most of this is from here : http://www.pyimagesearch.com/2015/04/20/sorting-contours-using-python-and-opencv/
   i = 1
   if criteria == 'X' or criteria ==  'x':
      i = 0
   boundingBoxes = [cv2.boundingRect(c) for c in allRect]
   (allRect, boundingBoxes) = zip(*sorted(zip(allRect, boundingBoxes),
      key=lambda b:b[1][i], reverse=False))
 
   return (allRect, boundingBoxes)
##-------------------------------------