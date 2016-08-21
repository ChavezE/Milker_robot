import cv2
import numpy as np
import time
import math
import LARC1 as rb
from random import randint

# ------ GLOBAL VARIABLES ---------
filename = 'image30.jpg'
binValue = 65 # parameter for the threshold


# loads image as imgOriginal
# This will have to change to taking snap everytime needed
# in order to do this we must initialice cv2.videoCapture in the future
# and remove parameter filename
def loadImage(capName):
   
   imgOriginal = cv2.imread(capName)
   imgOriginal = cv2.resize(imgOriginal, (720 ,480))
   return imgOriginal

#########   This will change when we use videoCapute #######
imgOriginal = loadImage(filename)

def getMyLegs(cowSquares, xTol, minSqrs):
   allLegs = []
   for i in range(len(cowSquares)):
      # x,y coord of individual sqr
      xi = cowSquares[i][4]
      yi = cowSquares[i][5]
      areai = cowSquares[i][0]
      tempSquares = []  # List to store neighbours from 'i'
      tempSquares.append([xi,yi,areai])   # adding first square
      
      # comparing with rest of Sqrs
      for j in range(len(cowSquares)):
         if(i != j):    # Check that it is not comparing with itself
            xj = cowSquares[j][4]
            yj = cowSquares[j][5]
            areaj = cowSquares[j][0]
            # If both are close in 'x' plane insert to the temporal list
            if abs(xj - xi) < xTol: 
               tempSquares.append([xj,yj,areaj])    

      # sorting tempSquares to compare in allSquares
      tempSquares = sorted(tempSquares, key=lambda x: x[0],reverse=False)

      if len(tempSquares) >= minSqrs and not rb.existInAllLegs(allLegs,tempSquares):
         # size RECTIFICATOR comparation
         stList = []
         for x in range(len(tempSquares)):
            stList.append(tempSquares[x][2])
         stDeviation = np.std(stList)
         #print "Std Deviation: ", stDeviation
         # ADJUST THIS IF EXPERIMENTALLY TO WORK EFFICIENT
         if(stDeviation < 500):
            allLegs.append(tempSquares)
   tempAllLegs = []
   for leg in allLegs:
      commonSqrLegs = [] 
      commonSqrLegs.append(leg)
      for legCompare in allLegs:
         if allLegs.index(leg) != allLegs.index(legCompare) and haveCommonSquares(leg,legCompare):
            commonSqrLegs.append(legCompare)
      
      lT = minStdDev(commonSqrLegs)
      if not rb.existInAllLegs(tempAllLegs,lT):
         tempAllLegs.append(lT)


   allLegs = tempAllLegs
   for x in range (len(allLegs)):
      allLegs[x] =  sorted(allLegs[x], key=lambda x: x[1],reverse=False)

   return allLegs

def haveCommonSquares(list1,list2):
   for squareAct in list1:
      if list2.count(squareAct) != 0:
         return True
   return False
# Recibes a list of lists with (x,y,area) = atom
# return list with less Standar Deviation in areas
def minStdDev(allLegs):
   minDev = -1
   stdDev = 100000
   for i in range(len(allLegs)):
      stList = []
      for x in range(len(allLegs[i])):
         stList.append(allLegs[i][x][2])
      tempDev = np.std(stList)
      if tempDev < stdDev:
         stdDev = tempDev
         minDev = i
   
   return allLegs[minDev]

def compareStdDeviation(list1,list2):
   std1 = 0
   std2 = 0
   for sqr in list1:
      std1 += sqr[2]
   std1 = np.std(std1)

   for sqr in list2:
      std2 += sqr[2]
   std2 = np.std(std2)

   if std1 < std2:
      return list1
   else: 
      return list2

# MAIN LOOP FUNCTION
def main():
   # NOTE: rb stands dor RoBorregos and is used to when implementing
   # a method that is in the module called LARC1.py

   filteredImage = rb.clearImage(imgOriginal)
   thresImage = rb.doThresHold(filteredImage,binValue)
   cv2.imshow('T',thresImage)

   # Getting contours here (and other thing we're not using)
   contours = rb.findContours(thresImage)
   # allRect is a list that contains posible squares ([area,extent,w,h,x,y])
   allRect = rb.getGoodSquares(contours,imgOriginal)
   # Next function is to find all the Legs
   allLegs = getMyLegs(allRect,20,3)
  
   # Print allLegs with different colors
   print len(allLegs)
   for i in range(len(allLegs)):
      B = randint(0,255)
      G = randint(0,255)
      R = randint(0,255)
      for sqr in allLegs[i]:
         xA = sqr[0]
         yA = sqr[1]
         cv2.circle(imgOriginal,(xA,yA),5,(B,G,R),-1)
   

   """
   # This line defines the font
   font = cv2.FONT_HERSHEY_SIMPLEX  
   This piece of code is to print the list allRect
   for cnt in range(0,len(allRect)):
      x = boundingBoxes[cnt][0]
      y = boundingBoxes[cnt][1]
      im = cv2.drawContours(imgOriginal,allRect,cnt,(0,255,0),3)
      im = cv2.circle(imgOriginal,(x,y), 3, (255,0,0), -1)
      text = str(cnt)
      cv2.putText(imgOriginal,text,(x,y), font, 1,(0,0,255),1,cv2.LINE_AA)
   """   
   cv2.imshow('im',imgOriginal)
   cv2.waitKey(0)


# ---------------------------------

# -----NOW, MAKE THE MAGIC HAPPEN-----

main()
cv2.destroyAllWindows()

