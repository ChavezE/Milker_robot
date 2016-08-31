import cv2
import numpy as np
import time
import math
import LARC1 as rb
from random import randint
import ajusteDeCurvas as aj

# ------ GLOBAL VARIABLES ---------
filename = 'image7.jpg'
binValue = 90 # parameter for the threshold


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
      tempSquares = rb.sortList(2,tempSquares)
      tempSquares = rb.sortList(0,tempSquares)

      if len(tempSquares) >= minSqrs and not rb.existInAllLegs(allLegs,tempSquares):
         # size RECTIFICATOR comparation
         stList = []
         for x in range(len(tempSquares)):
            stList.append(tempSquares[x][2])
         stDeviation = np.std(stList)
         #print "Std Deviation: ", stDeviation
         # ADJUST THIS IF EXPERIMENTALLY TO WORK EFFICIENT
         if(stDeviation < 200):
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

   return tempAllLegs

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
def findMiddleOfAllLegs(allLegs):
   midOfLegs = []
   font = cv2.FONT_HERSHEY_SIMPLEX
   for leg in allLegs:
      xA = 0
      yA = 0
      for sqr in leg:
         xA += sqr[0]
         yA += sqr[1]
      xA /= len(leg)
      yA /= len(leg)
      # B = randint(0,255)
      # G = randint(0,255)
      # R = randint(0,255)
      # cv2.circle(imgOriginal,(xA,yA),4,(B,G,R),-1)
      # cv2.putText(imgOriginal,(str(xA)+" "+str(yA)),(xA,yA), font, 0.4,(B,G,R),1,cv2.LINE_AA)
      midOfLegs.append([xA,yA])

   
   tempMOfLegs = []
   
   midOfLegs = rb.sortList(0,midOfLegs)

   for i in range(len(midOfLegs)):
      bF = False
      for j in range(len(midOfLegs)):
         if i != j and abs(midOfLegs[i][0] - midOfLegs[j][0]) < 35:
            bF = True
            xA = (midOfLegs[i][0] + midOfLegs[j][0])/2
            yA = (midOfLegs[i][1] + midOfLegs[j][1])/2
            newCord = [xA,yA]
            if not rb.existInAllLegs(tempMOfLegs,newCord):
               tempMOfLegs.append(newCord)
      if not bF:
         tempMOfLegs.append(midOfLegs[i])
   midOfLegs = tempMOfLegs 
   
   return midOfLegs

# MAIN LOOP FUNCTION
def main():
   # NOTE: rb stands dor RoBorregos and is used to when implementing
   # a method that is in the module called LARC1.py
   font = cv2.FONT_HERSHEY_SIMPLEX
   filteredImage = rb.clearImage(imgOriginal)
   #cv2.imshow('G',filteredImage)
   thresImage = rb.doThresHold(filteredImage,binValue)
   #cv2.imshow('T',thresImage)
   '''
   # Draw center of the image
   cv2.circle(imgOriginal,(360,240),5,(255,0,0),-1)
   # Getting contours here (and other thing we're not using)
   contours = rb.findContours(thresImage)
   # allRect is a list that contains posible squares ([area,extent,w,h,x,y])
   allRect = rb.getGoodSquares(contours,imgOriginal)
   #allRect = mergeSimilarLegs(allRect)
   B = randint(0,255)
   G = randint(0,255)
   R = randint(0,255)
   #for rect in allRect:
      #cv2.circle(imgOriginal,(rect[4],rect[5]),6,(B,G,R),-1)

   # Next function is to find all the Legs
   allLegs = getMyLegs(allRect,10,3)
   # Next function finds the middle point of each leg
   midOfLegs = findMiddleOfAllLegs(allLegs)
   for m in midOfLegs:
      cv2.putText(imgOriginal,(str(m[0])+", "+str(m[1])),(m[0],m[1]), font, 0.4,(B,G,R),1,cv2.LINE_AA)
   # Find a the mean (x,y) of midOfLegs
   xAct = 0
   yAct = 0
   for mid in midOfLegs:
      xAct += mid[0]
      yAct += mid[1]

   xAct /= len(midOfLegs)
   yAct /= len(midOfLegs)
   B = randint(0,255)
   G = randint(0,255)
   R = randint(0,255)
   cv2.circle(imgOriginal,(xAct,yAct),8,(B,G,R),-1)
   cv2.putText(imgOriginal,(str(xAct)+", "+str(yAct)),(xAct,yAct), font, 0.4,(B,G,R),1,cv2.LINE_AA)
   # Print allLegs with different colors

   print "Num Legs: ", len(allLegs)
   for l in allLegs:
      print l

   for i in range(len(allLegs)):
      B = randint(0,255)
      G = randint(0,255)
      R = randint(0,255)
      for sqr in allLegs[i]:
         xA = sqr[0]
         yA = sqr[1]
         cv2.circle(imgOriginal,(xA,yA),3,(B,G,R),-1)
   

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
   '''
   img2 = aj.histogramEqualization(imgOriginal)
   cv2.imshow('E',img2)
   img2 = rb.clearImage(img2)
   cv2.imshow('G',img2)
   img2 = rb.doThresHold(img2,50)
   cv2.imshow('T',img2)
   
   cv2.waitKey(0)


# ---------------------------------

# -----NOW, MAKE THE MAGIC HAPPEN-----

main()
cv2.destroyAllWindows()

