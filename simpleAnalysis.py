import cv2
import numpy as np
import time
import math
import LARC1 as rb

# ------ GLOBAL VARIABLES ---------
filename = 'image3.jpg'
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

# This method will recibe all countours in image and
# will make all necesary filtering in order to achive
# countors with high probability of being cow rectangles
def getGoodSquares(contours):

   # ----VARIABLES----
   #allR = [] # List that stores the contours. This will be returned
   #tempAllR = []
   cowSqrs = [] # This list is useful to sort the list allR
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
      rect_area = w*h
      if(rect_area>0): # sometimes this value is found
         extent = float(area/rect_area)
         if (extent >= 0.7):   # tolerance
            x,y,w,h = cv2.boundingRect(cnt)
            # allR.insert(0,cnt)
            cowSqrs.append([area,extent,w,h,x,y])

   #print "Len Conts: ", len(allR)
   # Print contours BEFORE the filters in green color
   #cv2.drawContours(imgOriginal, allR, -1, (0,255,0), 2) 
   

   # We will continue analysing the contours ONLY if we have at least 15
   if len(cowSqrs) < 15:
      print("Not enough squares")
      return allR

   # 2nd - FILTER Contours FOR ITS AREA
   # This function sorts both lists "allR" and "cowSqrs" in ascending order of its areas
   # (allR, cowSqrs) = zip(*sorted(zip(allR, cowSqrs), key=lambda b:b[1][0], reverse=False))
   cowSqrs = sorted(cowSqrs, key=lambda x: x[0],reverse=False)

   # Print elements in cowSqrs
   for sqr in cowSqrs:
      w = sqr[2]
      h = sqr[3]
      x = sqr[4]
      y = sqr[5]
      cv2.rectangle(imgOriginal,(x,y),(x+w,y+h),(255,0,0),2)
   
   """
   # HERE WE CALCULATE THE MEDIAN VALUE OF AREA FOR ALL THE CONTOURS IN allR
   medianArea = 0
   # The length of the list is odd
   if (len(cowSqrs) % 2) != 0:     
      medianArea = cowSqrs[len(cowSqrs)/2][0]
   # The lenght of the list is even
   else:
      medianArea = (cowSqrs[len(cowSqrs)/2][0] + cowSqrs[len(cowSqrs)/2 + 1][0])/2
   print "Median Area: ", (medianArea)
   
   # THIS CYCLE remove the contours that are 3 times bigger or less than 0.25 of the median area
   for cnt in allR:
      areaTemp = cv2.contourArea(cnt)
      if not((areaTemp > medianArea * 3) or (areaTemp < medianArea * 0.25)):
         tempAllR.insert(0, cnt)    
   cv2.drawContours(imgOriginal, tempAllR, -1, (255,255,0), 3)
   """

   # FILTER CONTOURS CREATING A AVERAGE THAT DISCARDS THE 5 BIGGEST CONTS 
   # AND 5 SMALLEST
   # **** This Filter works better than the median filter ******
   avgArea = 0
   for i in range(5, len(cowSqrs) - 6):
      avgArea += cowSqrs[i][0]
   avgArea /= (len(cowSqrs) - 10)
   #print "Avg Area", (avgArea)
   
   # THIS CYCLE remove the contours that are 3 times bigger or less 
   # than 0.1 of the median area
   for ind in range(len(cowSqrs)):
      areaTemp = cowSqrs[ind][0]
      if not((areaTemp > avgArea * 3) or (areaTemp < avgArea * 0.1)):
         #tempAllR.append(cnt)  
         tempCowSqrs.append(cowSqrs[ind])  
   #allR = tempAllR
   cowSqrs = tempCowSqrs
   #print "Len Conts: ", len(allR)
   #cv2.drawContours(imgOriginal, allR, -1, (255,0,0), 2)
   
   # FILTER squares for its POSITION on the image
   cowSqrs = sorted(cowSqrs, key=lambda x: x[5],reverse=False)
   # Clean temp variables
   tempCowSqrs = []
   neighX = [] # Here we will store the number of neighbours in X
   neighY = [] # Here we will store the number of neighbours in Y
   neighbours = []
   closeNeigh = []

   # The lists are initialiced with 0's
   for i in range(len(cowSqrs)):
      neighX.append(0) 
      neighY.append(0)
      closeNeigh.append(0)
      
   # Each contours is compared with the remaining next squares in the list
   for actSqr in cowSqrs:
      # The next 3 variables are from the current square
      actIndex = cowSqrs.index(actSqr)
      actY = actSqr[5]
      actX = actSqr[4]
      r = 70
      for i in range(1,len(cowSqrs)):

         compareIndex = actIndex + i
         if(compareIndex < len(cowSqrs)):
            compareY = cowSqrs[compareIndex][5]
            compareX = cowSqrs[compareIndex][4]
            d = math.sqrt(pow(compareX - actX,2) + pow(compareY - actY,2))
            if(abs(actY - compareY) <= 15):
               neighY[actIndex] += 1
               neighY[compareIndex] += 1
            if(abs(actX - compareX) <= 10):
               neighX[actIndex] += 1
               neighX[compareIndex] += 1
            if(d <= r):
               closeNeigh[actIndex] += 1
               closeNeigh[compareIndex] += 1
         else:
            break
   # Now we will remove the squares that have less than 4 neighbours
   for actSqr in cowSqrs:
      i = cowSqrs.index(actSqr)
      s = neighX[i] + neighY[i]
      if(s >= 4 and closeNeigh[i] >= 2):
         tempCowSqrs.append(actSqr)
         neighbours.append(s)
   cowSqrs = tempCowSqrs

   font = cv2.FONT_HERSHEY_SIMPLEX # This line defines the font
   for sqr in cowSqrs:
      actIndex = cowSqrs.index(sqr)
      x = sqr[4]
      y = sqr[5]
      text = str(neighbours[actIndex])
      cv2.putText(imgOriginal,text,(x,y), font, 0.4,(0,0,255),1,cv2.LINE_AA)

   # Print elements in cowSqrs
   for sqr in cowSqrs:
      w = sqr[2]
      h = sqr[3]
      x = sqr[4]
      y = sqr[5]
      cv2.rectangle(imgOriginal,(x,y),(x+w,y+h),(0,255,0),3)
   
   # AT THE END, RETURN THE CONTOURS THAT BELONG TO THE COW                                     
   return cowSqrs

# This method will sort countours respect to Y or X coord of bounding Rectangle
# Critieria defines whether X o Y is used DEFAULT IS y
# This is always from lef to right of from top to bottom --> Reverse = False
# Most of this is from here : http://www.pyimagesearch.com/2015/04/20/sorting-contours-using-python-and-opencv/
def boundingRectSort(allRect,criteria):
   i = 1
   if criteria == 'X' or criteria ==  'x':
      i = 0
   boundingBoxes = [cv2.boundingRect(c) for c in allRect]
   (allRect, boundingBoxes) = zip(*sorted(zip(allRect, boundingBoxes),
      key=lambda b:b[1][i], reverse=False))
 
   return (allRect, boundingBoxes)


# MAIN LOOP FUNCTION
def main():

   filteredImage = clearImage(imgOriginal)
   thresImage = doThresHold(filteredImage,binValue)
   cv2.imshow('T',thresImage)

   # Getting contours here (and other thing we're not using)
   image, contours, hierarchy = cv2.findContours(thresImage,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

   allRect = getGoodSquares(contours)
   #boundingBoxes = [] # will be used in parallel with allRect
   #allRect, boundingBoxes = boundingRectSort(allRect,'y')
   
   # At this point we have all posible squares filtered and stored  AS COUNTOURS in allRect

   font = cv2.FONT_HERSHEY_SIMPLEX # This line defines the font 
   #cv2.drawContours(imgOriginal,allRect,-1,(0,255,0))
   """
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

