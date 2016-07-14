import cv2
import numpy as np
import time

# ------ GLOBAL VARIABLES ---------
fileName = 'image2.jpg'  # In case we are analyzing a single image, the name is stored here
bestThres = 0  # Stores the best threshold value for binarization on the image
bestLen = 0    # Stores the maximum amount of squares found in the image
actThresValue = 0    # Stores the threshold value which has the greatest amount of squares
# ---------------------------------

# ------- DEFINING FUNCTIONS ------
def loadingImage(picName): # Loads the image or video frame and changes it to gray colorspace
   img = cv2.imread(picName)
   return img

def clearImage(img, thresVal):
   img = cv2.resize(img, (720 ,480))
   imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
   smooth = cv2.GaussianBlur(imgGray, (3, 3), 0)
   _, thres = cv2.threshold(smooth, thresVal, 255, cv2.THRESH_BINARY_INV)
   #cv2.imshow('thres',thres)
   kernel = np.ones((3,3), np.uint8) # The declaration of this kernel is for the morphological transformation
   morpho = cv2.erode(thres, kernel, iterations = 2)
   return morpho
   
def clearAdaptativeImage(img):
   imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
   imgGray = cv2.medianBlur(imgGray,5)
   th = cv2.adaptiveThreshold(imgGray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
            cv2.THRESH_BINARY,11,2)
   return th

def centerOfContour(contour): # Returns the coordinates of the contour's center
   M = cv2.moments(contour)
   cx = int(M['m10']/M['m00'])
   cy = int(M['m01']/M['m00'])
   return cx, cy

def printContourCoords(cx,cy,x,y):
   print "Center: %d, %d   LeftTopCorner: %d, %d" % (cx,cy,x,y)

def findCowSquares(limExtent,contours = []):
   cowSquares = []   # This list is to store only the squares that belog to the cow
   cowContours = [] # This list is to to store the contours that belong to the cow
   for cnt in contours:
      area = cv2.contourArea(cnt)
      x,y,w,h = cv2.boundingRect(cnt)
      rect_area = w*h
      extent = float(area)/rect_area
      
      if(extent >= limExtent):
         cowSquares.insert(0,[x,y,w,h,extent,area])
         cowContours.insert(0,cnt)
   return cowSquares,cowContours

def calculateAverages(cowSquares = []):
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

def sortList(index,l =[]): # This functions sorts a multivaraible list depending on the index specified
   l = sorted(l, key=lambda x: x[index],reverse=False) 
   return l

def findMedian(index,l=[]):
   if (len(l) % 2) != 0:     # The length of the list is even
      return l[len(l)/2][index]
   else:              # The lenght of the list is odd
      return (l[len(l)/2][index] + l[len(l)/2 + 1][index])/2

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

        

        tempAllR = []
   for cnt in allR:
      areaTemp = cv2.contourArea(cnt)
      if not((areaTemp > medianArea * 3) or (areaTemp < medianArea * 0.25)):
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
################################################
############### MAIN LOOP ######################
################################################
def loop():

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
      text = str(cnt)
      cv2.putText(imgOriginal,text,(x,y), font, 1,(0,0,255),1,cv2.LINE_AA)
   """   
   cv2.imshow('im',imgOriginal)
   cv2.waitKey(0)


# ---------------------------------

# -----NOW, MAKE THE MAGIC HAPPEN-----

# Load the image...
imgO = loadingImage(fileName)
# Clean the image ...
imgC = clearImage(imgO, 65)
# Find contours ...
contours = findContours(imgC)

loop()
cv2.destroyAllWindows()

