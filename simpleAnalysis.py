import cv2
import numpy as np
import time

# ------ GLOBAL VARIABLES ---------
fileName = 'image.jpg'  # In case we are analyzing a single image, the name is stored here
bestThres = 0  # Stores the best threshold value for binarization on the image
bestLen = 0    # Stores the maximum amount of squares found in the image
actThresValue = 0    # Stores the threshold value which has the greatest amount of squares
# ---------------------------------

# ------- DEFINING FUNCTIONS ------
def loadingImage(picName): # Loads the image or video frame and changes it to gray colorspace
   img = cv2.imread(picName)
   return img

def clearImage(img, thresVal):
   imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
   _, thres = cv2.threshold(imgGray, thresVal, 255, cv2.THRESH_BINARY_INV)
   smooth = cv2.GaussianBlur(thres, (3, 3), 0)
   kernel = np.ones((3,3), np.uint8) # The declaration of this kernel is for the morphological transformation
   morpho = cv2.erode(smooth, kernel, iterations = 2)
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
   for i in range(0, len(contours)):
      cnt = contours[i]
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

   #def detectHarCorners():
      
# ---------------------------------

# -----NOW, MAKE THE MAGIC HAPPEN-----

# Load the image...
imgO = loadingImage(fileName)

"""
Suponiendo que tenemos la coordenada de la esquina superior izquierda de cada cuadro de la vaca
lo que tenemos que hacer es algo como esto:

x0 y y0 son las coordenadas de la esq superior izquierda del cuadro para el que vamos a buscar

en un ciclo de todos los contours vas buscando las coordenadas de la esq superior izq de cada uno de los cuadros
que vamos a llamarles xT y yT

Tenemos un epsilon y decimos...

if abs((x0 - 2*w) - xT) < epsilon and abs(y0 - yT) < epsilon:
   # (x0,y0) tiene un vecino a la IZQUIERDA en (xT,yT)
if abs((x0 + 2*w) - xT) < epsilon and abs(y0 - yT) < epsilon:
   # (x0,y0) tiene un vecino a la DERECHA en (xT,yT)

Ya no lo escribi bien porque creo que las coordenadas que sacas cuando haces esto...
x = rect[0][0]
y = rect[0][1]
son del centro del circulo y no de la esquina superior izquierda. Tambien podria jalar pero si puedes dale tu

"""

"""

# Analyze for a single threshold value
imgO = loadingImage(fileName)
img = clearImage(imgO, 40)
_, contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, 
      cv2.CHAIN_APPROX_SIMPLE)
cowSquares,cowContours = findCowSquares(0.7,contours)
img = np.float32(img)
dst = cv2.cornerHarris(imgm4,5,0.04)
img[dst > 0.01*dst.max()] = [255,0,0]
cv2.drawContours(imgO, cowContours, -1, (0,255,0), 1)
cv2.imshow('o', imgO)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Iterate over the image to see the serch for the best threshold value
for k in range(0,80):
   img = clearImage(imgO,k)
   image, contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, 
      cv2.CHAIN_APPROX_SIMPLE)
   cowSquares, cowContours = findCowSquares(0.7,contours)
   


# 'average Extent' stores the average extent in pixels from an image's squares
# 'averageArea' stores the average area in pixels from an image's squares
   averageExtent, averageArea1, averageArea2 = calculateAverages(cowSquares) 
   if (len(cowSquares) >= 22):
      print ("%s Cow Sqrs: %s extent: %s area: %s, %s" % (k, len(cowSquares), averageExtent
                                                      ,averageArea1, averageArea2))
#Adaptative threshold analysis
img = clearAdaptativeImage(imgO)
_, contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, 
      cv2.CHAIN_APPROX_SIMPLE)
cv2.drawContours(imgO, contours, -1, (0,255,0), 1)

# Emilio's


def loop():
   binValue = 65 # parameter for the threshold
   ##########################################################################################
   imGray = cv2.cvtColor(imgO, cv2.COLOR_BGR2GRAY)
   imGray = cv2.GaussianBlur(imGray, (3,3), 2)
   _, thres1 = cv2.threshold(imGray,binValue,255,cv2.THRESH_BINARY_INV)
   thres1 = cv2.erode(thres1,np.ones((5,5),np.uint8))
   cv2.imshow('t',thres1)
   image, contours, hierarchy = cv2.findContours(thres1,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
   ##########################################################################################
   for cnt in contours:
      area = cv2.contourArea(cnt)
      rect = cv2.minAreaRect(cnt)
      x = rect[0][0]
      y = rect[0][1]
      w = rect[1][0]
      h = rect[1][1]
      angle = rect [2]
      box = cv2.boxPoints(rect)
      box = np.int0(box)
      #print x,y,w,h,angle
      rect_area = w*h
      if(rect_area>0):
         extent = float(area/rect_area)
         if (extent >= 0.7):   # tolerance
            im = cv2.drawContours(imgOriginal,[box],0,(0,0,255),3)
         #else:
         #  im = cv2.drawContours(imgOriginal,[box],0,(0,255,0),2)
      for cnt in range (0,len(contours)):
         area = cv2.contourArea(contours[cnt])
         x,y,w,h = cv2.boundingRect(contours[cnt])
         rect_area = w*h
         if(rect_area>0):
            extent = float(area/rect_area)
            if (extent >= 0.7):   # tolerance
               img = cv2.drawContours(imgOriginal, contours, cnt, (0,255,0), 2)
            #else:
            #  im = cv2.drawContours(imgOriginal,[box],0,(0,255,0),2)
   cv2.imshow('im',imgOriginal)
   cv2.waitKey(0)
loop()
cv2.destroyAllWindows()

"""


# Print some frames to check results ...
#cv2.imshow('o', imgO)
#cv2.imshow('t', img)


# -----------------------------------


"""
while (actThresValue <= 150):
   _, thres = cv2.threshold(frame_gray, actThresValue, 255, cv2.THRESH_BINARY_INV)
   smooth = cv2.GaussianBlur(thres, (3,3), 0)
   kernel = np.ones((3,3), np.uint8)
   morpho = cv2.erode(smooth, kernel, iterations = 2)

   # Finding contours (B,G,R)

   image, contours, hierarchy = cv2.findContours(morpho, cv2.RETR_TREE, 
   cv2.CHAIN_APPROX_SIMPLE)
   cowSquares = []   # This list is to store only the squares that belog to the cow

   for i in range(0,len(contours)):
      cnt = contours[i]
      area = cv2.contourArea(cnt)
      x,y,w,h = cv2.boundingRect(cnt)
      rect_area = w*h
      extent = float(area)/rect_area
      if(extent >= 0.7):
   # To know the center of the object
         M = cv2.moments(cnt)
         cx = int(M['m10']/M['m00'])
         cy = int(M['m01']/M['m00'])
         #print "Center %d : %d, %d   LeftTopCorner: %d, %d" % (i,cx,cy,x,y)
         cowSquares.insert(0,[x,y,w,h])
         #frame = cv2.drawContours(frame, contours, i, (0,255,0),1)
   actualLen = len(cowSquares)
   if (bestLen <= actualLen):
      bestThres = actThresValue
      bestLen = actualLen
   
   actThresValue = actThresValue + 1

"""
