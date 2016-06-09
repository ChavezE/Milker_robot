import cv2
import numpy as np

frame = cv2.imread('image.jpg')
frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
_, thres = cv2.threshold(frame_gray, 70, 255, cv2.THRESH_BINARY_INV)

#Options for Smoothing

#smooth = cv2.medianBlur(thres, 5)
#smooth = cv2.bilateralFilter(thres, 9, 75,75)
smooth = cv2.GaussianBlur(thres, (3,3), 0)

# Morphological Transformations
kernel = np.ones((5,5), np.uint8)
morpho = cv2.erode(smooth, kernel, iterations = 1)

# Finding contours (B,G,R)

image, contours, hierarchy = cv2.findContours(morpho, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
frame = cv2.drawContours(frame, contours, -1, (0,0,255),1)

for i in range(0,len(contours)):
   cnt = contours[i]
   area = cv2.contourArea(cnt)
   x,y,w,h = cv2.boundingRect(cnt)
   rect_area = w*h
   extent = float(area)/rect_area
   if(extent >= 0.8):
# To know the center of the object
      M = cv2.moments(cnt)
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      print "Center %d : %d, %d   LeftTopCorner: %d, %d" % (i,cx,cy,x,y)
      frame = cv2.drawContours(frame, contours, i, (0,255,0),1)


# Showing results
cv2.imshow('T', thres)
cv2.imshow('S', smooth)
cv2.imshow('M', morpho)
cv2.imshow('F', frame)

cv2.waitKey(0)
cv2.destroyAllWindows()
