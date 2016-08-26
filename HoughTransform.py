import cv2
import numpy as np

tVal = 80

img = cv2.imread('image40.jpg')
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
_, thres1 = cv2.threshold(gray,tVal,255,cv2.THRESH_BINARY_INV)
edges = cv2.Canny(thres1,200,200,apertureSize = 3)
cv2.imshow('c',edges)
_, contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
img = cv2.drawContours(img, contours, -1, (0,255,0), 3)
for cnt in contours:
	area = cv2.contourArea(cnt)
	rect = cv2.minAreaRect(cnt)
	w = int(rect[1][0])
	h = int(rect[1][1])
	rect_area = w * h
	if(rect_area > 0): # sometimes this value is found
		extent = float(area / rect_area)
	 	if (extent >= 0.7 and area > 40):   # tolerance
			x,y,w,h = cv2.boundingRect(cnt)
			cv2.circle(img,(x,y), 3, (255,0,0), -1)
cv2.imshow('i',img)
cv2.waitKey(0)
cv2.destroyAllWindows()
