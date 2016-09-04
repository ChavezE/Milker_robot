# ------ LIBRARIES ----------
import cv2
import time
import math
import numpy as np
import random
import LARC1 as rb
# ---------------------------

filename = 'image35.jpg'
cap = cv2.VideoCapture(0)


# loads image as imgOriginal
# This will have to change to taking snap everytime needed
# in order to do this we must initialice cv2.videoCapture in the future
# and remove parameter filename
def loadImage(capName):
    
    imgOriginal = cv2.imread(capName)
    return imgOriginal

#imgOriginal = loadImage(filename)

def nothing(x):
    pass

cv2.namedWindow('image')
# create trackbars for color change
cv2.createTrackbar('Thres','image',0,255,nothing)
# create switch for ON/OFF functionality
switch = '0 : OFF \n1 : ON'
cv2.createTrackbar(switch, 'image',0,1,nothing)

def goLive():
    while (1):
    
        _, imgOriginal = cap.read()
        cv2.imshow('imgOriginal',imgOriginal)
        filteredImage = rb.clearImage(imgOriginal)

        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break

        # get current positions of four trackbars
        binValue = cv2.getTrackbarPos('Thres','image')
        s = cv2.getTrackbarPos(switch,'image')


        if s == 0:
            pass 
        else:
            thresImage = rb.doThresHold(filteredImage,binValue)
            cv2.imshow('img', thresImage)
        
goLive()
cv2.destroyAllWindows()