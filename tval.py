import cv2
import numpy as np
import LARC1 as rb

filename = 'image35.jpg'
binValue = 75  # parameter for the threshold


# loads image as imgOriginal
# This will have to change to taking snap everytime needed
# in order to do this we must initialice cv2.videoCapture in the future
# and remove parameter filename
def loadImage(capName):
    
    imgOriginal = cv2.imread(capName)
    imgOriginal = cv2.resize(imgOriginal, (720 ,480))
    return imgOriginal

imgOriginal = loadImage(filename)

def nothing(x):
    pass

cv2.namedWindow('image')
# create trackbars for color change
cv2.createTrackbar('Thres','image',0,255,nothing)


# create switch for ON/OFF functionality
switch = '0 : OFF \n1 : ON'
cv2.createTrackbar(switch, 'image',0,1,nothing)

while(1):
    
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
        

cv2.destroyAllWindows()