import time
import cv2
import numpy as np

# emibas.py
# Here we go...


# Video capture
cap = cv2.VideoCapture(0)
def goLive():
	while (1):
		_, imgOriginal = cap.read()
		cv2.imshow('im',imgOriginal)
		c = cv2.waitKey(50)
		if c == 27:
			break
	
goLive()
cv2.destroyAllWindows()
