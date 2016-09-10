# ------ LIBRARIES ----------
import cv2
import time
import math
import numpy as np
import random
import LARC1 as rb
import gtk
import subprocess
# ---------------------------



def goLive():
	cap = cv2.VideoCapture(0)
	while (1):
		_, imgOriginal = cap.read()
		cv2.imshow('im',imgOriginal)
		c = cv2.waitKey(50)
		if c == 27:
			break
	cv2.destroyAllWindows()

def nothing(x):
    pass



def goLiveT():
	cap = cv2.VideoCapture(0)
	cv2.namedWindow('image')
	# create trackbars for color change
	cv2.createTrackbar('Thres','image',0,255,nothing)
	# create switch for ON/OFF functionality
	switch = '0 : OFF \n1 : ON'
	cv2.createTrackbar(switch, 'image',0,1,nothing)
	while (1):

		_, imgOriginal = cap.read()
		cv2.imshow('imgOriginal',imgOriginal)
		filteredImage = rb.clearImage(imgOriginal)

		# get current positions of four trackbars
		binValue = cv2.getTrackbarPos('Thres','image')
		s = cv2.getTrackbarPos(switch,'image')

		k = cv2.waitKey(1) & 0xFF
		if k == 27:
			break

		if s == 0:
			pass 
		else:
			thresImage = rb.doThresHold(filteredImage,binValue)
			cv2.imshow('img', thresImage)

	cv2.destroyAllWindows()

UVCDYNCTRLEXEC = "/usr/bin/uvcdynctrl"
endProcess = 'n'
while endProcess != 'y':
	print "----------------------------"
	mainOperation = str(raw_input('What do you want to do? '))
	if mainOperation == 's':
		method = str(raw_input('Which method do you want to execute? '))
		value = str(raw_input('Which value? '))
		subprocess.Popen([UVCDYNCTRLEXEC,"-s",method,"--",value])
	elif mainOperation == 'g':
		method = str(raw_input('Which method do you want to execute? '))
		result = subprocess.Popen([UVCDYNCTRLEXEC,"-g",method])
		print result
	elif mainOperation == 'a':
		subprocess.Popen([UVCDYNCTRLEXEC,"-c"])
	elif mainOperation == 'c':
		goLive()
	elif mainOperation == 't':
		goLiveT()

			
	endProcess = str(raw_input('Do you want to finish the process? '))
	
