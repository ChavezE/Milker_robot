import gtk
import subprocess
import time
import cv2
import numpy as np

def goLive():
	cap = cv2.VideoCapture(0)
	while (1):
		_, imgOriginal = cap.read()
		cv2.imshow('im',imgOriginal)
		c = cv2.waitKey(50)
		if c == 27:
			break
	cv2.destroyAllWindows()

UVCDYNCTRLEXEC = "/usr/bin/uvcdynctrl"
endProcess = 'n'
while endProcess != 'y':
	print "----------------------------"
	mainOperation = input('What do you want to do? ')
	if mainOperation == 's':
		method = str(input('Which method do you want to execute? '))
		value = str(input('Which value? '))
		subprocess.Popen([UVCDYNCTRLEXEC,"-s",method,"--",value])
	elif mainOperation == 'g':
		method = str(input('Which method do you want to execute? '))
		result = subprocess.Popen([UVCDYNCTRLEXEC,"-g",method])
		print result
	elif mainOperation == 'a':
		subprocess.Popen([UVCDYNCTRLEXEC,"-c"])
	elif mainOperation == 'c':
		goLive()

	endProcess = str(input('Do you want to finish the process? '))
	
