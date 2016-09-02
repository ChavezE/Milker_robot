'''
Created by: EMIBAS		01/09/2016
Version 1.0

Objective: This program needs to be runned every time the camera is connected to the RaspberryPi.
The main purpose is to calibrate some commands that are necessary in order to take useful pictures
when searching for the cow. The command that is used is the 'uvcdynctrl'.
'''
import gtk
import subprocess
import time

UVCDYNCTRLEXEC = "/usr/bin/uvcdynctrl"

def initializeCamera():
	subprocess.Popen([UVCDYNCTRLEXEC,"-s",'Pan Reset',"--",'1'])
	time.sleep(3)
	subprocess.Popen([UVCDYNCTRLEXEC,"-s",'Tilt Reset',"--",'1'])
	time.sleep(3)
	subprocess.Popen([UVCDYNCTRLEXEC,"-s",'Backlight Compensation',"--",'0'])
	subprocess.Popen([UVCDYNCTRLEXEC,"-s",'Exposure, Auto',"--",'1'])
	subprocess.Popen([UVCDYNCTRLEXEC,"-s",'White Balance Temperature, Auto',"--",'0'])
	subprocess.Popen([UVCDYNCTRLEXEC,"-s",'Focus',"--",'0'])
	subprocess.Popen([UVCDYNCTRLEXEC,"-s",'Gain',"--",'30'])


initializeCamera()
