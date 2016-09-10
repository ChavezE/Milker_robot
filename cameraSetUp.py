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

subprocess.Popen([UVCDYNCTRLEXEC,"-s",'Pan Reset',"--",'1'])
time.sleep(3)
subprocess.Popen([UVCDYNCTRLEXEC,"-s",'Tilt Reset',"--",'1'])
time.sleep(3)
subprocess.Popen([UVCDYNCTRLEXEC,"-s",'Backlight Compensation',"--",1'0'])
time.sleep(3)
subprocess.Popen([UVCDYNCTRLEXEC,"-s",'White Balance Temperature, Auto',"--",'0'])
time.sleep(3)
subprocess.Popen([UVCDYNCTRLEXEC,"-s",'Tilt (relative)',"--",'-1200'])
        

