# advice the arduino microcontroler that we are on line
# sending signal and wait for aknowledgement 
def checkForArduino():
	arduino.write('b') # sending the ping
	while (arduino.inWaiting() < 0):
		pass
	if (arduino.read(1) == 'b') # arduino is alive and ready
	return True;

def moveBot(cm):
	if cm == "forward":
		cm = "50"
	elif cm == "backward":
		cm = "-50"

	arduino.write('4')
	arduino.write('0')
	arduino.write(cm)
	while(arduino.inWaiting() < 0):
		pass
	return(arduino.read(1))

def turnBot(degrees):
	if degrees = "right":
		degrees = "90"
	elif degrees = "left":
		degrees = "-90"

	arduino.write('4')
	arduino.write('1')
	arduino.write(degrees)
	while(arduino.inWaiting() < 0):
		pass
	return(arduino.read(1))

def updateDirection(lasDir):
	if lasDir == "N":
		return "E"
	elif lasDir == "S":
		return "W"
	elif lasDir == "E":
		return "S"
	else:
		return "N"


# explore the arena to find the desired misterious liquid (milk)
# The coordenates of the arena will be taken this way:
# This approach starts after picking up the terrine
'''
	NOMENCLATURE:
	1. Empty Terrines zone
	2. Drop Terrines zone

			 EAST
	 	 _____________
   		|			  | 
NORTH 	|			  |	SOUTH
   		|			 º|
   		-2----	-----1-
   			 ***

   			 WEST
'''
# GLOBAL DIRECTION VARIABLE
headingWall = "N"
def analizeEnviroment():

	while 1:
		if checkCorner:
			turnBot("45")
			takePicture()
			# BRAKING INTO OTHER STATE HERE
			# BRAKING INTO OTHER STATE HERE
			turnBot("-45")
			checkCorner = False

		# face center of arena and look for cow
		turnBot("right")
		takePicture() # if COW IS FOUND, BRAKE THIS STATE AND GO TO NEXT
		# BRAKING INTO OTHER STATE HERE
		# BRAKING INTO OTHER STATE HERE
		turnBot("left") 

		takePicture() # to see if can keep moving on that direction
		res = moveBot("forward")
		if res == "0":
			
		elif res == "-1":
			# most probable you're at the corner
			# check the corner
			checkCorner = True
			turnBot("right")
			headingWall = updateDirection (headingWall) # update global 



# After we milked cow, it's time to go to the gate
def returnToWall():

	arduino.write("10")
	# Get to the wall you were before
	if headingWall == "E":
		arduino.write("0")
	elif headingWall == "S":
		arduino.write("90")
	else headingWall == "W":
		arduino.write("180")
	# note : if you're heading to N you can't trust your wall

	









