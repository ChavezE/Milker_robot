# This class would store rectangles
# and sort them to be used as vertical
# indicators
class Mylegs:
	""" squares here """
	# variables
	self.leftLeg = [] # here we store coordenates of left leg
	self.contLeftLeg = 0 # This will tell us the size of the list 'leftLeg'
	self.rightLeg = [] # coord for right leg
	self.contRightLeg = 0 # This will tell us the size of the list 'rightLeg'
	#
	def __init__(self, listaCuadros):
		# looking for the upper left-est square
		self.mostLeft = listaCuadros[0]
		for i in range(0,len(listaCuadros)):
		 	if (listaCuadros[i][0] <= self.mostLeft[0]) and (listaCuadros[i][1]
			<= self.mostLeft[1]):
			# When you find the upper left corner assign it to the atribute
		 		self.mostLeft = listaCuadros[i]
		# Assign the value find as the first coordinate of the left Leg
		self.leftLeg[0] = self.mostLeft
		self.contLeftLeg += 1
	def findLeftLeg(self, listaCuadros):
		# Define 'epsilon' which is the tolerance in px to find the other
		# coordinates that form the left Leg
		epsilon = 5
		# Looking for the coordinates that have the same 'x' coordinate
		for i in range(0, len(listaCuadros)):
			if(abs(listaCuadros[i][0] - self.leftLeg[0]) < epsilon):
				self.leftLeg[self.contLeftLeg] = listaCuadros[i][0]
				self.contLeftLeg += 1


lista = [[2,2],[2,3],[3,1],[7,0],[2,0]]

pata = Mylegs(lista)
print (pata.mostLeft)
