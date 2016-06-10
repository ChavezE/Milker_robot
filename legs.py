# This class would store rectangles
# and sort them to be used as vertical
# indicators
class Mylegs:
	""" squares here """


	def __init__(self, listaCuadros):

		# variables
		self.leftLeg = [] # here we store coordenates of left leg
		self.contLeftLeg = 0 # This will tell us the size of the list 'leftLeg'
		self.rightLeg = [] # coord for right leg
		self.contRightLeg = 0 # This will tell us the size of the list 'rightLeg'
		#

		# looking for left-est square
		mostLeft = listaCuadros[0] # auxiliar variable 
		for i in range(0,len(listaCuadros)): # first find the left
		 	if (listaCuadros[i][0] <= mostLeft[0]):
		 		mostLeft = listaCuadros[i]
		for i in range(0,len(listaCuadros)): # now find the upper
			if( listaCuadros[i][0] <= mostLeft[0] and listaCuadros[i][1] <= mostLeft[1]):
				mostLeft = listaCuadros[i]

		self.leftLeg.insert(0,mostLeft)
		self.contLeftLeg = self.contLeftLeg + 1 


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
