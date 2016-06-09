# This class would store rectangles
# and sort them to be used as vertical
# indicators
class Mylegs:
	""" squares here """
	# variables
	self.leftLeg = [] # here we store coordenates of left leg
	self.rightLeg = [] # coord for right leg
	#
	def __init__(self, listaCuadros):
		# looking for the upper left-est square
		self.mostLeft = listaCuadros[0]
		for i in range(0,len(listaCuadros)):
		 	if (listaCuadros[i][0] <= self.mostLeft[0]) and (listaCuadros[i][1] <= self.mostLeft[1]):
		 		self.mostLeft = listaCuadros[i]
	def findLeg(self):
		

lista = [[2,2],[2,3],[3,1],[7,0],[2,0]]

pata = Mylegs(lista)
print (pata.mostLeft)


		
