# This class would store rectangles
# and sort them to be used as vertical
# indicators
def upperLeftCorner(list1):
	# 'epsilon' is the error range in pixels
    epsilon = 0
	# The function "sorted" sorts a multidimensional list. Depending on x[0]
	# is the index that you use to compare
    list1 = sorted(list1, key=lambda x: x[0], reverse=False)
	# Now that we have the list ordered from left to right we need to find the
	# coordinate that is closest to the origin (0,0)
    for i in range(0, len(list1)):
        if abs(list1[0][0] - list1[i][0]) <= epsilon and list1[0][1] > list1[i][1]:
            temp = list1[0]
            list1[0]=list1[i]
            list1[i]=temp
    return list1[0]

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
		mostLeft = upperLeftCorner(listaCuadros)

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
