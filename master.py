#!/usr/bin/env python
import rospy

from std_msgs.msg import String

'''
Movi shiftByn hacia fuera de la clase, porque no depende
directamente de la clase y porque me tiraba el error de que no estaba definido.
Si lo queremos meter a la clase hay que agregarle un self en los parametros y
cambiar la llamada a self.shiftByn
'''
def shiftByn(lista,n): 
		return lista[n::]+lista[:n:]

class Master:

	def loadWorld(self,filename):
		archivo = open(filename)
		datos = archivo.read().splitlines()
		y,x = datos[0].split(' ')
		x = int(x)
		y = int(y)
		maze = [[None for i in range(x)] for j in range(y)]
		for i in range(x*y):
				celda = datos[i+1].split(' ')
				maze[int(celda[0])][int(celda[1])] = celda[2:]
		nStart = int(datos[x*y+2])
		starts = []
		for i in range(nStart):
			aux = datos[x*y + i + 3].split(' ')
			starts.append([int(aux[0]),int(aux[1]),aux[2]])
		nGoals = int(datos[x*y+2+nStart+2])
		goals = []
		for i in range(nGoals):
			aux = datos[x*y+5+nStart+i].split(' ')
			goals.append([int(aux[0]),int(aux[1]),aux[2]])
		maxDepth = int(datos[-1])
		return x,y,maze,starts,goals,maxDepth

	def possible(self,state,maze):
		ans = []
		y,x,direction = state[0]
		moves = state[1]
		if (y >= 0 and y < self.Y and x >= 0 and x < self.X):
			if direction == 'u':
				ans.append([[y,x,'r'],moves+['Right']])
				ans.append([[y,x,'l'],moves+['Left']])
				if maze[y][x][0] == '0' and (y < self.Y):
					ans.append([[y+1,x,direction],moves+['Go']])
			elif direction == 'l':
				ans.append([[y,x,'u'],moves+['Right']])
				ans.append([[y,x,'d'],moves+['Left']])
				if maze[y][x][1] == '0' and (x > 0):
					ans.append([[y,x-1,direction],moves+['Go']])
			elif direction == 'd':
				ans.append([[y,x,'l'],moves+['Right']])
				ans.append([[y,x,'r'],moves+['Left']])
				if maze[y][x][2] == '0' and (y > 0):
					ans.append([[y-1,x,direction],moves+['Go']])
			else:
				ans.append([[y,x,'d'],moves+['Right']])
				ans.append([[y,x,'u'],moves+['Left']])
				if maze[y][x][3] == '0' and (x < self.X):
					ans.append([[y,x+1,direction],moves+['Go']])
		return ans

	def findPath(self,maze,start,finish,depth,set_print):
		actual = [[],[]]
		next = []
		for initial in start:
			next.append([initial,[]])
		visited = []
		while actual[0] not in finish:
			if len(next) == 0:
				print('NO WAY')
				return []
			else:
				actual = next.pop(0)
				if actual[0] not in visited:
					visited.append(actual[0])
					pos = self.possible(actual,maze)
					for neighbour in pos:
						if neighbour[0] not in visited:
							next.append(neighbour)
					#if set_print:
					#	print(visited)
		return actual[1]

	def escucha(self,msg):
		if msg.data == 'done':
			self.done = True
		elif len(msg.data) == 1:
			self.what = int(msg.data)
		else:
			self.walls = msg.data.split('#')

	def __init__(self):
		dimX, dimY, self.maze, initial, objective, depth = self.loadWorld('laberintos/c.txt')
		self.X = dimX
		self.Y = dimY
#		print("Celdas")
#		for celdas in self.maze:
#			print(celdas)
		self.start = initial
		self.objective = objective
		self.depth = depth
		self.camino = []
		self.walls = []
		self.done = False
		self.current = []
		self.idMsg = 0
		self.emptyMaze = [[['0','0','0','0'] for i in range(dimX)] for j in range(dimY)]
		self.what = -1

		rospy.init_node('brain', anonymous=True)
		self.go = rospy.Publisher('todo', String)
		self.loc = rospy.Publisher('find', String)
		rospy.Subscriber('done', String, self.escucha)

		self.init_current_localization()

	def init_current_localization(self):
		##Primero se buscan todos los estados en los que podria estar
		for y in range(len(self.maze)):
			for x in range(len(self.maze[y])):
				self.current.append([[y, x, 'u']] + [shiftByn(self.maze[y][x], 0)])
				self.current.append([[y, x, 'l']] + [shiftByn(self.maze[y][x], 1)])
				self.current.append([[y, x, 'd']] + [shiftByn(self.maze[y][x], 2)])
				self.current.append([[y, x, 'r']] + [shiftByn(self.maze[y][x], 3)])

	def makePath(self,maze):
		self.camino = self.findPath(maze,self.start,self.objective,self.depth, False)
		print(self.start)
		#print("Camino")
		#for path in self.camino:
		#	print(path)
		ans = ""
		for paso in self.camino:
			ans += paso+"#"
		return ans[:-1]

	def localize(self):
		##Lo hacemos girar y que vea las murallas que lo rodean
		primerIntento = True
		while len(self.current) > 1 and (not rospy.is_shutdown()):
			con = 0
			while len(self.walls) < 4:
				self.loc.publish('Left#Left#Left#Left')
			self.loc.publish('')
			self.done = False
			print(self.walls)
			##Quitamos los estados que no tengan esas murallas
			remove = []
			currentBackup = [] + self.current;
			for choice in range(len(self.current)):
				if self.current[choice][1] != self.walls:
					remove.append(choice)
			while len(remove) > 0:
				self.current.pop(remove.pop(-1))
			##Hacemos que haga una accion y se repite el codigo
			## Hacemos mas de un intento para asegurarnos que no hay solucion
			if len(self.current) < 1:
				if primerIntento:
					print 'Primer intento'
					primerIntento = False
					self.current = currentBackup
					self.walls = []
					continue
				else:
					print 'Segundo intento'
					break
			else:
				primerIntento = True
			msg = ''
			for pared in self.walls:
				if pared == '1':
					msg += 'Left#'
				else:
					break
			msg += 'Go'
			print('Parece que estoy en:')
			for estado in self.current:
				print(estado)
			if len(self.current) == 1:
				return [self.current[0][0]]
			self.current = self.actualizarEstados(msg)
			print('Parece que estare en:')
			for estado in self.current:
				print(estado)
			self.walls = []
			print(msg,self.done)
			while not self.done and not rospy.is_shutdown():
				self.go.publish(msg)
			self.done = False
			print(len(self.current), con)
			self.go.publish('')

	def actualizarEstados(self,instruccion):
		new = []
		for estado in self.current:
			aux = estado
			for message in instruccion.split('#'):
				if message == 'Go':
					if aux[0][2] == 'u':
						newX = aux[0][1]
						newY = aux[0][0] + 1
						print(newX, newY)
						new.append([[newY,newX,'u']]+[shiftByn(self.maze[newY][newX],0)])
					elif aux[0][2] == 'l':
						newX = aux[0][1] - 1
						newY = aux[0][0]
						print(newX, newY)
						new.append([[newY,newX,'l']]+[shiftByn(self.maze[newY][newX],1)])
					elif aux[0][2] == 'd':
						newX = aux[0][1]
						newY = aux[0][0] - 1
						print(newX, newY)
						new.append([[newY,newX,'d']]+[shiftByn(self.maze[newY][newX],2)])
					elif aux[0][2] == 'r':
						newX = aux[0][1] + 1 #me tiro un error fuera de indice (4,0)
						newY = aux[0][0]
						print(newX, newY)
						new.append([[newY,newX,'r']]+[shiftByn(self.maze[newY][newX],3)])
				else:
					if aux[0][2] == 'u':
						aux[0][2] = 'l'
					elif aux[0][2] == 'l':
						aux[0][2] = 'd'
					elif aux[0][2] == 'd':
						aux[0][2] = 'r'
					elif aux[0][2] == 'r':
						aux[0][2] = 'u'
		return new

	def newState(self, state, action):
		where = state[2]
		y = state[0]
		x = state[1]
		if action == 'Right':
			if where == 'u':
				where = 'r'
			elif where == 'r':
				where = 'd'
			elif where == 'd':
				where = 'l'
			elif where == 'l':
				where = 'u'
		elif action == 'Left':
			if where == 'u':
				where = 'l'
			elif where == 'l':
				where = 'd'
			elif where == 'd':
				where = 'r'
			elif where == 'r':
				where = 'u'
		elif action == 'Go':
			if where == 'u':
				y = y + 1
			elif where == 'r':
				x = x + 1
			elif where == 'd':
				y = y - 1
			elif where == 'l':
				x = x - 1
		return [y,x,where]

	def manyStates(self, state, actions):
		for action in actions:
			state = self.newState(state,action)
		return state

	def modifyMap(self, maze, state):
		y = state[0]
		x = state[1]
		where = state[2]
		if where == 'u':
			maze[y][x][0] = 1
		elif where == 'l':
			maze[y][x][1] = 1
		elif where == 'd':
			maze[y][x][2] = 1
		elif where == 'r':
			maze[y][x][3] = 1
		return maze

	def explore(self):
		initial = self.start
		done = False
		while not done and not rospy.is_shutdown():
			print(self.start[0])
			mens = self.makePath(self.emptyMaze)
			if len(mens) == 0:
				break
			while not rospy.is_shutdown() and self.what < 0:
				self.go.publish(mens)
			actions = mens.split('#')
			self.go.publish('')
			if (len(actions) == self.what):
				done = True
				break
			hechos = []
			for i in range(self.what):
				hechos.append(actions.pop(0))
			aux = self.manyStates(self.start[0], hechos)
			self.start = [aux]
			self.emptyMaze = self.modifyMap(self.emptyMaze, self.start[0])
			self.what = -1

if __name__ == "__main__":
	mas = Master()
	mas.explore()
#	start = mas.localize()
#	mas.start = start
#	print(start)
#	print('ME ENCONTRE, ESTE ES EL TALADRO QUE PERFORARA EL LABERINTO, QUIEN COGNO OS CREEIS QUE SOY')
#	mens = mas.makePath(mas.maze)
#	while not rospy.is_shutdown() and not mas.done:
#		mas.go.publish(mens)
	rospy.spin()
