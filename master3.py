#!/usr/bin/env python
import rospy

from std_msgs.msg import String
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

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
		print(start,finish,'ROBOT QL')
		actual = [[],[]]
		next = []
		for initial in start:
			next.append([initial,[]])
		visited = []
		while actual[0] not in finish:
			#print(next,'AGG TMRE')
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
		#print(msg.data)
		if msg.data == 'done':
			self.done = True
		elif len(msg.data) == 1:
			self.what = int(msg.data)
		else:
			self.walls = msg.data.split('#')
		
	def collector(self,msg):
		if msg.data == 'True':
			self.collected = True
			print('done collecting')
	
	def recolector(self,msg):
		self.mensajes.append(msg.data)
		self.num_acc += 1
		print(self.mensajes,self.num_acc)

	def reconocedoor(self,msg):
		if msg.data == '1' and self.aux == 0:
			self.aparicion = self.num_acc 
			self.aux += 1
			print(self.aparicion,'aparece puerta')
		if len(msg.data) > 1 and self.aux == 1:
			actions = msg.data.split('#')
			self.puerta = self.manyStates(self.actual, actions)
			print('door found')

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
		self.collected = False
		self.mensajes = []
		self.num_acc = 0
		self.aparicion = None
		self.aux = 0
		self.actual = None
		self.puerta = None
		self.check_grid = None

		rospy.init_node('brain', anonymous=True)
		self.go = rospy.Publisher('todo', String)
		self.loc = rospy.Publisher('find', String)
		self.buscador = rospy.Publisher('puertas', String)
		rospy.Subscriber('done', String, self.escucha)
		rospy.Subscriber('colectabuzz', String, self.collector)
		rospy.Subscriber('sapo', String, self.recolector)
		rospy.Subscriber('doorFinder', String, self.reconocedoor)
		self.chatter = SoundClient()

		self.init_current_localization()

	def init_current_localization(self):
		##Primero se buscan todos los estados en los que podria estar
		for y in range(len(self.maze)):
			for x in range(len(self.maze[y])):
				self.current.append([[y, x, 'u']] + [shiftByn(self.maze[y][x], 0)])
				self.current.append([[y, x, 'l']] + [shiftByn(self.maze[y][x], 1)])
				self.current.append([[y, x, 'd']] + [shiftByn(self.maze[y][x], 2)])
				self.current.append([[y, x, 'r']] + [shiftByn(self.maze[y][x], 3)])

	# Reset the visited cells and set which are reachable
	# pos 0 -> Reachable
	# pos 1 -> Visited
	def reset_reachable_grid(self):
		self.check_grid = [[[True, False] for i in range(self.X)] for j in range(self.Y)]
		if self.start[0]:
			self.check_grid[self.start[0][0]][self.start[0][1]][1] = True
			for j in range(self.Y):
				for i in range(self.X):
					possible_route = self.findPath(self.maze, self.start, self.makeGoals(i, j), self.depth, False)
					if len(possible_route) == 0 and  [j, i] != self.start[0]:
						self.check_grid[j][i][0] = False

	def reset_check_grid(self):
		if self.start[0]:
			for j in range(self.Y):
				for i in range(self.X):
					if self.check_grid[j][i][0]:
						self.check_grid[j][i][1] = False

	def makeGoals(self,x,y):
		ans = []
		ans.append([y,x,'u'])
		ans.append([y,x,'l'])
		ans.append([y,x,'d'])
		ans.append([y,x,'r'])
		return ans

	def makePath(self,maze):
		#print(maze)
		self.camino = self.findPath(maze,self.start,self.objective,self.depth, False)
		#print("Camino")
		#for path in self.camino:
		#	print(path)
		ans = ""
		#self.camino = self.findPath(maze,self.start,self.objective,self.depth, False)
		for paso in self.camino:
			ans += paso+"#"
		print(self.camino[:-1])
		return ans[:-1]

	def localize(self):
		##Lo hacemos girar y que vea las murallas que lo rodean
		primerIntento = True
		while len(self.current) > 1 and (not rospy.is_shutdown()):
			con = 0
			while len(self.walls) < 4 and (not rospy.is_shutdown()):
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
		return [self.current[0][0]]

	def actualizarEstados(self,instruccion):
		new = []
		for estado in self.current:
			aux = estado
			for message in instruccion.split('#'):
				if message == 'Go':
					if aux[0][2] == 'u':
						newX = aux[0][1]
						newY = aux[0][0] + 1
						#print(newX, newY)
						new.append([[newY,newX,'u']]+[shiftByn(self.maze[newY][newX],0)])
					elif aux[0][2] == 'l':
						newX = aux[0][1] - 1
						newY = aux[0][0]
						#print(newX, newY)
						new.append([[newY,newX,'l']]+[shiftByn(self.maze[newY][newX],1)])
					elif aux[0][2] == 'd':
						newX = aux[0][1]
						newY = aux[0][0] - 1
						#print(newX, newY)
						new.append([[newY,newX,'d']]+[shiftByn(self.maze[newY][newX],2)])
					elif aux[0][2] == 'r':
						newX = aux[0][1] + 1 #me tiro un error fuera de indice (4,0)
						newY = aux[0][0]
						#print(newX, newY)
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

	def checkImages(self):
		while len(self.walls) < 4 and (not rospy.is_shutdown()):
			self.loc.publish('Left#Left#Left#Left')
		self.loc.publish('')
		self.done = False
		self.walls = []

	def newState(self, state, action, inverted = False):
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
			if inverted is False:
				if where == 'u':
					y = y + 1
				elif where == 'r':
					x = x + 1
				elif where == 'd':
					y = y - 1
				elif where == 'l':
					x = x - 1
			elif inverted is True:
				if where == 'u':
					y = y - 1
				elif where == 'r':
					x = x - 1
				elif where == 'd':
					y = y + 1
				elif where == 'l':
					x = x + 1
		return [y,x,where]

	def manyStates(self, state, actions, inverted = False):
		if inverted is False:
			for action in actions:
				state = self.newState(state,action)	
		elif inverted is True:
			for i in range(len(actions)):
				state = self.newState(state,actions[-1-i],True)
		return state

	def markManyStates(self, state, actions, inverted=False):
		if inverted is False:
			for action in actions:
				state = self.newState(state, action)
		elif inverted is True:
			print(actions, 'truth has come')
			for i in range(len(actions)):
				state = self.newState(state, actions[-1 - i], True)
				self.check_grid[state[1]][state[0]][1] = True

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

	def undo_actions(self, for_undo_list):
		actions = []
		for do in for_undo_list:
			if do != 'Go':
				do = do.split('#')
				for i in do:
					if i == 'Left':
						i = i.replace('Left', 'Right')
					elif i == 'Right':
						i = i.replace('Right', 'Left')
					actions.append(i)
			elif do == 'Go':
				actions.append(do)
			else:
				print(do)
		return actions

	def go_objective(self, objective):
		initial = self.start
		self.done = False
		aux = self.objective
		#print(initial, self.puerta, self.objective, 'lkdd')
		self.objective = objective
		print("Estoy en " + str(self.start) + ", ire a " + str(self.objective))
		mens = self.makePath(self.maze)
		while not self.done and not rospy.is_shutdown():
			self.go.publish(mens)
			#print(self.done)
		self.go.publish('')
		self.start = self.objective
		self.objective = aux
		self.done = False

	def modo_busqueda(self):
		self.reset_reachable_grid()
		self.set_check_state()
		while not self.collected and not rospy.is_shutdown():
			for j in range(self.Y):
				for i in range(self.X):
					# Verificamos las celdas que son alcanzables y no han sido visitadas
					if self.check_grid[j][i][0] and not self.check_grid[j][i][1] and not self.collected:
						self.go_objective(self.makeGoals(i,j))
						self.checkImages()
			# Si llega hasta aca y no se ha encontrado los objetivos, reinicio las celdas alcanzadas
			self.reset_check_grid()
	def set_check_state(self):
		undo_actions = self.undo_actions(self.mensajes)
		self.markManyStates(self.start[0], undo_actions, True)

	def findDoor(self):
		if self.aparicion is None:
			print('no encontre la puerta')
			return
		if self.aparicion == 0:
			undo = self.mensajes
		else:
			undo = self.mensajes[self.aparicion:]
		actions = []
		for do in undo:
			if do != 'Go':
				do = do.split('#')
				for i in do:
					if i == 'Left':
						i = i.replace('Left','Right')
					elif i == 'Right':
						i = i.replace('Right','Left')
					actions.append(i)
			elif do == 'Go':
					actions.append(do)
			else:
				print(do)
		self.puerta = self.manyStates(self.start[0],actions,True)
		print(self.puerta,'puerta')	
	
	def goDoor(self):
		initial = self.start
		self.done = False
		aux = self.objective
		self.objective = [self.puerta]
		#print(self.objective)
		mens = self.makePath(self.maze)
		while not self.done and not rospy.is_shutdown():
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
			self.what = -1
#			self.go.publish(mens)
#			print(self.done)
		self.go.publish('')
		self.done = False
		print("Estoy en " + str(self.start) + ", ire a " + str(self.objective))
		self.objective = aux
		self.buscador.publish('find') #falta probar si sirve

				

if __name__ == "__main__":
	mas = Master()
	mas.start = mas.localize()
	puerta = False
	if mas.puerta is not None:
		mas.findDoor()
		print('puertecinha')	
		puerta = True
	if mas.collected is False:
		print('entrando a modo busqueda')
		mas.modo_busqueda()
	if puerta is False:
		mas.findDoor()
		print('pupupu-puerta')
	print('Ire a la puerta')
	#mas.puerta = [1,3,'u']
	mas.goDoor()
	mas.buscador.publish('Clear')
	rospy.sleep(10)
	mas.chatter.say('Starting Mision')
	rospy.sleep(20)
	mens = 'Go'
	while not rospy.is_shutdown() and not mas.done:
		mas.go.publish(mens)
	mas.go.publish('')
	mas.start = [mas.manyStates(self.start, ['Go'])]
	mas.explore()
	print("Termine")
	#rospy.spin()
