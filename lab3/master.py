#!/usr/bin/env python
import rospy

from std_msgs.msg import String

def shiftByn(lista,n): 
		return lista[n::]+lista[:n:]

'''
Moví shiftByn hacia fuera de la clase, porque no depende
directamente de la clase y porque me tiraba el error de que no estaba definido.
Si lo queremos meter a la clase hay que agregarle un self en los parámetros y
cambiar la llamada a self.shiftByn
'''

class Master:
	def prettyMaze(self,x,y,uglyMaze):
		prettyMaze = []
		for i in range(x):
			col = []
			for j in range(y):
				data = uglyMaze[i*y+j][2:]
				col.append(data)
			prettyMaze.append(col)
		return prettyMaze

	def loadWorld(self,filename):
		archivo = open(filename)
		datos = archivo.read().splitlines()
		y,x = datos[0].split(' ')
		x = int(x)
		y = int(y)
		maze = []
		for i in range(x):
			for j in range(y):
				celda = datos[i*y+j+1].split(' ')
				maze.append(celda)
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
		if direction == 'u':
			ans.append([[y,x,'r'],moves+['Right']])
			ans.append([[y,x,'l'],moves+['Left']])
			if maze[x][y][0] == '0' and (y < self.Y):
				ans.append([[y+1,x,direction],moves+['Go']])
		elif direction == 'l':
			ans.append([[y,x,'u'],moves+['Right']])
			ans.append([[y,x,'d'],moves+['Left']])
			if maze[x][y][1] == '0' and (x > 0):
				ans.append([[y,x-1,direction],moves+['Go']])
		elif direction == 'd':
			ans.append([[y,x,'l'],moves+['Right']])
			ans.append([[y,x,'r'],moves+['Left']])
			if maze[x][y][2] == '0' and (y > 0):
				ans.append([[y-1,x,direction],moves+['Go']])
		else:
			ans.append([[y,x,'d'],moves+['Right']])
			ans.append([[y,x,'u'],moves+['Left']])
			if maze[x][y][3] == '0' and (x < self.X):
				ans.append([[y,x+1,direction],moves+['Go']])
		return ans

	def findPath(self,maze,start,finish,depth):
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
					print(visited)
		return actual[1]

	def escucha(self,msg):
		print(msg.data)
		if msg.data == '1':
			self.done = True
		else:
			self.walls = msg.data.split('#')[:-1]

	def __init__(self):
		dimX, dimY, maze, initial, objective, depth = self.loadWorld('c.txt')
		awesomeMaze = self.prettyMaze(dimX, dimY, maze)
		self.X = dimX
		self.Y = dimY
		self.maze = awesomeMaze
		for celda in awesomeMaze:
			print(celda)
		self.start = initial
		self.objective = objective
		self.depth = depth
		self.camino = []
		self.walls = []
		self.done = False
		self.current = []

		rospy.init_node('brain',anonymous=True)
		self.go = rospy.Publisher('todo',String)
		self.loc = rospy.Publisher('find',String)
		rospy.Subscriber('done',String,self.escucha)

	def makePath(self):
		self.camino = self.findPath(awesomeMaze,initial,objective,depth)
		for path in self.camino:
			print(path)			
		ans = ""
		for paso in self.camino:
			ans += paso+"#"
		return ans[:-1]

	
	def localize(self):
		##Primero se buscan todos los estados en los que podria estar
		for x in range(len(self.maze)):#la variable 'awesomeMaze' tiraba error de que no estaba definido
			for y in range(len(self.maze)):#la cambié por self.maze definida en el init
				self.current.append([[x,y,'u']]+shiftByn(self.maze[x][y],0))
				self.current.append([[x,y,'l']]+shiftByn(self.maze[x][y],1))
				self.current.append([[x,y,'d']]+shiftByn(self.maze[x][y],2))
				self.current.append([[x,y,'r']]+shiftByn(self.maze[x][y],3))
		print(len(self.current))
		##Lo hacemos girar y que vea las murallas que lo rodean
		while len(self.current) > 1:
			while len(self.walls) < 4:
				self.loc.publish('Right#Right#Right#Right')
			##Quitamos los estados que no tengan esas murallas
			remove = []
			for choice in range(len(self.current)):
				if self.current[choice][1] != currentWalls:
					remove.append(choice)
			while len(remove) > 0:
				aux = remove.pop()
				self.current.pop(aux)
			##Hacemos que haga una accion y se repite el codigo
			msg = ''
			for pared in self.walls:
				if pared == '1':
					msg += 'Right#'
				else:
					break
			msg =+ 'Go'
			self.current = self.actualizarEstados()
			self.walls = []
			print('Parece que estoy en:')
			for estado in self.current:
				print(estado[0])
			while not self.done:
				self.go.publish(msg)
			self.done = False

	def actualizarEstados():
		new = []
		for estado in self.current:
			if estado[0][2] == 'u':
				newX = estado[0][0]
				newY = estado[0][1] + 1
				new.append([[newX,newY,'u']]+shiftByn(awesomeMaze[newX][newY],0))
			elif estado[0][2] == 'l':
				newX = estado[0][0] - 1
				newY = estado[0][1]
				new.append([[newX,newY,'l']]+shiftByn(awesomeMaze[newX][newY],1))
			elif estado[0][2] == 'd':
				newX = estado[0][0]
				newY = estado[0][1] - 1
				new.append([[newX,newY,'d']]+shiftByn(awesomeMaze[newX][newY],2))
			elif estado[0][2] == 'r':
				newX = estado[0][0] + 1
				newY = estado[0][1]
				new.append([[newX,newY,'u']]+shiftByn(awesomeMaze[newX][newY],3))
		return new

if __name__ == "__main__":
	mas = Master()
	start = mas.localize()
	mas.start = start
	mens = mas.makePath()
	while not rospy.is_shutdown() and not mas.done:
		mas.go.publish(mens)
	rospy.spin()
