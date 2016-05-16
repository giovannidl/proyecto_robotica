#!/usr/bin/env python
#import rospy

#from std_msgs.msg import String

class Master:
	def prettyMaze(self,x,y,uglyMaze):
		for i in range(x):
			for j in range(y):
				uglyMaze[i][j] = uglyMaze[i][j][2:]
		return uglyMaze

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

	def findPath(self,maze,start,finish,depth, set_print):
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
					if set_print:
						print(visited)
		return actual[1]

	def escucha(self,msg):
		print(msg.data)
		if msg.data == '1':
			self.done = True

	def __init__(self):
		dimX, dimY, maze, initial, objective, depth = self.loadWorld('laberintos/c1.txt')
		self.X = dimX
		self.Y = dimY
		print("Celdas")
		for celdas in maze:
			print(celdas)
		self.start = initial
		self.objective = objective
		self.depth = depth
		self.camino = self.findPath(maze,initial,objective,depth, False)
		self.done = False
		print("Camino")
		for path in self.camino:
			print(path)
			

		#rospy.init_node('brain',anonymous=True)
		#self.go = rospy.Publisher('todo',String)
		#rospy.Subscriber('done',String,self.escucha)

	def makePath(self):
		ans = ""
		for paso in self.camino:
			ans += paso+"#"
		return ans[:-1]

if __name__ == "__main__":
	mas = Master()
	mens = mas.makePath()
	#while not rospy.is_shutdown() and not mas.done:
	#mas.go.publish(mens)
	print("Termine")
	#rospy.spin()
