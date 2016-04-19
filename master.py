#!/usr/bin/env python
import rospy

from std_msgs.msg import String

class Master:
	def prettyMaze(self,x,y,uglyMaze):
		prettyMaze = []
		for i in range(x):
			col = []
			for j in range(y):
				data = uglyMaze[i*x+j][2:]
				col.append(data)
			prettyMaze.append(col)
		return prettyMaze

	def loadWorld(self,filename):
		archivo = open(filename)
		datos = archivo.read().splitlines()
		x,y = datos[0].split(' ')
		x = int(x)
		y = int(y)
		maze = []
		for i in range(x):
			for j in range(y):
				celda = datos[i*x+j+1].split(' ')
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
			if maze[x][y][0] == '0':
				ans.append([[y+1,x,direction],moves+['Go']])
		elif direction == 'l':
			ans.append([[y,x,'u'],moves+['Right']])
			ans.append([[y,x,'d'],moves+['Left']])
			if maze[x][y][1] == '0':
				ans.append([[y,x-1,direction],moves+['Go']])
		elif direction == 'd':
			ans.append([[y,x,'l'],moves+['Right']])
			ans.append([[y,x,'r'],moves+['Left']])
			if maze[x][y][2] == '0':
				ans.append([[y-1,x,direction],moves+['Go']])
		else:
			ans.append([[y,x,'d'],moves+['Right']])
			ans.append([[y,x,'u'],moves+['Left']])
			if maze[x][y][3] == '0':
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
				return False
			else:
				actual = next.pop(0)
				visited.append(actual[0])
				pos = self.possible(actual,maze)
				for neighbour in pos:
					if neighbour[0] not in visited:
						next.append(neighbour)
		return actual[1]

	def escucha(self,msg):
		if msg.data == '1':
			self.done = True

	def __init__(self):
		dimX, dimY, maze, initial, objective, depth = self.loadWorld('c.txt')
		awesomeMaze = self.prettyMaze(dimX, dimY, maze)
		self.X = dimX
		self.Y = dimY
		self.maze = awesomeMaze
		self.start = initial
		self.objective = objective
		self.depth = depth
		self.camino = self.findPath(awesomeMaze,initial,objective,depth)
		self.done = False

		rospy.init_node('brain',anonymous=True)
		self.go = rospy.Publisher('todo',String)
		rospy.Subscriber('done',String,self.escucha)

	def makePath(self):
		ans = ""
		for paso in self.camino:
			ans += paso+"#"
		return ans[:-1]

if __name__ == "__main__":
	mas = Master()
	mens = mas.makePath()
	while not rospy.is_shutdown() and not mas.done:
		mas.go.publish(mens)
	print("Termine")
	rospy.spin()
