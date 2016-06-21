#!/usr/bin/env python
import rospy
import math
import random
import numpy
import roslib; roslib.load_manifest('sound_play')

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

MAX_DISTANCE = 3
WALL_DISTANCE = 0.6

class Nodo:	
	def espera(self,seg):
		for i in range(int(self.rate*seg)):
			self.r.sleep()
	
	def parar(self):
		stop = Twist()
		self.cmd_vel.publish(stop)

	def modulo(self, X, Y):
		return math.sqrt(X**2+Y**2)

	def odometryCb(self,msg):
		if self.partir:
			self.inicioX = msg.pose.pose.position.x
			self.inicioY = msg.pose.pose.position.y
			#self.thetaI = (msg.pose.pose.orientation.z + 1) * 180
			self.partir = False
		self.posx = msg.pose.pose.position.x
		self.posy = msg.pose.pose.position.y
		if (msg.pose.pose.orientation.z > 0):
			self.theta = math.acos(msg.pose.pose.orientation.w)/(math.pi/2)*180
		else:
			self.theta = 360 - math.acos(msg.pose.pose.orientation.w)/(math.pi/2)*180
		#self.theta = (msg.pose.pose.orientation.z + 1)*180

	def obstaculo(self,msj):
		self.distance = map(float,msj.data.split(':'))

		self.left = self.distance[0] < 0.6 or self.distance[0] > 90

		if self.distance[1] < 0.6 or self.distance[1] > 90:
			self.stackDistance[1] = min(self.stackDistance[1] + 1, MAX_DISTANCE)
			self.center = not self.stackDistance[1] == MAX_DISTANCE
		else:
			self.stackDistance[1] = max(self.stackDistance[1] - 1, 0)
			self.center = not self.stackDistance[1] == 0

		self.right = self.distance[2] < 0.6 or self.distance[2] > 90

		self.auxPared =  max(abs(self.distance[3] - self.distance[1]), abs(self.distance[4] - self.distance[1]))
		if self.distance[3] < WALL_DISTANCE and self.distance[4] < WALL_DISTANCE and self.auxPared < 0.03 and self.auxPared > 0:
			self.pared = True
		else:
			self.pared = False

	def enderezame(self,data):
		self.distsPared = data.data.split(';')
		self.distsPared = map(float,self.distsPared)
		if self.center or self.left or self.right:
			if max(abs(self.distsPared[0] - self.distsPared[1]), abs(self.distsPared[1] - self.distsPared[2])) < 0.01 and self.distsPared[0] != 0:
				self.enderezado = True
			else:
				self.enderezado = False
			aux = self.distsPared[2] - self.distsPared[0]
			if aux != 0:
				self.sentidoEnderezado = (aux) / abs(aux)
		else:
			self.enderezado = True

	def solve(self,msg):
		if (msg.data == ''):
			self.todo = []
			#print('SOY UN NINO LIMPIO')
		elif len(self.todo) == 0:
			self.todo = msg.data.split('#')
			self.actions.publish(msg.data)
			print self.todo
			self.ocupado = True
			acciones_terminadas = 0
			for accion in self.todo:
				self.espera(0.2)
				print(accion)
				print('Solve Center: ' + str(self.distance[1]) + ' Pared: ' + str(self.pared) + ' DisPared: ' + str(self.auxPared))
				if accion == "Go":
					if self.distance[1] < WALL_DISTANCE: # and self.pared:
						print('ignoring go')
						pass
					else:
						self.avanza(self.largoPared,0.4)
				elif accion == "Left":
					self.gira(90,1)
				elif accion == "Right":
					self.gira(270,-1)
				if (max(self.distance) < 0.8):
					self.enderezar(1)
				acciones_terminadas += 1
			self.slave.publish(str(acciones_terminadas))
			self.espera(0.2)
			print('Im done')
			self.slave.publish('done')
			#self.chatter.say('Goal reached, its time to party')
			self.espera(0.5)
			self.ocupado = False

	def loc(self,msg):
		if (msg.data == ''):
			self.todo = []
		elif len(self.todo) == 0:
			self.todo = msg.data.split('#')
			self.actions.publish(msg.data)
			paredes = ''
			self.ocupado = True
			for accion in self.todo:
				self.espera(0.2)
				if self.identificaPared():
					paredes += '1#'
					print(self.items)
				else:
					paredes += '0#'
				if accion == "Go":
					self.avanza(self.largoPared,0.4)
				elif accion == "Left":
					self.gira(90,1)
				elif accion == "Right":
					self.gira(270,-1)
				if self.distance[1] < 0.8:
					self.enderezar(1)
			print('ugh')
			if self.collected():
				self.collection.publish('True')
				print(self.collector,'collection')
				#self.chatter.say('All set, annihilation incoming')
			self.slave.publish(paredes[:-1])
			self.espera(0.5)
			self.ocupado = False

	def cosas(self, data):
		self.items = data.data.split(':')
		#print(self.items)

	def verify(self, data):	
		if data.data == 'Clear':
			self.collector = [False,False,False]
		elif data.data == 'find':#falta probar si sirve
			aux = []
			vueltas = 0
			for i in range(4):
				if self.items[2] == '1' and self.auxPuerta == 0:
					self.chatter.say('Door found')
					print('puerta!',self.auxPuerta,aux)
					if len(aux) > 1:
						self.hodor.publish('#'.join(aux))#para saber posicion exacta de la puerta, no tiene un uso claro
					self.auxPuerta += 1
					break
				else:
					print('no puerta :c')
					roboto.gira(90,1)
					roboto.enderezar(1)
					aux.append('Left')
					rospy.sleep(1)
					vueltas += 1

	def __init__(self):
		#Aca se definen variables utiles
		self.posx = 0
		self.posy = 0
		self.inicioX = 0
		self.inicioY = 0
		self.theta = 0
		self.cl = 0.03
		self.partir = False
		self.rate = 20
		self.right = False
		self.center = False
		self.left = False
		self.distance = [10,10,10]
		self.stackDistance = [MAX_DISTANCE / 2,MAX_DISTANCE / 2,MAX_DISTANCE / 2]
		self.objetivo = [0,0,0]
		self.pierdeObjetivo = False
		self.sentidoObjetivo = 1
		self.alcanceObjetivo = False
		self.distsPared = [0,0,0]
		self.enderezado = False
		self.sentidoEnderezado = 1
		self.ocupado = False
		self.largoPared = 0.8
		self.todo = []
		self.pared = False
		self.auxPared = 0
		self.auxPuerta = 0
		self.items = [False,False,False]
		self.collector = [False,False,False]

		#Inicializar el nodo y suscribirse/publicar
		rospy.init_node('roboto', anonymous=True) #make node 
		rospy.Subscriber('odom',Odometry,self.odometryCb)
		rospy.Subscriber('obstaculo',String,self.obstaculo)
		rospy.Subscriber('enderezador3',String,self.enderezame)
		rospy.Subscriber('todo',String,self.solve)
		rospy.Subscriber('find',String,self.loc)
		rospy.Subscriber('watchRoboto',String,self.cosas)
		rospy.Subscriber('puertas',String,self.verify)
		self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist)
		self.slave = rospy.Publisher('done', String)	
		self.collection = rospy.Publisher('colectabuzz', String)	
		self.actions = rospy.Publisher('sapo', String)	
		self.hodor = rospy.Publisher('doorFinder', String)	
		self.check = rospy.Publisher('ask', String)	
		self.r = rospy.Rate(20);  #se asegura de mantener el loop a 20 Hz
		self.chatter = SoundClient()

	def avanza(self,metros,vel):
		self.inicio = 0
		self.partir = True
		cont = 0.1
		move_cmd = Twist()
		move_cmd.angular.z = -0.02
		move_cmd.linear.x = cont * vel #m/s
		vel_max = vel
		recorrido = 0
		while (not rospy.is_shutdown()) and (abs(recorrido) < metros*(1-self.cl)) or self.partir:
			self.cmd_vel.publish(move_cmd)
			self.r.sleep()
			recorrido = self.modulo(self.posx - self.inicioX, self.posy - self.inicioY)
			if not self.center and (abs(recorrido) < metros*(1-self.cl) * 0.9):
				cont = min(cont + 0.05, 1)
				#print('acelero')
			else:
				cont = max(cont - 0.05, 0.1)
				#print('freno')

			#if (self.distance[0]) > 0.5:
			if self.right and not self.left:
				move_cmd.angular.z = 1
				cont = max(cont - 0.05, 0.5)
				#cont = 0.3
			#elif (self.distance[2]) > 0.5:
			elif self.left and not self.right:
				cont = max(cont - 0.05, 0.5)
				move_cmd.angular.z = -1
				#cont = 0.3
			else:
				move_cmd.angular.z = -0.02
			move_cmd.linear.x = vel_max * cont
			#print(vel_max * cont)
			if (self.distance[1] < 0.5 and self.distance[1] != 0.0):
				#print(self.distance[1])
				break
		while (not rospy.is_shutdown()) and self.distance[1] < 0.1:
			move_cmd.linear.x = -0.1
			move_cmd.angular.z = 0
			self.cmd_vel.publish(move_cmd)
		self.parar()
		self.espera(0.7)

	def gira(self,grados,vel): 
		objetivo = grados
		#zobj = objetivo/180 - 1
		move_cmd = Twist()
		move_cmd.angular.z = vel
		error = 3
		self.partir = True
		thetaReal = True
		while (not rospy.is_shutdown()) and ((abs(self.theta - objetivo) > error) or thetaReal):
			if not self.partir and thetaReal:
				thetaReal = False
				objetivo += self.theta
				#print(self.theta, objetivo)
				if (objetivo > 360):
					objetivo -= 360
			self.cmd_vel.publish(move_cmd)
			self.r.sleep()
			#if (vel > 0 and not self.right) or (vel < 0 and not self.left):
			#	break
		self.parar()
		self.espera(0.7)

	def arco(self, radio, grados, angular):
		#piso liso: 1.51, piso rugoso: 1.93
		objetivo = grados
		move_cmd = Twist()
		move_cmd.angular.z = angular
		move_cmd.linear.x = abs(angular) * radio / 1.85
		error = 5
		self.partir = True
		thetaReal = True
		while (not rospy.is_shutdown()) and ((abs(self.theta - objetivo) > error) or thetaReal):
			if not self.partir and thetaReal:
				thetaReal = False
				objetivo = grados + self.theta
				if (objetivo > 360):
					objetivo -= 360
			self.cmd_vel.publish(move_cmd)
			self.r.sleep()
		self.parar()
		self.espera(0.7)

	def ncuadrados(self, n, lado, vel):
		for i in range(n):
			self.cuadrado(lado, vel)	

	def cuadrado(self,lado,vel):
		for i in range(4):
			self.avanza(lado,vel)
			self.gira(90,1.5)

	def hacerDCC(self,radio,velL,velA):
		#Hacer D
		self.avanza(radio*2, velL)
		self.gira(90, velA + 0.5)
		self.arco(radio, 180, velA)
		#Hacer C
		#self.gira(180, velA)
		self.avanza(2.2*radio, -velL)
		#self.gira(180, velA)
		self.espera(1)
		self.arco(radio, 180, velA)
		#Hacer otra C
		self.avanza(2.2*radio, velL)
		self.espera(1)
		self.gira(180, velA + 0.5)
		self.espera(1)
		self.arco(radio, 180, -velA)

	def pasea(self):
		while (not rospy.is_shutdown()):
			if not (self.left or self.right):
				self.avanza(10,0.4)
			elif self.left:
				self.gira(10000, -1)
			else:
				self.gira(10000, 1)

	def enderezar(self,vel):
		move_cmd = Twist()
		move_cmd.angular.z = vel * self.sentidoEnderezado
		while (not rospy.is_shutdown()):
			self.cmd_vel.publish(move_cmd)
			self.r.sleep()
			self.check.publish('1')
			#print(self.items)
			for i in range(len(self.items)):
				if self.items[i] == '1' and i == 0:
					self.collector[0] = True
				elif self.items[i] == '1' and i == 1:
					self.collector[1] = True
				elif self.items[i] == '1' and i == 2:
					self.collector[2] = True
					self.hodor.publish('1')
			#print(self.collector)
			#self.yell()
			if (self.enderezado):
				break
		print(self.items)
		self.parar()
		self.espera(0.7)
		#roboto.chatter.say('Objective located ready for annihilation')

	def avanzaPasillo(self, vel):
		while(not rospy.is_shutdown()):
			if self.sentidoEnderezado > 0 and self.left:
				self.gira(10000, -1)
			elif self.sentidoEnderezado < 0 and self.right:
				self.gira(10000, 1)
			elif not self.center:
				self.avanza(10, vel)
			else:
				print("estamos rodeados")

			if self.center:
				self.enderezar(1)
				self.chatter.say('Llegue')
				break

	def identificaPared(self):
		print(self.distance)
		#if self.center and (self.left or self.right):
		print('Identifica' + str(self.pared))
		if self.distance[1] < 0.7:# and self.pared:
			self.enderezar(1)
			for j in range(4):
				#print('identificando')
				self.check.publish('1')
				for i in range(len(self.items)):
					if self.items[i] == '1' and i == 0:
						self.collector[0] = True
					elif self.items[i] == '1' and i == 1:
						self.collector[1] = True
					elif self.items[i] == '1' and i == 2:
						self.collector[2] = True
						self.hodor.publish('1')
			print(self.collector)
			self.yell()
			# self.chatter.say('Objective lost')
			print('Pared')
			val = True
		else:
			print('Pasillo')
			#self.chatter.say('Objective found')
			val = False
		rospy.sleep(1)
		return val
				
	def buscaPared(self):
		for i in range(4):
			self.identificaPared()
			self.gira2(90,1)

	def yell(self):
		self.chatter.stopAll()
		if self.items[0] == '1':
			self.chatter.say('AverageMan ready to kill')
		elif self.items[1] == '1':
			self.chatter.say('One step closer to murder')
		elif self.items[2] == '1':
			self.chatter.say('Killing spree is ours')
		else:
			self.chatter.say('Boring')

	def collected(self):
		aux = 0
		for i in range(len(self.collector)):
			if self.collector[i] == True:
				aux += 1
		if aux == 3:
			return True
		return False

if __name__ == "__main__":
	roboto = Nodo()
	rospy.sleep(1)
	#for i in range(4):
	#	roboto.gira(90,1)
	#	roboto.enderezar(1)
		#rospy.sleep(1)
	#	roboto.yell()
	#	rospy.sleep(1)
	rospy.spin()
