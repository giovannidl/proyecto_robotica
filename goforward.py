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
		
		'''
		if self.distance[0] < 0.6 or self.distance[0] > 90:
			self.stackDistance[0] = min(self.stackDistance[0] + 1, MAX_DISTANCE)
			self.left = not self.stackDistance[0] == MAX_DISTANCE
		else:
			self.stackDistance[0] = max(self.stackDistance[0] - 1, 0)
			self.left = not self.stackDistance[0] == 0
		'''
		self.left = self.distance[0] < 0.6 or self.distance[0] > 90

		if self.distance[1] < 0.6 or self.distance[1] > 90:
			self.stackDistance[1] = min(self.stackDistance[1] + 1, MAX_DISTANCE)
			self.center = not self.stackDistance[1] == MAX_DISTANCE
		else:
			self.stackDistance[1] = max(self.stackDistance[1] - 1, 0)
			self.center = not self.stackDistance[1] == 0

		'''
		if self.distance[2] < 0.6 or self.distance[2] > 90:
			self.stackDistance[2] = min(self.stackDistance[2] + 1, MAX_DISTANCE)
			self.right = not self.stackDistance[2] == MAX_DISTANCE
		else:
			self.stackDistance[2] = max(self.stackDistance[2] - 1, 0)
			self.right = not self.stackDistance[2] == 0
		'''
		self.right = self.distance[2] < 0.6 or self.distance[2] > 90

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
		#print msg
		if not self.ocupado and len(self.todo) == 0:
			self.todo = msg.data.split('#')
			print self.todo
			self.ocupado = True
			for accion in self.todo:
				self.espera(0.2)
				print(accion)
				if accion == "Go":
					#self.chatter.say("Avanza")
					#self.avanzaSimulador(self.largoPared,0.4)
					self.avanza(self.largoPared,0.4)
				elif accion == "Left":
					#self.chatter.say("Gira izquierda")
					#self.giraSimulador(90,1)
					self.gira(90,1)
				else:
					#self.chatter.say("Gira derecha")
					#self.giraSimulador(-90,-1)
					self.gira(270,-1)
				if (max(self.distance) < 1):
					#self.Enderezado = False
					self.enderezar(1)
			if len(self.todo) > 0:
				#self.slave.publish("1")
				#self.chatter.say('Bitch, Im awesome')
				self.chatter.say('Goal reached, its time to party')
			self.ocupado = False

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

		#Inicializar el nodo y suscribirse/publicar
		rospy.init_node('roboto', anonymous=True) #make node 
   		rospy.Subscriber('odom',Odometry,self.odometryCb)
		rospy.Subscriber('obstaculo',String,self.obstaculo)
		rospy.Subscriber('enderezador3',String,self.enderezame)
		rospy.Subscriber('todo',String,self.solve)	
		self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist)
		self.slave = rospy.Publisher('done',String)						
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
				print('acelero')
			else:
				cont = max(cont - 0.05, 0.1)
				print('freno')

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
				print(self.distance[1])
				break
		while (not rospy.is_shutdown()) and self.distance[1] < 0.1:
			move_cmd.linear.x = -0.1
			move_cmd.angular.z = 0
			self.cmd_vel.publish(move_cmd)
		self.parar()
		self.espera(0.7)

	def gira(self,grados,vel): 
		objetivo = grados - 5
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
			if (self.enderezado):
				break
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
		if self.center and (self.left or self.right):
			self.enderezar(1)			
			self.chatter.say('Objective found')
		else:
			self.chatter.say('Objective lost')
		rospy.sleep(1)
				
	def buscaPared(self):
		for i in range(4):
			self.identificaPared()
			self.gira2(90,1)

if __name__ == "__main__":
	roboto = Nodo()
	rospy.sleep(1)
	roboto.chatter.stopAll()	
	#roboto.gira(270,-1)
	roboto.avanza(3,0.4)
	rospy.sleep(1)
	rospy.spin()
