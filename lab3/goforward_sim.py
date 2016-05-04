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
		self.center = self.distance[1] < 0.6 or self.distance[1] > 90
		self.right = self.distance[2] < 0.6 or self.distance[2] > 90

	def enderezame(self,data):
		self.distsPared = data.data.split(';')
		self.distsPared = map(float,self.distsPared)
		if min(self.distsPared) < 0.6 or max(self.distsPared) > 90:
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
					self.gira(-90,-1)
				if (max(self.distance) < 1):
					print(max(self.distance),'este')
					self.Enderezado = False
					self.enderezar(1)
			if len(self.todo) > 0:
				#self.slave.publish("1")
				self.chatter.say('Bitch, I am awesome')
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
		rospy.Subscriber('/turtlebot/odom',Odometry,self.odometryCb)
		rospy.Subscriber('obstaculo',String,self.obstaculo)
		rospy.Subscriber('amigoFiel',String,self.amigo)
		rospy.Subscriber('enderezador3',String,self.enderezame)
		rospy.Subscriber('todo',String,self.solve)
		self.cmd_vel = rospy.Publisher('/turtlebot/cmd_vel',Twist)		
		self.slave = rospy.Publisher('done',String)						
		self.r = rospy.Rate(20);  #se asegura de mantener el loop a 20 Hz
		self.chatter = SoundClient()

	def avanzaSimulador(self,metros,vel):
		self.inicio = 0
		self.partir = True
		cont = 0.1
		move_cmd = Twist()
		move_cmd.angular.z = 0
		move_cmd.linear.x = vel #m/s
		vel_max = vel
		recorrido = 0
		while (not rospy.is_shutdown()) and (abs(recorrido) <= metros):
			self.cmd_vel.publish(move_cmd)
			self.r.sleep()
			recorrido = self.modulo(self.posx - self.inicioX, self.posy - self.inicioY)
			if self.distance[1] > 0.6:
				cont = min(cont + 0.1, 1)
			else:
				cont = max(cont - 0.1, 0.1)
			if self.distance[0] - self.distance[2] > 0.2:
				move_cmd.angular.z = 1
			elif self.distance[2] - self.distance[0] > 0.2:
				move_cmd.angular.z = -1
			else:
				move_cmd.angular.z = -0.02
			#move_cmd.linear.x = vel_max * cont
			#print(vel_max * cont)
			if (self.distance[1] < 0.5 or self.distance[1] > 90):
				break
		self.parar()
		self.espera(0.7)

	def giraSimulador(self,grados,vel): 
		objetivo = grados
		#zobj = objetivo/180 - 1
		error = 2
		move_cmd = Twist()
		move_cmd.angular.z = vel
		self.partir = True
		thetaReal = True
		#print "voy a girar con theta igual a " + str(self.theta)
		while (not rospy.is_shutdown()) and ((abs(self.theta - objetivo)) > error or thetaReal):
			if not self.partir and thetaReal:
				thetaReal = False
				objetivo += self.theta + error
				#print(self.theta, objetivo)
				if (objetivo > 360):
					objetivo -= 360
				elif (objetivo < 0):
					objetivo += 360
			self.cmd_vel.publish(move_cmd)
			self.r.sleep()
		self.parar()
		self.espera(0.7)

	def gira2(self,grados,vel): 
		objetivo = grados - 5
		#zobj = objetivo/180 - 1
		move_cmd = Twist()
		move_cmd.angular.z = vel
		error = 2
		self.partir = True
		thetaReal = True
		while (not rospy.is_shutdown()) and ((abs(self.theta - objetivo) > error) or thetaReal):
			if not self.partir and thetaReal:
				thetaReal = False
				objetivo += self.theta
				if (objetivo > 360):
					objetivo -= 360
			self.cmd_vel.publish(move_cmd)
			self.r.sleep()
		self.parar()
		self.espera(0.7)

	def enderezar(self,vel):
		move_cmd = Twist()
		move_cmd.angular.z = vel * self.sentidoEnderezado
		print(self.sentidoEnderezado)
		while (not rospy.is_shutdown()):
			self.cmd_vel.publish(move_cmd)
			self.r.sleep()
			print(self.enderezado)
			if (self.enderezado):
				break
		self.parar()
		self.espera(0.7)
		#roboto.chatter.say('Objective located ready for annihilation')

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
#rospy.sleep(1)
	#roboto.chatter.say('ATTACK')
	#rospy.sleep(1)
	#roboto.persigueAmigo()
	#roboto.pasea()
	#roboto.gira(90,1)
	#roboto.gira(90,1)
	#roboto.gira(90,1)
	#roboto.gira(90,1)
	#roboto.enderezar(1)
	#roboto.avanzaPasillo(0.4)
	#roboto.buscaPared()
	rospy.sleep(1)
	rospy.spin()
