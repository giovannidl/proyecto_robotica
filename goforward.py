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

	def amigo(self,msj):
		self.objetivo = msj.data.split(':')
		if len(self.objetivo) < 3:
			self.objetivo.append(0)
		self.objetivo = map(float,self.objetivo)
			

		# Vemos si el objetivo sigue en el  centro
		if self.objetivo[0] < 280:
			self.pierdeObjetivo = True
			self.sentidoObjetivo = 1
		elif self.objetivo[0] > 360:
			self.pierdeObjetivo = True
			self.sentidoObjetivo = -1
		else:
			self.pierdeObjetivo = False

		# Vemos si se alcanzo el objetivo
		if self.objetivo[2] < 0.5:
			self.alcanceObjetivo = True
		else:
			self.alcanceObjetivo = False

	def enderezame(self,data):
		self.distsPared = data.data.split(';')
		self.distsPared = map(float,self.distsPared)
		if max(abs(self.distsPared[0] - self.distsPared[1]), abs(self.distsPared[1] - self.distsPared[2])) < 0.03 and self.distsPared[0] != 0:
			self.enderezado = True
		else:
			self.enderezado = False
		aux = self.distsPared[2] - self.distsPared[0]
		if aux != 0:
			self.sentidoEnderezado = (aux) / abs(aux)

	def solve(self,msg):
		todo = msg.data.split('#')
		self.ocupado = True
		for accion in todo:
			if accion == "Go":
				self.chatter.say("Avanza")
				self.avanza(self.largoPared,0.4)
			elif accion == "Left":
				self.chatter.say("Gira izquierda")
				self.gira(90,-1)
			else:
				self.chatter.say("Gira derecha")
				self.gira(90,1)
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
		self.distance = [0,0,0]
		self.objetivo = [0,0,0]
		self.pierdeObjetivo = False
		self.sentidoObjetivo = 1
		self.alcanceObjetivo = False
		self.distsPared = [0,0,0]
		self.enderezado = False
		self.sentidoEnderezado = 1
		self.ocupado = False
		self.largoPared = 0.8

		#Inicializar el nodo y suscribirse/publicar
		rospy.init_node('roboto', anonymous=True) #make node 
   		rospy.Subscriber('odom',Odometry,self.odometryCb)
		rospy.Subscriber('obstaculo',String,self.obstaculo)
		rospy.Subscriber('amigoFiel',String,self.amigo)
		rospy.Subscriber('enderezador3',String,self.enderezame)
		rospy.Subscriber('todo',String,self.solve)
		self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist)						
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
		while (not rospy.is_shutdown()) and (abs(recorrido) < metros*(1-self.cl)):
			self.cmd_vel.publish(move_cmd)
			self.r.sleep()
			recorrido = self.modulo(self.posx - self.inicioX, self.posy - self.inicioY)
			if min(self.distance) > 0.6:
				cont = min(cont + 0.1, 1)
			else:
				cont = max(cont - 0.1, 0.1)
			move_cmd.linear.x = vel_max * cont
			print(vel_max * cont)
			if (min(self.distance) < 0.5 or max(self.distance) > 90):
				break
		self.parar()
		self.espera(0.7)

	def avanzaAmigo(self,metros,vel):
		self.inicio = 0
		self.partir = True
		cont = 0.1
		move_cmd = Twist()
		move_cmd.angular.z = -0.02
		move_cmd.linear.x = cont * vel #m/s
		vel_max = vel
		recorrido = 0
		while (not rospy.is_shutdown()) and (abs(recorrido) < metros*(1-self.cl)):
			self.cmd_vel.publish(move_cmd)
			self.r.sleep()
			recorrido = self.modulo(self.posx - self.inicioX, self.posy - self.inicioY)
			if self.objetivo[2] > 0.8:
				cont = min(cont + 0.1, 1)
			else:
				cont = max(cont - 0.1, 0.1)
			move_cmd.linear.x = vel_max * cont
			print(vel_max * cont)
			if (self.alcanceObjetivo):
				break
			elif (min(self.distance) < 0.5 or max(self.distance) > 90):
				break
		self.parar()
		self.espera(0.7)

	def gira(self,grados,vel): 
		objetivo = grados - 5
		#zobj = objetivo/180 - 1
		move_cmd = Twist()
		move_cmd.angular.z = vel
		error = 2
		self.partir = True
		thetaReal = True
		print('voy a girar')
		while (not rospy.is_shutdown()) and ((abs(self.theta - objetivo) > error) or thetaReal):
			if not self.partir and thetaReal:
				thetaReal = False
				objetivo += self.theta
				if (objetivo > 360):
					objetivo -= 360
			self.cmd_vel.publish(move_cmd)
			self.r.sleep()
			if (vel > 0 and not self.right) or (vel < 0 and not self.left):
				break
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
		print('voy a girar')
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

	def giraAmigo(self,grados,vel): 
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
			if not self.pierdeObjetivo:
				break
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

	def persigueAmigo(self):
		while (not rospy.is_shutdown()):
			if self.pierdeObjetivo:
				self.giraAmigo(10000, self.sentidoObjetivo)
			elif not self.center:
				self.avanzaAmigo(10,0.3)
			else:
				print("No puedo llegar")

	def enderezar(self,vel):
		move_cmd = Twist()
		move_cmd.angular.z = vel * self.sentidoEnderezado
		print(self.sentidoEnderezado)
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
