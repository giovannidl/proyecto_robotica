#!/usr/bin/env python
import rospy
import math
import random

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


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
		self.left = msj.data[0] == '1'
		self.right = msj.data[1] == '1'
		self.distance = int(msj.data[2:])

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
		self.left = False
		self.distance = 0

		#Inicializar el nodo y suscribirse/publicar
		rospy.init_node('roboto', anonymous=True) #make node 
   		rospy.Subscriber('odom',Odometry,self.odometryCb)
		rospy.Subscriber('obstaculo',String,self.obstaculo)
		self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist)						
		self.r = rospy.Rate(20);  #se asegura de mantener el loop a 20 Hz

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
			if self.distance > 60:
				cont = min(cont + 0.1, 1)
			else:
				cont = max(cont - 0.1, 0.1)
			move_cmd.linear.x = vel_max * cont
			print(vel_max * cont)
			if (self.right or self.left):
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
		while (not rospy.is_shutdown()) and ((abs(self.theta - objetivo) > error) or thetaReal):
			if not self.partir and thetaReal:
				thetaReal = False
				objetivo += self.theta
				if (objetivo > 360):
					objetivo -= 360
			self.cmd_vel.publish(move_cmd)
			self.r.sleep()
			if self.distance > 60:
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

if __name__ == "__main__":
	roboto = Nodo()
	roboto.pasea()
	#rospy.spin()
