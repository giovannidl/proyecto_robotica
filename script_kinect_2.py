#-*- coding: utf-8 -*-

import rospy
import roslib
import numpy

from sensor_msgs.msg import Image
from std_msgs.msg import String

from matplotlib import pyplot as plt

import cv2
from cv_bridge import CvBridge, CvBridgeError

face = cv2.imread('Lab6_imagenes/averageman.jpg',cv2.IMREAD_GRAYSCALE)
key = cv2.imread('Lab6_imagenes/llave.jpg',cv2.IMREAD_GRAYSCALE)
door = cv2.imread('Lab6_imagenes/puerta.jpg',cv2.IMREAD_GRAYSCALE)
yLeft = cv2.imread('Lab6_imagenes/doblarIzq.jpg',cv2.IMREAD_GRAYSCALE)
yRight = cv2.imread('Lab6_imagenes/doblarDer.jpg',cv2.IMREAD_GRAYSCALE)
nLeft = cv2.imread('Lab6_imagenes/nodoblarIzq.jpg',cv2.IMREAD_GRAYSCALE)
nRight = cv2.imread('Lab6_imagenes/nodoblarDer.jpg',cv2.IMREAD_GRAYSCALE)
templates = [face, key, door]

class Turtlebot_Kinect(object):
    def __init__(self):
        self.__depth_img = rospy.Subscriber('/camera/depth/image',Image ,self.__depth_handler)
        self.__depth_img = rospy.Subscriber('/camera/depth/image_raw',Image ,self.__depth_handler)
        self.__rgb_img= rospy.Subscriber('/camera/rgb/image_color',Image,self.__rgb_handler)#original rgb/image, no hace nada
        self.obs = rospy.Publisher('obstaculo',String)
        self.amigo = rospy.Publisher('amigoFiel',String)
        self.dis2 = rospy.Publisher('enderezador3',String)
        self.watcher = rospy.Publisher('watchRoboto',String)
        self.bridge = CvBridge()
        self.current_cv_depth_image = numpy.zeros((1,1,3))
        self.current_cv_rgb_image = numpy.zeros((1,1,3))
        self.lowGreen = numpy.array([60,50,50])
        self.upGreen = numpy.array([80,255,2500])
        self.objectiveX = 0
        self.objectiveY = 0

    def __depth_handler(self, data):
        try:
            self.current_cv_depth_image = numpy.asarray(self.bridge.imgmsg_to_cv(data,"32FC1"))
            #rospy.loginfo("imagen depth recibida " + str(self.current_cv_depth_image.shape))
            mensaje = self.revisaImagen(self.current_cv_depth_image)
            objetivo = self.buscaAmigo(self.current_cv_depth_image)
            enderezadores = self.tomaPuntos(self.current_cv_depth_image)
            self.obs.publish(mensaje)
            self.amigo.publish(objetivo)
            self.dis2.publish(enderezadores)
            #rospy.loginfo('1: '+str(self.current_cv_depth_image[320,240]))
            #print mensaje
            
        except CvBridgeError, e:
            print e

    def __rgb_handler(self, data):
        try:
            self.current_cv_rgb_image = numpy.asarray(self.bridge.imgmsg_to_cv(data,"bgr8"))
            #rospy.loginfo("imagen rgb recibida " + str(self.current_cv_rgb_image.shape))
            '''
            # concateno ambas imágenes sólo para visualización
            I = self.current_cv_rgb_image
            D = numpy.zeros((I.shape[0], I.shape[1],3), numpy.uint8)
            if self.current_cv_depth_image.shape[:2] == self.current_cv_rgb_image.shape[:2]:
                rospy.loginfo("imagen concatenada")
                D[:,:,0] = self.current_cv_depth_image*20 # sólo para visualización
                D[:,:,1] = D[:,:,0]
                D[:,:,2] = D[:,:,1]
            
            cv2.imshow("image_rgb",numpy.concatenate((I,D),axis=1))
            cv2.waitKey(10)
            '''
            imagen1 = cv2.cvtColor(self.current_cv_rgb_image, cv2.COLOR_BGR2GRAY)
            imagen = cv2.blur(imagen1,(10,10))
            #interes = cv2.inRange(imagen, self.lowGreen, self.upGreen)
            ans = self.reconocer(imagen1)
            self.watcher.publish(':'.join(map(str,ans)))
            #print('Y-L',ans[3]==1,'Y-R',ans[4]==1,'N-L',ans[5]==1,'N-R',ans[6]==1)
            '''
            cv2.imshow('filtro', interes)
            cv2.waitKey(10)
            contours, hierarchy = cv2.findContours(interes, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            max_area = 0
            if len(contours) != 0:
                index_moments = 0
                max_area = 0
                Contour = 0
                for cnt in contours:
                    moments = cv2.moments(cnt)
                    area = moments['m00']
                    if area > max_area:
                        index_moments = moments
                        max_area = area
                        Contour = cnt
            
            #self.objectiveX = (numpy.uint32)(index_moments['m10']/max_area)
            #self.objectiveY = (numpy.uint32)(index_moments['m01']/max_area)
            
            if max_area > 3350:
                self.objectiveX = (numpy.uint32)(index_moments['m10']/max_area)
                self.objectiveY = (numpy.uint32)(index_moments['m01']/max_area)
            else:
                self.objectiveX = -1
                self.objectiveY = -1
            
            '''
        except CvBridgeError, e:
            print e

    def revisaImagen(self, depth):
        #res = '00'
        aux = []
        for i in range(5):
            middle = depth[238+i]
            up = depth[98+i]
            down = depth[378+i]
            aux.append(middle[:200])
            aux.append(up[:200])
            aux.append(down[:200])
        num_izq = self.superMaxi(aux)
        aux = []
        for i in range(5):
            middle = depth[238+i]
            up = depth[98+i]
            down = depth[378+i]
            aux.append(middle[240:270])
            #aux.append(up[180:380])
            #aux.append(down[240:270])
        num_cent_izq = self.superMaxi(aux)
        aux = []
        for i in range(5):
            middle = depth[238+i]
            up = depth[98+i]
            down = depth[378+i]
            aux.append(middle[280:360])
            #aux.append(up[180:380])
            #aux.append(down[300:340])
        num_cent = self.superMaxi(aux)
        aux = []
        for i in range(5):
            middle = depth[238+i]
            up = depth[98+i]
            down = depth[378+i]
            aux.append(middle[350:420])
            #aux.append(up[180:380])
            #aux.append(down[370:400])
        num_cent_der = self.superMaxi(aux)
        aux = []
        for i in range(5):
            middle = depth[238+i]
            up = depth[98+i]
            down = depth[378+i]
            aux.append(middle[430:560])
            aux.append(up[430:560])
            aux.append(down[430:560])
        num_der = self.superMaxi(aux)
        '''
        obs_cen = num_cent < 0.4 or num_cent > 90
        obs_izq = num_izq < 0.4 or num_izq > 90
        obs_der = num_der < 0.4 or num_der > 90

        if (obs_cen or (obs_der and obs_izq)):
            res = "11"
        elif (obs_izq):
            res = "10"
        elif (obs_der):
            res = "01"
        res += str(int(100*min(num_cent, num_izq, num_der)))
        '''
        aux = str(num_izq) + ':' + str(num_cent) + ':' + str(num_der) + ':' + str(num_cent_izq) + ':' + str(num_cent_der)
        return aux

    def maxi(self,lista):
        res = 100
        for i in range(len(lista)):
            if str(lista[i]) != "nan" and lista[i] < res:
                res = lista[i]
        if res == 100:
            res = 0
        return res

    def superMaxi(self, listas):
        res = 100
        for lista in listas:
            aux = self.maxi(lista)
            if aux < res:
                res = aux
        return res

    def buscaAmigo(self,depths):
		if self.objectiveX > 0:
			res = str(self.objectiveX) + ':' + str(self.objectiveY) + ':' + str(depths[self.objectiveY][self.objectiveX])
		else:
			res = str(self.objectiveX) + ':' + str(self.objectiveY) + ':' + '100'
		return res

    def tomaPuntos(self,depths):
		lineas = depths[238:243]
		puntoInicio = 100
		puntoMedio = 100
		puntoFinal = 100
		for linea in lineas:
			puntoInicio = min(puntoInicio, min(linea[90:95]))
			puntoMedio = min(puntoMedio, min(linea[318:323]))
			puntoFinal = min(puntoFinal, min(linea[530:535]))
		puntoInicio = str(puntoInicio)
		puntoMedio = str(puntoMedio)
		puntoFinal = str(puntoFinal)
		
		#if (punto1 == 'nan'):
		#	punto1 = '100'
		#if (punto2 == 'nan'):
		#	punto2 = '100'

		msj = puntoInicio+';'+puntoMedio+';'+puntoFinal
		return msj
    def reconocer(self, imagen, n = 0):
#        print('in')
        ans = [0,0,0,0,0,0,0]
        for i in range(len(templates)):
            #print(temple.shape)#,imagen.type())
            temple = templates[i]
            w, h = temple.shape[::-1]
            #temple = cv2.cvtColor(temple, cv2.COLOR_BGR2GRAY)
            res = cv2.matchTemplate(imagen,temple,cv2.TM_SQDIFF)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
            '''
            if (i == 1):
                top_left = min_loc
                bottom_right = (top_left[0] + w, top_left[1] + h)
                cv2.rectangle(imagen,top_left, bottom_right, (255,0,0) , 2)
                cv2.imshow('blarg',imagen)
                cv2.waitKey(10)
                print(min_val)
            '''
            if (i == 0 and min_val < 360000000):
                ans[0] = 1
            elif (i == 1 and min_val < 420000000):
                ans[1] = 1
            elif (i == 2 and min_val < 350000000):
                ans[2] = 1
            '''
            elif (i == 3 and min_val < 280000000):
                ans[3] = 1
            elif (i == 4 and min_val < 250000000):
                ans[4] = 1
            elif (i == 5 and min_val < 250000000):
                ans[5] = 1
            elif (i == 6 and min_val < 250000000):
                ans[6] = 1
            '''
        if (sum(ans) > 1 and n < 10):
            ans = self.reconocer(imagen, n+1)
            #elif (i == 0):
            #    print('AverageMan',min_val, min_loc)
            #else:
            #    print('Puerta',min_val, min_loc)
        return ans

if __name__ == '__main__':
    rospy.init_node("test_move_action_client")

    handler = Turtlebot_Kinect()
    
    rospy.spin()
    

