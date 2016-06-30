#-*- coding: utf-8 -*-

import numpy as np
import argparse
import cv2

'''
ap = argparse.ArgumentParser()
ap.add_argument('-i','--image',help = 'Lab6_imagenes/doblarIzq')
args = vars(ap.parse_args())
'''
temple = cv2.imread('Lab6_imagenes/doblarDer.jpg',cv2.IMREAD_GRAYSCALE)
image = cv2.imread('Lab6_imagenes/nodoblarIzq.jpg')
image2 = cv2.imread('Lab6_imagenes/doblarIzq.jpg')
crop_img = image[10:180, 20:180]
crop_img2 = temple[20:180, 20:180]
cv2.imshow("cropped", crop_img)
cv2.waitKey(0)
'''
lower = np.array([0,0,0])
upper = np.array([15,15,15])
shapeMask = cv2.inRange(crop_img,lower,upper)
shapeMask2 = cv2.inRange(temple,0,10)


(cnts, _) = cv2.findContours(shapeMask2.copy(), cv2.RETR_EXTERNAL,
cv2.CHAIN_APPROX_SIMPLE)
print "I found %d black shapes" % (len(cnts))
cv2.imshow("Mask", shapeMask2)
cv2.imshow('mako',shapeMask)
 
# loop over the contours

for c in cnts:
	#cv2.drawContours(crop_img2, [c], -1, (0, 255, 0), 2)
	#cv2.imshow("Image", temple)
	#cv2.waitKey(0)
	pass
w, h = temple.shape[::-1]
res = cv2.matchTemplate(shapeMask2,shapeMask,cv2.TM_SQDIFF)
min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
if (min_val < 530000):#izq=530000(42916500.0)=der noder=6200000=noizq
	print('found')
top_left = min_loc
bottom_right = (top_left[0] + w, top_left[1] + h)
cv2.rectangle(crop_img2,top_left, bottom_right, (255,0,0) , 2)
cv2.imshow('blarg',temple)
cv2.waitKey(0)
print(min_val)

'''
