#!/usr/bin/env python
import rospy 
import cv2 
import numpy as np
import pandas as pd
import gc
import time 
#import Pure_Persuit as Pp

from matplotlib import pyplot as plt
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int16
from std_msgs.msg import Float32
#from lane_detection.msg import Num

global cv2_img
global archivo

class laneDetector :

    def __init__(self):
	
	self.arr = rospy.Publisher('Area_error', Float32, queue_size=1) #Error de area
	self.curve = rospy.Publisher('curve', Float32, queue_size=1) #Curvatura
	self.devi = rospy.Publisher('devi', Float32, queue_size=1) #Desvi
	self.pts1 = rospy.Publisher('Area_centro_robot', Float32, queue_size=1) #puntos medios carril
	self.pts2 = rospy.Publisher('Area_centro_carril', Float32, queue_size=1) #puntos medios carril
        rospy.Subscriber("/app/camera/rgb/image_raw/compressed", CompressedImage, self.imageCallback) 
        self.steering_pub = rospy.Publisher('/AutoNOMOS_mini/manual_control/steering', Int16, queue_size=1)
        self.speed_pub = rospy.Publisher('/AutoNOMOS_mini/manual_control/speed', Int16, queue_size=1)
        
	self.steering_16 = Int16()	
        self.speed_16 = Int16()
        self.speed_16.data = 90
        self.speed = 0
			

		
	

    def mainPrin(self, imgd):
	cv2.waitKey(1)
	self.cv2_img = imgd
	
	archivo = open("Original.txt","w")
	
	lane_img = self.PreProcessImage(self.cv2_img)

        birdView, birdViewL, birdViewR, minverse = self.perspectiveWarp(lane_img)
	img, grayscale, thresh, blur, canny = self.processImage(birdView)
	hist, leftBase, rightBase = self.plotHistogram(thresh)
        ploty, left_fit, right_fit, left_fitx, right_fitx, draw_info = self.slide_window_search(thresh, hist, minverse)
   	
	meanPts, result, mean_x, color_warpL, color_warpR, color_warp_total = self.draw_lane_lines(lane_img, thresh, minverse, draw_info, birdView)
	
	area_left, area_right = self.area(color_warpL, color_warpR, color_warp_total) #Obtener el area de cada lado del carril
	
	#deviation, directionDev = self.offCenter(mean_x, lane_img)
	#curveRad = self.measure_lane_curvature(ploty, left_fitx, right_fitx)
	#del self.cv2_img, birdView, birdViewL, birdViewR, minverse, img, grayscale, thresh, blur, canny, hist, leftBase, rightBase, ploty, left_fit, right_fit, left_fitx, right_fitx, draw_info, meanPts, result, mean_x, color_warpL, color_warpR, color_warp_total, area_left, area_right, deviation, directionDev, curveRad
	



    def imageCallback(self, msg):
        np_arr = np.fromstring(msg.data, np.uint8)  
        self.cv2_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) 
	
	### !!! Main call
	
	start = time.time()

	lane = self.mainPrin (self.cv2_img)
	end = time.time()
	print(end - start)

        self.speed_16.data = self.speed
        self.steering_pub.publish(self.steering_16)
        self.speed_pub.publish(self.speed_16)
        #cv2.imshow("Lane", self.cv2_img)
	#cv2.imwrite("Lane.jpg", self.cv2_img)
	gc.collect()
	
	
	
	
	

	

	
	#gc.collect()

########  PRE-PROCESAMIENTO DE LA IMAGEN DE ENTRADA #################################################
################################################################################
    def PreProcessImage(self, image):
        ##-----------------------------
	    
	#im_in = image[192:480,0:640]
	im_in = image[0:480,0:640]
	#cv2.imshow('Imagen Principal', im_in)	
	#cv2.imwrite('Imagen Principal.jpg', im_in)
	#cv2.imwrite('Imagen Principal Recortada.jpg', im_in)
	#cv2.imwrite('image.jpg',image)
    	self.speed = 0
	self.steering_16 = 90
	return im_in

################################################ #############################
####  FUNCION PARA APLICAR TRANSFORMACION DE PERSPECTIVA ##############################

    def perspectiveWarp(self, im_in):

	#Homografia
	rows = im_in.shape[0]; cols = im_in.shape[1]
	#Seleccionamos cuatro puntos usando un arreglo de numpy
	pts1 = np.float32([[0,300],[0,rows],[cols,300],[cols,rows]])
	#pts1 = np.float32([[0,100],[0,rows],[cols,100],[cols,rows]])
	#Seleccionamos cuatro putnos de destino
	x = 220
	pts2 = np.float32([[0,0],[x,rows],[cols,0],[cols-x,rows]])
	#Se calcula la matriz para la correccion de perspectiva
	M = cv2.getPerspectiveTransform(pts1,pts2)
	#Obtenemos la imagen con correccin de pespectiva
	birdseye = cv2.warpPerspective(im_in, M, (cols,rows))

	#birdseye = birdseye[192:480,0:640] #cambiar tamano de imagen
	#cv2.imshow("res0",im_ins)
    	#Matriz inversa para deshacer la imagen de la ventana final
    	minv = cv2.getPerspectiveTransform(pts2, pts1)

#	inpImage = im_in
	# Obtener el tamano de la imagen
#    	img_size = (inpImage.shape[1], inpImage.shape[0])


    	# Puntos de perspectiva a deformar
#    	src = np.float32([[125, 109],
#                      [488, 115],
#                      [6, 189],
#                      [588, 201]])
#    	src = np.float32([[127, 297],
#                      [411, 297],
#                      [3, 438],
#                      [608, 438]])

#    	# Ventana que se muestra
#    	dst = np.float32([[0, 0],
#                      [inpImage.shape[1], 0],
#                     [0, inpImage.shape[0]],
#                      [inpImage.shape[1], inpImage.shape[0]]])

    	# Matriz para deformar la imagen de la ventana de ojo de pajaro
#    	matrix = cv2.getPerspectiveTransform(src, dst)
    	# Matriz inversa para deshacer la imagen de la ventana final
#    	minv = cv2.getPerspectiveTransform(dst, src)
	
#    	birdseye = cv2.warpPerspective(inpImage, matrix, img_size)

    	# Obtener las dimensiones de la ventana de ojo de pajaro
    	height, width = birdseye.shape[:2]

    	# Dividir la vista de pajaro en 2 mitades para separar los carriles izquierdo y derecho
    	birdseyeLeft  = birdseye[0:height, 0:width // 2]
    	birdseyeRight = birdseye[0:height, width // 2:width]

    	# Mostrar las diferentes imagenes de vista de pajaro
    	#cv2.imshow("Birdseye" , birdseye)
    	#cv2.imshow("Birdseye Left" , birdseyeLeft)
    	#cv2.imshow("Birdseye Right", birdseyeRight)
	#cv2.imwrite('Bird.jpg',birdseye)
 	#print(img_size)
	return birdseye, birdseyeLeft, birdseyeRight, minv


################################################ #############################
#### INICIO - FUNCION PARA PROCESAR LA IMAGEN ###################################### #

    def processImage(self,inpImage):


    # Se convierte la imagen a escala de grises, aplica el umbral, se difumina y se extrae los bordes
        gray = cv2.cvtColor(inpImage, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray, 160, 255, cv2.THRESH_BINARY)
        blur = cv2.GaussianBlur(thresh,(3, 3), 0)
        canny = cv2.Canny(blur, 40, 60)

    # Muestra las imagenes procesadas
        #cv2.imshow("Image Original", inpImage)
        #cv2.imshow("Grayscale", gray)
        #cv2.imshow("Thresholded", thresh)
        #cv2.imshow("Blurred", blur)
        #cv2.imshow("Canny Edges", canny)
	#cv2.imwrite('Gray.jpg',gray)
	#cv2.imwrite('Thresh.jpg',thresh)
	#cv2.imwrite('blur.jpg',blur)
	#cv2.imwrite('Canny.jpg',canny)        
	return inpImage, gray, thresh, blur, canny


#### INCIO - FUNCION PARA TRAZAR EL HISTOGRAMA DE IMAGEN DEFORMADA ###################
    def plotHistogram(self, thresh):
	
	inpImage = thresh

        histogram = np.sum(inpImage[inpImage.shape[0] // 2:, :], axis = 0)

        midpoint = np.int(histogram.shape[0] / 2)
        leftxBase = np.argmax(histogram[:midpoint])
        rightxBase = np.argmax(histogram[midpoint:]) + midpoint
	
	#plt.figure(1)
	#plt.xlabel("Image X Coordinates")
        #plt.ylabel("Number of White Pixels")
	#plt.plot(histogram)
	#plt.show()
	#cv2.imshow("his",inpImage)

        return histogram, leftxBase, rightxBase
   

#### INICIO - APLICAR EL METODO DE VENTANA CORREDIZA PARA DETECTAR CURVAS #####################
    def slide_window_search(self, binary_warped, histogram, Minv):

	#binary_warped = binary_warped[250:480,0:640]	
	#cv2.imshow('binary', binary_warped)

    # Encuentre el inicio de las lineas de carril izquierdo y derecho usando la informacion del histograma
        out_img = np.dstack((binary_warped, binary_warped, binary_warped)) * 255 #Combinacion de profundidad: combinacion a lo largo del eje longitudinal.
	       
	midpoint = np.int(histogram.shape[0] / 2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    # Se utilizan un total de 9 ventanas
        nwindows = 9
        window_height = np.int(binary_warped.shape[0] / nwindows)
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        leftx_current = leftx_base
        rightx_current = rightx_base
        margin = 50
        minpix = 10
        left_lane_inds = []
        right_lane_inds = []

   
	#### INICIO - Recorre en iteracion las ventanas y busca lineas de carril #####
        for window in range(nwindows):
            win_y_low = binary_warped.shape[0] - (window + 1) * window_height
            win_y_high = binary_warped.shape[0] - window * window_height
     	    win_xleft_low = leftx_current - margin
   	    win_xleft_high = leftx_current + margin
    	    win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
    	    cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high),
    	    (0,255,0), 2)
     	    cv2.rectangle(out_img, (win_xright_low,win_y_low), (win_xright_high,win_y_high),
    	    (0,255,0), 2)
   	    good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
   	    (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
    	    good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
    	    (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]
    	    left_lane_inds.append(good_left_inds)
    	    right_lane_inds.append(good_right_inds)
    	    if len(good_left_inds) > minpix:
    	        leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
    	    if len(good_right_inds) > minpix:
     	        rightx_current = np.int(np.mean(nonzerox[good_right_inds]))
    #### Termina bucle para recorrer las ventanas y buscar lineas de carril #######

        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]
   
    
#	if len(lefty) == 0:
#	   self.arr.publish(15000)
	   
#  	Aplicar ajuste polinomial de segundo grado para ajustar curvas
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)
	
#	global left_fit2
#	global right_fit2
	
#	left_fit2 = left_fit
#	right_fit2 = right_fit

        ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0])
        left_fitx = left_fit[0] * ploty**2 + left_fit[1] * ploty + left_fit[2]
        right_fitx = right_fit[0] * ploty**2 + right_fit[1] * ploty + right_fit[2]

		

        ltx = np.trunc(left_fitx)
        rtx = np.trunc(right_fitx)
        #plt.plot(right_fitx)
        
        ret = {}
        ret['leftx'] = leftx
        ret['rightx'] = rightx
        ret['left_fitx'] = left_fitx
        ret['right_fitx'] = right_fitx
        ret['ploty'] = ploty

        out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
        out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

	#plt.figure(1)    	
	#plt.imshow(out_img) #Ventanas deslizantes
        #plt.plot(left_fitx,  ploty, color = 'yellow')
        #plt.plot(right_fitx, ploty, color = 'red')
        #plt.xlim(0, 640)
        #plt.ylim(288, 0)
	#plt.show()
	#cv2.imshow('Ventana_deslizante',out_img)
	#cv2.imwrite('vent.jpg',out_img)
        return ploty, left_fit, right_fit, ltx, rtx, ret





#### INICIA - FUNCION PARA MOSTRAR VISUALMENTE EL AREA DE CARRILES DETECTADOS ####################
    def draw_lane_lines(self, original_image, warped_image, Minv, draw_info, trans):
		
	copy = original_image
        left_fitx = draw_info['left_fitx']
        right_fitx = draw_info['right_fitx']
        ploty = draw_info['ploty']
	

#	min_ld = 200	
#	max_ld = 480

#	left_fitx = left_fitx[min_ld:max_ld]
#	right_fitx = right_fitx[min_ld:max_ld]
#	ploty = ploty [min_ld:max_ld]

	cv2.imshow("imaged",warped_image)
	#cv2.imwrite("imaged.jpg",warped_image)
	#warped_image = warped_image[200:480,0:640]
	#original_image = original_image[200:480,0:640]
	#cv2.imshow("aw",warped_image)
        warp_zero = np.zeros_like(warped_image).astype(np.uint8) #Devuelve una matriz de ceros con la misma forma y tipo que la matriz dada.
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
	color_warpL = np.dstack((warp_zero, warp_zero, warp_zero))
	color_warpR = np.dstack((warp_zero, warp_zero, warp_zero))
	color_warp_total = np.dstack((warp_zero, warp_zero, warp_zero))

        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        #pts = np.hstack((pts_left, pts_right))

        mean_x = np.mean((left_fitx, right_fitx), axis=0) #Puntos medios del carril
	




		

        pts_mean = np.array([np.flipud(np.transpose(np.vstack([mean_x, ploty])))])
	pts_meant = np.array([np.flipud(np.transpose(np.vstack([mean_x[::-1], ploty[::-1]])))])

	
	#pts_center = np.hstack((mean_x)) #Nuevos puntos
	mean_xT = mean_x-(320-mean_x[np.size(mean_x)-1]) #Linea central auto
	mean_xTT = mean_x+(320-mean_x[np.size(mean_x)-1]) #Linea central auto
	#
	#plt.plot(mean_xT, color = 'green')	
	
	#self.pts1.publish(mean_x[np.size(mean_x)-1])
	#self.pts2.publish(mean_xTT[np.size(mean_x)-1])


	#print(np.size(mean_xT))
	#print(mean_x)
	mean_xT = np.array([np.flipud(np.transpose(np.vstack([mean_xT[::-1], ploty[::-1]])))])
	mean_xTT = np.array([np.flipud(np.transpose(np.vstack([mean_xTT[::-1], ploty[::-1]])))])
	
    	area_left = np.hstack((pts_left, pts_mean)) #Area izquierda 
   	area_right = np.hstack((pts_meant, pts_right[::-1])) #Area derecha
	area_total = np.hstack((mean_xT, pts_right[::-1])) #Area total

	#cv2.line(color_warp,(320,0),(320,480),(255,100,255),10)
    	t = np.array([320])
    	for i in range (0,479):
       	   t1=([320])
       	   t = np.concatenate((t,t1), axis= 0)
    	
	points = np.array([np.flipud(np.transpose(np.vstack([t, ploty])))])
    	pointdos = np.array([np.flipud(np.transpose(np.vstack([t, ploty[::-1]])))])

	
	#if (320 >= mean_x[np.size(mean_x)-1] > 0):
	   #area_left = np.hstack((pts_mean, mean_xT))
	   #area_right = [0] #Area izquierda De otra forma area
	   #cv2.fillPoly(color_warp, np.int_([area_left]), (0, 0, 255)) ##De otra forma area
	   #color_warp_total = 1
	   #print("izq")
	#if (640 > mean_x[np.size(mean_x)-1] > 320):
	   #area_left = np.hstack((pts_meant, points)) #Area izquierda De otra forma area
	   #cv2.fillPoly(color_warp, np.int_([area_total]), (0, 0, 255)) ##De otra forma area
	   #color_warp_total = -1
	   #print("dre")
	#print(mean_x[np.size(mean_x)-1])
	
	#area_left = np.hstack((pts_mean, mean_xTT)) #Area izquierda De otra forma area		
	#area_right = np.hstack((pointdos, pts_right[::-1])) #Area derecha
	#pts = pts[0:10]
#    	cv2.fillPoly(color_warpL, np.int_([area_left]), (0, 0, 255))
#    	cv2.fillPoly(color_warpR, np.int_([area_right]), (0,0 , 255))
#	cv2.fillPoly(color_warp_total, np.int_([area_total]), (0,0 , 255))
	
	
	#cv2.fillPoly(color_warpL, np.int_([area_left]), (0, 0, 255))
	#area_left = np.hstack((pts_mean, mean_xTT)) #Area izquierda De otra forma area
    	#cv2.fillPoly(color_warp, np.int_([area_left]), (0, 255, 0))
    	#cv2.polylines(color_warp, np.int_([pts_mean]), False, (255, 0, 0),2)
    	#cv2.polylines(color_warp, np.int_([mean_xTT]), False, (255, 255, 0),2)

	##imagen en vista pajaro
	#cv2.fillPoly(warped_image, np.int_([pts]), (0, 255, 255))
    	#cv2.polylines(warped_image, np.int_([pts_mean]), False, (255, 0, 255),2)
	#cv2.imshow("pj",warped_image)
	###


    	#new_color_warpL = cv2.cvtColor(color_warpL, cv2.COLOR_BGR2RGB)
    	#areas = cv2.add(new_color_warpL,color_warpR)
	#pts_meanr = np.array([np.flipud(np.transpose(np.vstack([mean_x, ploty])))])        

	#cv2.polylines(areas, np.int_([pts_meanr]), False, (255, 255, 0),2)
    	#cv2.imshow('Areas',areas)
	#cv2.imwrite('Areas.jpg',areas)
    	#cv2.imshow('Area desde el centro del carril', color_warpR)
	#cv2.imwrite('Area desde el centro del carril.jpg', color_warpR)
    	#cv2.imshow('Area desde el centro del auto', color_warp_total)
	#cv2.imwrite('Area desde el centro del auto.jpg', color_warp_total)
	#Fusionar imagenes-areas
	#RR = cv2.cvtColor(color_warpR, cv2.COLOR_RGB2BGR)
	#cv2.imwrite("rr.jpg",RR)
	#fusion = cv2.addWeighted(RR , 50, color_warp_total, 0.3, 10)
	#cv2.imshow('Blended Image',fusion)
	#cv2.imwrite('Blended Image.jpg',fusion)
	#fusion = fusion[0:480,320:640]
	#cv2.imshow('Blended Imagesd',fusion)



	
	#####Formar Carril central a partir de un solo lado######
	
	puntos_der = right_fitx-153
	puntos_izq = np.array([np.flipud(np.transpose(np.vstack([puntos_der[::-1], ploty[::-1]])))]) ##Puntos iqz
	pts_mit = np.mean((puntos_der, right_fitx), axis=0) #Puntos medios del carril
	pts_medio = np.array([np.flipud(np.transpose(np.vstack([pts_mit, ploty])))])
	pts_medioT = np.array([np.flipud(np.transpose(np.vstack([pts_mit[::-1], ploty[::-1]])))])
	pts_movil = pts_mit+(320-pts_mit[np.size(pts_mit)-1]) #Linea central auto
	pts_movil = np.array([np.flipud(np.transpose(np.vstack([pts_movil[::-1], ploty[::-1]])))])
	#pts = np.hstack((pts_right , pts_control))
	#print("centro",pts_control)
	#print("izq", right_fitx)
	area_left = np.hstack((pts_left, pts_medio)) #Area izquierda 
   	area_right = np.hstack((pts_medioT, pts_right[::-1])) #Area derecha
	area_total = np.hstack((pts_movil, pts_right[::-1])) #Area total
	error = np.hstack((pts_movil, pts_medio)) #Area total
	#cv2.polylines(color_warp, np.int_([puntos_izq]), False, (255, 0, 0),4)
	#cv2.polylines(color_warp, np.int_([pts_right]), False, (255, 0, 255),4)
    	cv2.fillPoly(color_warp, np.int_([error]), (0,255 , 0))
	cv2.polylines(color_warp, np.int_([pts_medio]), False, (255, 0, 0),4)
	cv2.polylines(color_warp, np.int_([pts_movil]), False, (0, 0, 255),4)

	cv2.fillPoly(color_warpL, np.int_([area_left]), (0, 0, 255))
    	cv2.fillPoly(color_warpR, np.int_([area_right]), (0,0 , 255))
	cv2.fillPoly(color_warp_total, np.int_([area_total]), (0,0 , 255))
	#cv2.fillPoly(color_warp, np.int_([area_total]), (0,255 , 0))
	#cv2.fillPoly(color_warp, np.int_([area_right]), (0,0 , 255))

	
	#########################################################
        result2 = cv2.addWeighted(trans, 1, color_warp, 0.3, 0)	
	cv2.imshow('IMAGE FINALa', result2)
	cv2.imwrite('erroarea.jpg',result2)
        newwarp = cv2.warpPerspective(color_warp, Minv, (original_image.shape[1], original_image.shape[0]))
        result = cv2.addWeighted(original_image, 1, newwarp, 0.3, 0)
        cv2.imshow('IMAGE FINAL', result)
        #cv2.imshow('color_warpL', color_warpL)
        #cv2.imshow('color_warpR', color_warpR)
	cv2.imwrite('Finalareaizq.jpg', result)
	
	#plt.figure(1)   
	#plt.plot(mean_x, color = 'yellow')
	#plt.xlim(0, 480)
        #plt.ylim(640,0 ) 	
        #
        #plt.plot(left_fitx, color = 'red')
	#plt.plot(right_fitx, color = 'red')

	#plt.show()
	#cv2.imshow('Ventana_deslizante',out_img)
	#cv2.imwrite('vent.jpg',out_img)

        return pts_mean, result, mean_x, color_warpL, color_warpR,color_warp_total

####Funcion para encontrar el area de cada lado del carril###
###############################################################
    def area(self, color_warpL,color_warpR, color_warp_total):

	#color_warpL = color_warpL[200:480,0:640]
	#color_warpR = color_warpR[200:480,0:640]
	#color_warp_total = color_warp_total[200:480,0:640]

	color_warpL = cv2.cvtColor(color_warpL, cv2.COLOR_BGR2GRAY)
	color_warpR = cv2.cvtColor(color_warpR, cv2.COLOR_BGR2GRAY)
        color_warp_total = cv2.cvtColor(color_warp_total, cv2.COLOR_BGR2GRAY)

	
	retL , threshL = cv2.threshold(color_warpL,50,255,0) #Izq
	retR , threshR = cv2.threshold(color_warpR,50,255,0) #Der
    	retT , threshT = cv2.threshold(color_warp_total,50,255,0) #Total
	    
	_, contoursL,hierarchL = cv2.findContours(threshL, 1, 1)
	_, contoursR,hierarchR = cv2.findContours(threshR, 1, 2)
	_, contoursT,hierarchT = cv2.findContours(threshT, 1, 2)
	
	cntL = contoursL[0]
	cntR = contoursR[0]
	cntT = contoursT[0]

	self.Area_left =  cv2.contourArea(cntL)
	self.Area_right = cv2.contourArea(cntR)
	self.Area_total = cv2.contourArea(cntT)
	#self.Area_right = 0

		
	#Area_left2 = self.Area_left
	#Area_total2 = self.Area_total	
	#print("Area izq: ", ((self.Area_left*0.4)/(220*220)))
	#print("Area der: ", ((self.Area_right*0.4)/(220*220)))
    	#print("Area Total: ", self.Area_total)
    	
	#data = self.Area_right-self.Area_total
	#df = pd.DataFrame(data, columns = ['first_name'])
	#df.to_LibreOffice('example.xlsx', sheet_name='example')
	
	#np.savetxt('Original.txt', data)
	
	#self.pts1.publish(((self.Area_total*0.4)/(220*220))) #Area centro robot
	#self.pts2.publish(((self.Area_left*0.4)/(220*220))) #area centro carril
	#error = (self.Area_left)*((0.4)/(220*220))*color_warp_total #Error de area
	error = (self.Area_right-self.Area_total)*((0.6*0.4)/(480*160))

	#if self.Area_left == 0:
	#error = error*(-1)
	self.arr.publish(error) #publicacion area
	print("Error",error)
	#archivo = open("Original.txt","w")
	#archivo.write(",")
	#archivo.write('%s'%error)
	#archivo.write(",")
	
	#plt.figure(1)    	
	#plt.imshow(color_warpR)
	#plt.show()
	
	cv2.imshow('color_warpT', color_warp_total)
	cv2.imshow('color_warpL_', color_warpL)
	cv2.imshow('color_warpR_', color_warpR)
	return self.Area_left, self.Area_right

#### Funcion encontrar desviacion del centro al robot ##################
################################################################################
    def offCenter(self, meanPts, inpFrame):

    	# Calculating deviation in meters
	#meanPts = meanPts[::1]
    	mpts = meanPts[np.size(meanPts)-1].astype(int)
    	pixelDeviation = inpFrame.shape[1] / 2 - abs(mpts)
    	deviation = (pixelDeviation * 0.4)/220
    	direction = "left" if deviation < 0 else "right"
	
	#print("deviation", deviation)
	self.devi.publish(deviation)
	#print(direction)
    	return deviation, direction



    def measure_lane_curvature(self, ploty, leftx, rightx):
    	leftx = leftx[::-1]  # Reverse to match top-to-bottom in y
    	rightx = rightx[::-1]  # Reverse to match top-to-bottom in y
	
	ym_per_pix = (0.4/220)
	xm_per_pix = (0.4/220)
	
    # Choose the maximum y-value, corresponding to the bottom of the image
    	y_eval = np.max(ploty)

    # Fit new polynomials to x, y in world space
    	left_fit_cr = np.polyfit(ploty*ym_per_pix, leftx*xm_per_pix, 2)
    	right_fit_cr = np.polyfit(ploty*ym_per_pix, rightx*xm_per_pix, 2)

    # Calculate the new radii of curvature
    	left_curverad  = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0]) 
	right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
    # Now our radius of curvature is in meters
        #print(left_curverad, 'm', right_curverad, 'm')
	self.curve.publish(left_curverad)


    	return (left_curverad + right_curverad) / 2.0


if __name__ == '__main__':
    rospy.init_node('lanedetection', anonymous = True)
    rospy.loginfo("Init Lane detection")
    ld = laneDetector()
    r = rospy.Rate(10)
	
    try:
        rospy.spin()
        r.sleep()
    except rospy.ROSInterruptException:
        pass


##-----------------------------
