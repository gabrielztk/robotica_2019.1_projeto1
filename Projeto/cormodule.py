#! /usr/bin/env python
# -*- coding:utf-8 -*-


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import smach
import smach_ros


def cross(img_rgb, point, color, width,length):
        cv2.line(img_rgb, (point[0] - length/2, point[1]),  (point[0] + length/2, point[1]), color ,width, length)
        cv2.line(img_rgb, (point[0], point[1] - length/2), (point[0], point[1] + length/2),color ,width, length) 


def identifica_cor(image):
	frame = image.copy()
	'''
	Segmenta o maior objeto cuja cor é parecida com cor_h (HUE da cor, no espaço HSV).
	'''

	# No OpenCV, o canal H vai de 0 até 179, logo cores similares ao 
	# vermelho puro (H=0) estão entre H=-8 e H=8. 
	# Precisamos dividir o inRange em duas partes para fazer a detecção 
	# do vermelho:
	frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


	# COR VERMELHA
	cor_menor_vermelho = np.array([0, 50, 50])
	cor_maior_vermelho = np.array([8, 255, 255])
	segmentado_cor_vermelho = cv2.inRange(frame_hsv, cor_menor_vermelho, cor_maior_vermelho)

	#cor_menor_vermelho = np.array([172, 50, 50])
	#cor_maior_vermelho = np.array([180, 255, 255])
	#segmentado_cor_vermelho += cv2.inRange(frame_hsv, cor_menor_vermelho, cor_maior_vermelho)


	# A operação MORPH_CLOSE fecha todos os buracos na máscara menores 
	# que um quadrado 7x7. É muito útil para juntar vários 
	# pequenos contornos muito próximos em um só.
	segmentado_cor_vermelho = cv2.morphologyEx(segmentado_cor_vermelho,cv2.MORPH_CLOSE,np.ones((7, 7)))

	# Encontramos os contornos na máscara e selecionamos o de maior área
	#contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)	
	img_out_vermelho, contornos_vermelho, arvore_vermelho = cv2.findContours(segmentado_cor_vermelho.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 

	maior_contorno_vermelho = None
	maior_contorno_area_vermelho = 0

	for cnt in contornos_vermelho:
	    area = cv2.contourArea(cnt)
	    if area > maior_contorno_area_vermelho:
	        maior_contorno_vermelho = cnt
	        maior_contorno_area_vermelho = area

	# Encontramos o centro do contorno fazendo a média de todos seus pontos.
	if maior_contorno_vermelho is not None :
	    cv2.drawContours(frame, [maior_contorno_vermelho], -1, [0, 0, 255], 5)
	    maior_contorno_vermelho = np.reshape(maior_contorno_vermelho, (maior_contorno_vermelho.shape[0], 2))
	    media_vermelho = maior_contorno_vermelho.mean(axis=0)
	    media_vermelho = media_vermelho.astype(np.int32)
	    cross(frame, tuple(media_vermelho), [0,255,0], 1, 17)
	    #cv2.circle(frame, tuple(media_vermelho), 5, [0, 255, 0])
	else:
	    media_vermelho = (0, 0)


	# COR AZUL
	cor_menor_azul = np.array([110, 50, 50])
	cor_maior_azul = np.array([130, 255, 255])
	segmentado_cor_azul = cv2.inRange(frame_hsv, cor_menor_azul, cor_maior_azul)

	#cor_menor_azul = np.array([172, 50, 50])
	#cor_maior_azul = np.array([180, 255, 255])
	#segmentado_cor_azul += cv2.inRange(frame_hsv, cor_menor_azul, cor_maior_azul)


	# A operação MORPH_CLOSE fecha todos os buracos na máscara menores 
	# que um quadrado 7x7. É muito útil para juntar vários 
	# pequenos contornos muito próximos em um só.
	segmentado_cor_azul = cv2.morphologyEx(segmentado_cor_azul,cv2.MORPH_CLOSE,np.ones((7, 7)))

	# Encontramos os contornos na máscara e selecionamos o de maior área
	#contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)	
	img_out_azul, contornos_azul, arvore_azul = cv2.findContours(segmentado_cor_azul.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 

	maior_contorno_azul = None
	maior_contorno_area_azul = 0

	for cnt in contornos_azul:
	    area = cv2.contourArea(cnt)
	    if area > maior_contorno_area_azul:
	        maior_contorno_azul = cnt
	        maior_contorno_area_azul = area

	# Encontramos o centro do contorno fazendo a média de todos seus pontos.
	if maior_contorno_azul is not None :
	    cv2.drawContours(frame, [maior_contorno_azul], -1, [255, 0, 0], 5)
	    maior_contorno_azul = np.reshape(maior_contorno_azul, (maior_contorno_azul.shape[0], 2))
	    media_azul = maior_contorno_azul.mean(axis=0)
	    media_azul = media_azul.astype(np.int32)
	    cross(frame, tuple(media_azul), [0,255,0], 1, 17)
	    #cv2.circle(frame, tuple(media_azul), 5, [0, 255, 0])
	else:
	    media_azul = (0, 0)

	# Representa a area e o centro do maior contorno no frame
	font = cv2.FONT_HERSHEY_COMPLEX_SMALL

	cv2.imshow('Cores', frame)
	cv2.waitKey(1)

	centro = (frame.shape[1]//2, frame.shape[0]//2)

	return centro, media_vermelho, maior_contorno_area_vermelho, media_azul, maior_contorno_area_azul