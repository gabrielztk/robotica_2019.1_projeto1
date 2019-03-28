#! /usr/bin/env python
# -*- coding:utf-8 -*-


import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt8


INICIO = 0
BUMPER_FRENTE = 1
BUMPER_TRAS = 2

relative_angle = math.radians(60) # ângulo para a curva
angular_speed = math.pi/5 # velocidade da curva


distancia_lista = [] #lista com as 360 distâncias medidas pelo scan laser

bumper = 0 #valor de 1-4 que indica qual bumper foi acionado por último 


def scaneou(dado): #função usada pelo subscriber recebe_scan, prenche a lista de distâncias

	global distancia_lista
	distancia_lista = dado.ranges


def bumper_info(dado): #função usada pelo subscriber recebe_bumper, prenche a lista de distâncias

	global bumper
	bumper = dado.data

	


if __name__=="__main__":

	rospy.init_node("bumper_bate")

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 ) #publisher da velocidade do robo
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou) #subscriber do scan laser
	recebe_bumper = rospy.Subscriber("/bumper", UInt8, bumper_info) #subscriber do bumber


	ESTADO = INICIO #seleciona o que fazer dentro do loop

	while not rospy.is_shutdown(): #LOOP principal

		
		
		if ESTADO == INICIO: #estado inicial -- Checa se algum bumper foi cionado, se não, continua pra frente

			if bumper == 1 or bumper == 2: #checa se os bumpers dianteiros foram precionados
				print(str(bumper) + " INICIO_1")
				ESTADO = BUMPER_FRENTE
				PARTE = 0
				
				if bumper == 1:
					relative_angle = -math.radians(60)
					angular_speed = -math.pi/5
					lado_a = 0
				elif bumper == 2:
					relative_angle = math.radians(60)
					angular_speed = math.pi/5
					lado_a = 1
				bumper = 0

			elif bumper == 3 or bumper == 4: #checa se os bumpers traseiros foram precionados
				print(str(bumper) + " INICIO_2")
				ESTADO = BUMPER_TRAS
				bumper = 0

			else: #se nenhum foi presionado, continua pra frente
				print(str(bumper) + " INICIO_3")
				vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0))
				velocidade_saida.publish(vel)

		elif ESTADO == BUMPER_FRENTE:

			

			if PARTE == 0 and ESTADO == BUMPER_FRENTE:
				t0 = rospy.Time.now().to_sec()
				RODA = True
				while RODA:
					if bumper == 0:
						print(str(bumper) + " FRENTE_1.1")
						vel = Twist(Vector3(-0.1,0,0), Vector3(0,0,0))
						velocidade_saida.publish(vel)
						delta_t = (rospy.Time.now().to_sec() - t0)
						if delta_t > 3:
							RODA = False

						rospy.sleep(0.02)
					else:
						print(str(bumper) + " FRENTE_1.2")
						ESTADO = INICIO
						RODA = False
				bumper = 0
						



				PARTE = 1

			elif PARTE == 1 and ESTADO == BUMPER_FRENTE: #curva 

				current_angle = 0
				t0 = rospy.Time.now().to_sec()

				RODA = True
				while RODA:
					if bumper == 0:
						print(str(bumper) + " FRENTE_2.1")
						vel = Twist(Vector3(0,0,0), Vector3(0,0,angular_speed))
						velocidade_saida.publish(vel)
						t1 = rospy.Time.now().to_sec()
						current_angle = angular_speed*(t1-t0)
						delta_a = (current_angle - relative_angle)
						if (lado_a == 0 and delta_a < 0) or (lado_a == 1 and delta_a > 0):
							RODA = False

						rospy.sleep(0.02)
					else:
						print(str(bumper) + " FRENTE_2.2")
						ESTADO = INICIO
						RODA = False
				bumper = 0
				ESTADO = INICIO
						

		elif ESTADO == BUMPER_TRAS:
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
			velocidade_saida.publish(vel)
			ESTADO = INICIO
			print(str(bumper) + " TRAS")


		rospy.sleep(0.5)


		




		