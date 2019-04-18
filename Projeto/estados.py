#! /usr/bin/env python
# -*- coding:utf-8 -*-

__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda"]


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt8
from cv_bridge import CvBridge, CvBridgeError
import smach
import smach_ros


# Variável do bumper
bumper = 0

# Variável com a lista de distancias que o scan laser capta
scan_laser = []

# Variável que contem as posições do scan que devem ser checadas
angulo_desvio = 30
numero_scans_direita_frente = np.arange(1, 1 + angulo_desvio, 2)
numero_scans_esquerda_frente = np.arange(360 - angulo_desvio, 360, 2)
numero_scans_direita_tras = np.arange(179 - angulo_desvio, 179, 2)
numero_scans_esquerda_tras = np.arange(181, 181 + angulo_desvio, 2)

# Variável que conta quantos vezez em seguida o scan foi acionado
vezez = 0

# Variáveis angulares
relative_angle = math.radians(45)
angular_speed = math.pi/5

tolerancia_x = 50
tolerancia_y = 20
area_ideal = 60000 # área da distancia ideal do contorno - note que varia com a resolução da câmera
tolerancia_area = 20000

# Atraso máximo permitido entre a imagem sair do Turbletbot3 e chegar no laptop do aluno
atraso = 1.5
check_delay = False # Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados

# Variáveis de checagem
NADA = 0
BUMPER = 1
PSCAN = 2

ESQUERDA = 1
DIREITA = 2

FRENTE = 1
TRAS =2


# Variáveis direcionais
lado = ESQUERDA
direcao = FRENTE

# Variáveis do tempo
sleep_time = 0.02
time_start = 0

# Variável da distancia dos lasers
distancia = 0.4



def scaneou(dado): #função usada pelo subscriber recebe_scan, prenche a lista de distâncias

	global scan_laser
	scan_laser = dado.ranges

def bumper_info(dado): #função usada pelo subscriber recebe_bumper, prenche a lista de distâncias

	global bumper
	bumper = dado.data

def conta_scan():
	global sensores
	global scan_da

	lista_esquerda_frente = []
	lista_direita_frente = []
	lista_esquerda_tras = []
	lista_direita_tras = []

	for n in numero_scans_esquerda_frente:
		if scan_laser[n] != 0:
			lista_esquerda_frente.append(scan_laser[n])

	for n in numero_scans_direita_frente:
		if scan_laser[n] != 0:
			lista_direita_frente.append(scan_laser[n])

	for n in numero_scans_esquerda_tras:
		if scan_laser[n] != 0:
			lista_esquerda_tras.append(scan_laser[n])

	for n in numero_scans_direita_tras:
		if scan_laser[n] != 0:
			lista_direita_tras.append(scan_laser[n])

	if direcao == FRENTE:
		return direcao_scan(lista_esquerda_frente, lista_direita_frente)
	else:
		return direcao_scan(lista_esquerda_tras, lista_direita_tras)


def direcao_scan(lista_esquerda, lista_direita):
	global sensores
	global scan_da

	if (len(lista_esquerda) > 0) and (len(lista_direita) > 0):
		esquerda = min(lista_esquerda)
		direita = min(lista_direita)


		if esquerda < distancia or direita < distancia:
			if esquerda < direita:
				if direcao == FRENTE:
					volta = DIREITA
				else:
					volta = ESQUERDA
				sensores = PSCAN

			elif direita < esquerda:
				if direcao == FRENTE:
					volta = ESQUERDA
				else:
					volta = DIREITA
				sensores = PSCAN

			else:
				volta = None

		else:
			volta = None


	elif (len(lista_esquerda) > 0) or (len(lista_direita) > 0):

		if (len(lista_esquerda) > 0):
			if min(lista_esquerda) < distancia:
				if direcao == FRENTE:
					volta = ESQUERDA
				else:
					volta = DIREITA
				sensores = PSCAN

			else:
				volta = None


		elif (len(lista_direita) > 0):
			if min(lista_direita) < distancia:
				if direcao == FRENTE:
					volta = DIREITA
				else:
					volta = ESQUERDA
				sensores = PSCAN

			else:
				volta = None

		else:
			volta = None

	else:
		volta = None



	return volta



def checa_sensoreres():
	global bumper
	global scan_da
	global sensores

	sensores = 0

	if len(scan_laser) > 0:
		value = conta_scan()

		if value is not None:

			scan_da = value




	if bumper != 0:
		sensores = BUMPER



	





## Classes - estados
class Roda(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['roda', 'obstaculo'])

	def execute(self, userdata):
		global velocidade_saida
		global direcao


		rospy.sleep(sleep_time )
		checa_sensoreres()
		
		
		direcao = FRENTE

		if sensores != 0:
			return 'obstaculo'

		else:
			vel = Twist(Vector3(0.3, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			return 'roda'



		


class Pra_Tras(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['tras', 'obstaculo', 'acabou'])

	def execute(self, userdata):
		global velocidade_saida
		global time_start
		rospy.sleep(sleep_time )
		#checa_sensoreres()

		time_now = rospy.Time.now().to_sec()
		dif = time_now - time_start

		if dif < 1:
			vel = Twist(Vector3(-0.3, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			return 'tras'

		else:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			time_start =rospy.Time.now().to_sec()
			return 'acabou'


class Pra_Frente(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['frente', 'obstaculo', 'acabou'])

	def execute(self, userdata):
		global velocidade_saida
		global time_start
		rospy.sleep(sleep_time )
		#checa_sensoreres()

		time_now = rospy.Time.now().to_sec()
		dif = time_now - time_start

		if dif < 1:
			vel = Twist(Vector3(0.3, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			return 'frente'

		else:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			time_start =rospy.Time.now().to_sec()
			return 'acabou'

		





class Gira(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['obstaculo', 'girando', 'acabou'])

	def execute(self, userdata):
		global velocidade_saida
		checa_sensoreres()
		
		rospy.sleep(sleep_time )

		time_now = rospy.Time.now().to_sec()
		current_angle = angular_speed*(time_now - time_start)

		if sensores == BUMPER:
			return 'obstaculo'

		elif (current_angle < relative_angle) and lado == ESQUERDA:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, angular_speed))
			velocidade_saida.publish(vel)
			return 'girando'

		elif (-current_angle > -relative_angle) and lado == DIREITA:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -angular_speed))
			velocidade_saida.publish(vel)
			return 'girando'

		else:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			return 'acabou'




class Obstaculo(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['girando', 'tras', 'frente'])

	def execute(self, userdata):
		global velocidade_saida
		global time_start
		global bumper
		global lado
		rospy.sleep(sleep_time )

		time_start = rospy.Time.now().to_sec()

		if bumper != 0:

			if bumper == 3:
				bumper = 0
				lado = ESQUERDA
				return 'frente'

			elif bumper == 4:
				bumper = 0
				lado = DIREITA
				return 'frente'

			elif bumper == 1:
				bumper = 0
				lado = DIREITA
				return 'tras'

			elif bumper == 2:
				bumper = 0
				lado = ESQUERDA
				return 'tras'


		else:
			if scan_da == DIREITA:
				lado = ESQUERDA
				return 'girando'
			else:
				lado = DIREITA
				return 'girando'

			 

# main
def main():
	global velocidade_saida
	global buffer

	
	rospy.init_node('estados')
	sm = smach.StateMachine(outcomes=['terminei'])

	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()

	# Para usar a webcam 
	#recebe_imagemr = rospy.Subscriber("/kamera", CompressedImage, roda_todo_frame, queue_size=1, buff_size = 2**24) #subscriber da camera do PC
	#recebe_imagem = rospy.Subscriber("/kamera", CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24) #subscriber da camera do robo 
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou) #subscriber do scan laser
	recebe_bumper = rospy.Subscriber("/bumper", UInt8, bumper_info) #subscriber do bumber

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	# Create a SMACH state machine


	# Open the container
	with sm:
		# Add states to the container
		#smach.StateMachine.add('LONGE', Longe(), 
		#                       transitions={'ainda_longe':'ANDANDO', 
		#                                    'perto':'terminei'})
		#smach.StateMachine.add('ANDANDO', Andando(), 
		#                       transitions={'ainda_longe':'LONGE'})

		smach.StateMachine.add('RODA', Roda(),
								transitions={'obstaculo': 'OBSTACULO',
								'roda':'RODA'})

		smach.StateMachine.add('GIRANDO', Gira(),
								transitions={'girando': 'GIRANDO',
								'acabou':'RODA', 'obstaculo' : 'OBSTACULO'})

		smach.StateMachine.add('OBSTACULO', Obstaculo(),
								transitions={'girando':'GIRANDO', 'tras' : 'TRAS', 'frente' : 'FRENTE'})

		smach.StateMachine.add('TRAS', Pra_Tras(),
								transitions={'tras': 'TRAS',
								'obstaculo':'OBSTACULO', 'acabou' : 'GIRANDO'})

		smach.StateMachine.add('FRENTE', Pra_Frente(),
								transitions={'frente': 'FRENTE',
								'obstaculo':'OBSTACULO', 'acabou' : 'GIRANDO'})



		

	# Execute SMACH plan
	outcome = sm.execute()	
	
	rospy.spin()
	
	sis.stop()


if __name__ == '__main__':
	main()