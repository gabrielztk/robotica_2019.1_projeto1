#! /usr/bin/env python
# -*- coding:utf-8 -*-

__author__ = ["Beatriz Mie", "Gabriel Zanetti"]


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
from imutils.video import VideoStream
from imutils.video import FPS
import argparse



import cormodule as cm
import mobilenet_simples as mb

bridge = CvBridge()

cv_image = None

(major, minor) = cv2.__version__.split(".")[:2]


# Classes da Mobilenet
CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
	"bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
	"dog", "horse", "motorbike", "person", "pottedplant", "sheep",
	"sofa", "train", "tvmonitor"]

alvo = "cat"


# Variáveis para permitir que o roda_todo_frame troque dados com a máquina de estados
media = []
centro = []
area = 0.0

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


# Variáveis lineares lin_max = 0.22
linear_speed = 0.3
linear_speed_tracking = 0

# Variáveis angulares ang_max = 2.84
relative_angle = math.radians(45)
angular_speed = math.pi/5
angular_speed_gira = math.pi/5

tolerancia_x = 50
tolerancia_y = 20
area_ideal = 20000 # área da distancia ideal do contorno - note que varia com a resolução da câmera
area_minima = 10000
tolerancia_area = 20000

# Atraso máximo permitido entre a imagem sair do Turbletbot3 e chegar no laptop do aluno
atraso = 0.5
check_delay = True # Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados

# Variáveis de checagem
NADA = 0
BUMPER = 1
PSCAN = 2

ESQUERDA = 1
DIREITA = 2

FRENTE = 1
TRAS =2

TRACKING = 1

AZUL = 0
VERMELHO = 1

# Variáveis direcionais
lado = ESQUERDA
direcao = FRENTE

# Variáveis do tempo
sleep_time = 0.02
time_start = 0

# Variável da distancia dos lasers
distancia = 0.2

# Variável das cores
area_azul = None
media_azul = None
area_vermelho = None
media_vermelho = None

# Variável da mobilenet
results = []
contador = 0

# Variável da tracking
tracking = NADA

# Funções usadas pelos Subscribers

def roda_todo_frame(imagem): #função usada pelo subscriber recebe_imagem
	global cv_image
	global centro
	global media_vermelho
	global area_vermelho
	global media_azul
	global area_azul
	global results
	global tracking
	global contador
	global centro_tracking
	global fps
	global initBB
	global area_tracking
	global WIDTH
	global HEIGTH

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime
	delay = lag.secs

	if delay > atraso and check_delay==True:
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		centro,media_vermelho, area_vermelho, media_azul, area_azul = cm.identifica_cor(cv_image)

		WIDTH, HEIGTH, g = cv_image.shape

		if tracking == NADA:

			results = mb.detect(cv_image, alvo)

		else:
			tracking_update()

		if len(results) > 0:
			contador += 1
		else:
			contador = 0

		if contador > 3:
			tracking = TRACKING

			pi_x = results[0][2][0]
			pi_y = results[0][2][1]
			pf_x = results[0][3][0]
			pf_y = results[0][3][1]

			w = pf_x - pi_x
			h = pf_y - pi_y

			centro_x = pi_x + w/2
			centro_y = pi_y + h/2

			area_tracking = w*h

			centro_tracking = (centro_x, centro_y)

			initBB = (pi_x, pi_y, w, h)
			tracker.init(cv_image, initBB)
			fps = FPS().start()
			results = []
			tracking_update()

		depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)


def tracking_update():
	global tracker
	global tracking
	global fps
	global cv_image
	global box_tracking
	global centro_tracking
	global area_tracking

	fps.update()
	fps.stop()

	success, box_tracking = tracker.update(cv_image)

	# check to see if the tracking was a success
	if success:
		(x, y, w, h) = [int(v) for v in box_tracking]
		cv2.rectangle(cv_image, (x, y), (x + w, y + h),
			(0, 255, 0), 2)

		centro_x = x + w/2
		centro_y = y + h/2

		centro_tracking = (centro_x, centro_y)
		cm.cross(cv_image, centro_tracking, [255,0,0], 1, 17)
		area_tracking = w*h
		

	else:
		tracker.clear()
		tracking = NADA
		if int(major) == 3 and int(minor) < 3:
			tracker = cv2.Tracker_create(args["tracker"].upper())

		else:
			tracker = OPENCV_OBJECT_TRACKERS[args["tracker"]]()




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


def velocidade_linear(area_im, cor):
	global linear_speed_tracking
	
	if cor == VERMELHO:
		area_max = WIDTH*HEIGTH*0.75
		m = -0.22/(area_max - area_ideal)
		b = -m*area_max
		vel = (m*area_im + b)
		linear_speed_tracking = (vel)

	else:
		area_max = WIDTH*HEIGTH*0.75
		m = -0.22/(area_max - area_ideal)
		b = -m*area_max
		vel = (m*area_im + b)
		linear_speed_tracking = (vel)




def velocidade_angular(diferenca):
	global angular_speed


	m = 1.6/(WIDTH/2)
	b = -m*0
	ang = m*diferenca + b
	angular_speed = ang
	print(ang)



def mobilenet_alvo():
	global alvo
	print("Você deve selecionar um tipo de objeto da seguinte lista: ")
	print(CLASSES)
	alvo = raw_input("Insira sua escolha: ")


## Classes - estados
class Roda(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['roda', 'obstaculo', 'tracking'])

	def execute(self, userdata):
		global velocidade_saida
		global direcao
		global results
		global contador
		global tracker
		global tracking
		global fps
		global results
		global cv_image

		rospy.sleep(sleep_time )
		checa_sensoreres()
		
		
		direcao = FRENTE

		if sensores != 0:
			return 'obstaculo'

		elif area_azul > area_ideal or area_vermelho > area_ideal or tracking == TRACKING:
			return 'tracking'

		else:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
			velocidade_saida.publish(vel)
			return 'roda'


class Tracking(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['roda', 'obstaculo', 'tracking'])

	def execute(self, userdata):
		global velocidade_saida
		global direcao
		global angular_speed
		rospy.sleep(sleep_time )
		checa_sensoreres()
		


		if sensores != 0:
			return 'obstaculo'

		elif tracking == TRACKING:
			direcao =FRENTE
			dif = math.fabs(centro[0] - centro_tracking[0])
			velocidade_angular(dif)

			

			if  math.fabs(centro_tracking[0]) > math.fabs(centro[0] + tolerancia_x):
				vel = Twist(Vector3(linear_speed, 0, 0), Vector3(0, 0, angular_speed))
				velocidade_saida.publish(vel)
				print('tracking esquerda')
				return 'roda'

			if math.fabs(centro_tracking[0]) < math.fabs(centro[0] - tolerancia_x):
				vel = Twist(Vector3(linear_speed, 0, 0), Vector3(0, 0, -angular_speed))
				velocidade_saida.publish(vel)
				print('tracking direita')
				return 'roda'

			else:
				vel = Twist(Vector3(linear_speed, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				print('tracking frente')
				return 'roda'

		elif area_azul > area_vermelho:
			direcao = TRAS
			dif = math.fabs(centro[0] - media_azul[0])
			velocidade_linear(area_azul, AZUL)
			velocidade_angular(dif)
			

			if  math.fabs(media_azul[0]) > math.fabs(centro[0] + tolerancia_x):
				vel = Twist(Vector3(-linear_speed, 0, 0), Vector3(0, 0, -(angular_speed)))
				velocidade_saida.publish(vel)
				print('azul esquerda')
				return 'roda'

			if math.fabs(media_azul[0]) < math.fabs(centro[0] - tolerancia_x):
				vel = Twist(Vector3(-linear_speed, 0, 0), Vector3(0, 0, (angular_speed)))
				velocidade_saida.publish(vel)
				print('azul direita')
				return 'roda'

			else:
				vel = Twist(Vector3(-linear_speed, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				print('tracking frente')
				return 'roda'

		elif area_azul < area_vermelho:
			direcao =FRENTE
			dif = math.fabs(centro[0] - media_vermelho[0])
			velocidade_linear(area_vermelho, VERMELHO)
			velocidade_angular(dif)
			if linear_speed_tracking <= 0.1:
				angular_speed = 0

			if  math.fabs(media_vermelho[0]) > math.fabs(centro[0] + tolerancia_x):
				vel = Twist(Vector3(linear_speed_tracking, 0, 0), Vector3(0, 0, (angular_speed)))
				velocidade_saida.publish(vel)
				print('vermelho esquerda')
				return 'roda'

			if math.fabs(media_vermelho[0]) < math.fabs(centro[0] - tolerancia_x):
				vel = Twist(Vector3(linear_speed_tracking, 0, 0), Vector3(0, 0, -(angular_speed)))
				velocidade_saida.publish(vel)
				print('vermelho direita')
				return 'roda'
			else:
				vel = Twist(Vector3(linear_speed_tracking, 0, 0), Vector3(0, 0, 0))
				velocidade_saida.publish(vel)
				print('tracking frente')
				return 'roda'


		else:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
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
			vel = Twist(Vector3(-linear_speed, 0, 0), Vector3(0, 0, 0))
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
			vel = Twist(Vector3(linear_speed, 0, 0), Vector3(0, 0, 0))
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
		current_angle = angular_speed_gira*(time_now - time_start)

		if sensores == BUMPER:
			return 'obstaculo'

		elif (current_angle < relative_angle) and lado == ESQUERDA:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, angular_speed_gira))
			velocidade_saida.publish(vel)
			return 'girando'

		elif (-current_angle > -relative_angle) and lado == DIREITA:
			vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -angular_speed_gira))
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
	global tracker
	global OPENCV_OBJECT_TRACKERS
	global args

	# Seleciona o alvo
	mobilenet_alvo()

	# construct the argument parser and parse the arguments
	ap = argparse.ArgumentParser()
	ap.add_argument("-v", "--video", type=str,
		help="path to input video file")
	ap.add_argument("-t", "--tracker", type=str, default="kcf",
		help="OpenCV object tracker type")
	args = vars(ap.parse_args())

	# extract the OpenCV version info
	(major, minor) = cv2.__version__.split(".")[:2]

	# if we are using OpenCV 3.2 OR BEFORE, we can use a special factory
	# function to create our object tracker
	if int(major) == 3 and int(minor) < 3:
		tracker = cv2.Tracker_create(args["tracker"].upper())

	# otherwise, for OpenCV 3.3 OR NEWER, we need to explicity call the
	# approrpiate object tracker constructor:
	else:
		# initialize a dictionary that maps strings to their corresponding
		# OpenCV object tracker implementations
		OPENCV_OBJECT_TRACKERS = {
			"csrt": cv2.TrackerCSRT_create,
			"kcf": cv2.TrackerKCF_create,
			"boosting": cv2.TrackerBoosting_create,
			"mil": cv2.TrackerMIL_create,
			"tld": cv2.TrackerTLD_create,
			"medianflow": cv2.TrackerMedianFlow_create,
			"mosse": cv2.TrackerMOSSE_create
		}

		# grab the appropriate object tracker using our dictionary of
		# OpenCV object tracker objects
		tracker = OPENCV_OBJECT_TRACKERS[args["tracker"]]()
	
	rospy.init_node('estados')
	sm = smach.StateMachine(outcomes=['terminei'])

	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()

	recebe_imagemr = rospy.Subscriber("/kamera", CompressedImage, roda_todo_frame, queue_size=1, buff_size = 2**24) #subscriber da camera do PC
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou) #subscriber do scan laser
	recebe_bumper = rospy.Subscriber("/bumper", UInt8, bumper_info) #subscriber do bumber

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	# Create a SMACH state machine


	# Open the container
	with sm:
		# Add states to the container

		smach.StateMachine.add('RODA', Roda(),
								transitions={'obstaculo': 'OBSTACULO',
								'roda':'RODA', 'tracking' : 'TRACKING'})

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

		smach.StateMachine.add('TRACKING', Tracking(),
								transitions={'obstaculo': 'OBSTACULO',
								'roda':'RODA', 'tracking' : 'TRACKING'})


		

	# Execute SMACH plan
	outcome = sm.execute()	
	
	rospy.spin()
	
	sis.stop()


if __name__ == '__main__':
	main()