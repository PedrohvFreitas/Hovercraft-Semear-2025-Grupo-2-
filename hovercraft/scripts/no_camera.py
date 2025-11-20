#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge
import threading
from collections import deque

global kernel
kernel = np.ones((5, 5), np.uint8)

def hsv(input):
    #função para aplicar a máscara HSV

    #Converte a imagem de RGB para HSV
    hsv = cv2.cvtColor(input, cv2.COLOR_BGR2HSV)

    #Aplica o filtro de cor para a cor vermelha 
    mask = cv2.inRange(hsv,(125, 40, 40), (150, 255, 255))

    return(mask)

def erodir(input):
    #função que realiza erosão da máscara hsv
    img_erodida = cv2.erode(input,kernel,iterations=2)

    return(img_erodida)

def dilatar(input):
    #função que dilata a imagem erodida
    img_dilatada = cv2.dilate(input, kernel, iterations=2)

    return(img_dilatada)

class robo_seguidor:

    def __init__(self):

        rospy.init_node('camera', anonymous=True) 
        #anonymous faz com que o nó tenha nome único

        self.buffer = deque(maxlen = 10)# Tem que ver qual o tamnho ideal
        #cria o buffer que recebe ass imagens da função callback

        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.imagem_callback)
        #pega a imagem do topico do ros

        self.controle_pub = rospy.Publisher('/info_objeto', Float32MultiArray, queue_size=5)
        #publica no topico de controle

        self.bridge = CvBridge()
        #ponte ROS/openCV

        self.imagem_lock = threading.Lock() 
        # lock para evitar conflitos de thread. Acho que isso da certo pq meu computador n tava tankando single threading pra exibir as coisas do gazebo (?)
        #cria a janela com os 6 sliders pra ajustar o HSV, erosão e dilatação em tempo real

    def imagem_callback(self, img_crua):
        try:
            with self.imagem_lock:
                self.buffer.append(self.bridge.imgmsg_to_cv2(img_crua, desired_encoding="bgr8")) 
        except Exception as e:
            rospy.logerr(f"Erro ao converter imagem: {e}")
    #pega a imagem crua, converte pra bgr8 e armazena. Try e except pra caso dessincronize o recebimento de imagens se ficar pesado dms, chatgpt auramaxxing. Sem o threading lock a janela do openccv fica preta
    
if __name__ == '__main__':
    robo = robo_seguidor()
    taxa = rospy.Rate(30)
    #esse rate vai cair dependendo do fps da camera
    while not rospy.is_shutdown():
        # processa eventos do OpenCV no loop principal, pq aparentemente se for fora isso ferra com o gazebo sem motivo algum
        cv2.waitKey(1)

        imagem_para_processar = None
        #Quando o buffer estava vazio o codigo quebrava, pois nada era passado para imagem_para_processar, por isso precisei atribuir um valor inicial á essa variavel
        
        with robo.imagem_lock:
            if robo.buffer:
                imagem_para_processar = robo.buffer.popleft()
        #acessa a imagem pra processar com a "chave"

        if imagem_para_processar is not None:
            # aplica os filtros e exibe as janelas aqui. Sem esse is not none simplesmente nao abre a janela do opencv mt foda
            img_hsv = hsv(imagem_para_processar)
            img_erodida = erodir(img_hsv)
            img_dilatada = dilatar(img_erodida)
            
            #pega a altura e a largura da imagem original
            altura_imagem, largura_imagem, _ = imagem_para_processar.shape
            
            # monta os contornos da imagem binária(pós filtro)
            contornos, _ = cv2.findContours(img_dilatada, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            msg = Float32MultiArray()
            msg.data = [0.0, 0.0, 0.0, float(largura_imagem), float(altura_imagem)]
            #cria a msg
            
            if contornos:
                maior_contorno = max(contornos, key=cv2.contourArea)
                M = cv2.moments(maior_contorno)
                #esse moments é mt foda, basicamente um dicionário de várias informações úteis dos contornos

                if M["m00"] > 0:
                    centro_x_objeto = int(M["m10"] / M["m00"])
                    centro_y_objeto = int(M["m01"] / M["m00"])
                    area_objeto = int(M["m00"])
                    #o centroide é calculado com uma média ponderada que basicamente vira 1/2 ou seja 0.5
                    #esse m00 é a área calculada mt eficientemente pela própria função moments
                    
                    # atualiza a mensagem com os dados do objeto
                    msg.data[0] = float(centro_x_objeto)
                    msg.data[1] = float(centro_y_objeto)
                    msg.data[2] = float(area_objeto)
                    # largura e altura da imagem já estão na mensagem la de cima

            # Sempre publica a mensagem, dentro ou fora do if
            robo.controle_pub.publish(msg)

        taxa.sleep()
        #isso é mt foda, faz o programa relaxar se ele já cumpriu a atividade dentro do ciclo dos 30hz, entao evita que as coisas fiquem dessincronizadas e economiza cpu