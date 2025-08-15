#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

detected_estate = 0

def imagem_callback(cam_image):

    global detected_estate

    try:
        frame = bridge.imgmsg_to_cv2(cam_image, desired_encoding='bgr8')

    except CvBridgeError as e:
        rospy.logerr(e)
        return

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (125, 40, 40), (150, 255, 255))#Ajustar para a cor que for selecionada. Penso em roxo ou laranja

    kernel = np.ones((5, 5), np.uint8)
    img_erosao = cv2.erode(mask,kernel,iterations=2)
    img_tratada = cv2.dilate(img_erosao, kernel, iterations=2)
    
    M = cv2.moments(img_tratada)

    altura, largura, _ = frame.shape
    
    min = altura * largura * 0.120

    cmd = Twist()

    cv2.imshow('Video', frame)
    cv2.waitKey(1)


    if(M['m00'] > min):
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])


        if detected_estate == 0:
            rospy.loginfo("Objeto detectado")
            detected_estate = 1
        
        #Mexer nessa parte do controle dps, fazer o PID e sofisticar a forma como o robo anda
        centro_tela = largura/2

        erro = Float64()

        erro.data = cx - centro_tela

        pub_distx.publish(erro)

        cmd.angular.z = -0.005 * erro.data #Ajustar o valor da multiplicação 
        cmd.linear.x = 1.0

        pub_vel.publish(cmd)
    
    else:

        if detected_estate == 1:
            rospy.loginfo("Objeto não detectado")
            detected_estate = 0

        cmd.linear.x = 0.0
        cmd.angular.z = 1.0

        pub_vel.publish(cmd)

rospy.init_node('follow_object')
rospy.loginfo("No follow_object foi iniciado")

pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
pub_distx = rospy.Publisher("/distancia_centro", Float64, queue_size=10)
sub_im = rospy.Subscriber("hovercraft/camera/image_raw", Image, callback= imagem_callback)

bridge = CvBridge()

cv2.destroyAllWindows()

rospy.spin()


