#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Float64

PINO_ESC = 17 #numero do pino da ESC, Não sei qual é
FREQ_PWM = 50

GPIO.setmode(GPIO.BCM)
GPIO.setup(PINO_ESC, GPIO.OUT)

pwm_esc = GPIO.PWM(PINO_ESC, FREQ_PWM)
pwm_esc.start(0)


def pwm_callback(percentage_vel_motor):
    dc_min = 0.05 #motor desligado
    dc_max = 0.1 #motor a 100%

    dc = (dc_min + ((dc_max - dc_min) * percentage_vel_motor)) * 100

    pwm_esc.ChangeDutyCycle(dc)


def desligar_motor():
    print("\nDesligando o motor e limpando os pinos GPIO.")
    pwm_esc.stop()
    GPIO.cleanup()

sub_motor_vel = rospy.Subscriber("/control_node", Float64,callback = pwm_callback)

rospy.on_shutdown(desligar_motor)

rospy.spin()




