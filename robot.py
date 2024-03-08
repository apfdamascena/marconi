import math
import RPi.GPIO as GPIO
from time import sleep
import time
from robot_hardware import RobotHardware



# ENTRADAS SE FOREM CONFIGURADAS COMO SAIDAS QUEIMAM!!!

Enc_A = RobotHardware.LEFT_ENCODED_FRONT
Enc_B = RobotHardware.LEFT_ENCODED_BACK

counter = 10

def init():
    print ("Rotary Encoder Test Program")
    GPIO.setwarnings(True)
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(RobotHardware.LEFT_ENABLE, GPIO.OUT)
    GPIO.setup(RobotHardware.RIGHT_ENABLE, GPIO.OUT)


    GPIO.setup(RobotHardware.LEFT_ENCODED_FRONT, GPIO.IN)
    GPIO.setup(RobotHardware.LEFT_ENCODED_BACK, GPIO.IN)

    GPIO.setup(RobotHardware.LEFT_BACKWARD, GPIO.OUT)
    GPIO.setup(RobotHardware.LEFT_FORWARD, GPIO.OUT)

    GPIO.setup(RobotHardware.RIGHT_BACKWARD,   GPIO.OUT)
    GPIO.setup(RobotHardware.RIGHT_FORWARD, GPIO.OUT)


    GPIO.output(RobotHardware.RIGHT_BACKWARD,   GPIO.LOW)
    GPIO.output(RobotHardware.LEFT_BACKWARD,   GPIO.LOW)

    GPIO.output(RobotHardware.LEFT_FORWARD, GPIO.HIGH)
    GPIO.output(RobotHardware.RIGHT_FORWARD, GPIO.HIGH)

    pwm_r = GPIO.PWM(RobotHardware.RIGHT_ENABLE, 1000)
    pwm_l = GPIO.PWM(RobotHardware.LEFT_ENABLE, 1000)


    pwm_r.start(0)
    pwm_l.start(0)


    pwm_r.ChangeDutyCycle(60)
    pwm_l.ChangeDutyCycle(100)

    GPIO.add_event_detect(Enc_A, GPIO.RISING, callback=rotation_decode, bouncetime=5)

    while True:
        sleep(1)

    return


# This is a callback function that can be used when events happen

sampling_time = 0.05
last_time = time.time()

delta_theta = 0


def rotation_decode(Enc_A):

    global counter
    global delta_theta
    global last_time

    sleep(0.004)
    Switch_A = GPIO.input(Enc_A)
    Switch_B = GPIO.input(Enc_B)


    if (Switch_A == 1) and (Switch_B == 0):
        counter += 1

        step_angle = 2*math.pi/12
        reduction = 38
        delta_theta += step_angle/reduction
        now = time.time()
        if (now - last_time) >= sampling_time:
            dt = now-last_time
            print ("ANGULO: ", delta_theta)
            print ("VELOCIDADE ANGULAR: ", delta_theta/dt)

            last_time = now
            delta_theta = 0
        print("direction -> ", counter)
        while Switch_B == 0:
            Switch_B = GPIO.input(Enc_B)
        while Switch_B == 1:
            Switch_B = GPIO.input(Enc_B)
        return

    elif (Switch_A == 1) and (Switch_B == 1):
        counter -= 1
        print("direction <- ", counter)
        while Switch_A == 1:
            Switch_A = GPIO.input(Enc_A)
        return
    else:
        return

def main():
    try:
        init()
        while True :
            sleep(1)

    except KeyboardInterrupt:
        GPIO.cleanup()

if __name__ == '__main__':
    main()