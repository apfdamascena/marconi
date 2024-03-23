import math
import RPi.GPIO as GPIO
from time import sleep
import time
from robot_hardware import RobotHardware
from encoder import Encoder
import csv


counter = 10

encoderA = Encoder(RobotHardware.RIGHT_ENCODED_FRONT, RobotHardware.RIGHT_ENCODED_BACK)
encoderB = Encoder(RobotHardware.LEFT_ENCODED_FRONT, RobotHardware.LEFT_ENCODED_BACK)


data = []

def init(left_power, right_power, direction_left="F", direction_right="F"):

    global encoder

    print ("Rotary Encoder Test Program")
    GPIO.setwarnings(True)
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(RobotHardware.LEFT_ENABLE, GPIO.OUT)
    GPIO.setup(RobotHardware.RIGHT_ENABLE, GPIO.OUT)

    GPIO.setup(RobotHardware.LEFT_BACKWARD, GPIO.OUT)
    GPIO.setup(RobotHardware.LEFT_FORWARD, GPIO.OUT)

    GPIO.setup(RobotHardware.RIGHT_BACKWARD,   GPIO.OUT)
    GPIO.setup(RobotHardware.RIGHT_FORWARD, GPIO.OUT)

    if direction_left == "F":
        GPIO.output(RobotHardware.LEFT_BACKWARD,   GPIO.LOW)
        GPIO.output(RobotHardware.LEFT_FORWARD, GPIO.HIGH)
    elif direction_left == "B":
        GPIO.output(RobotHardware.LEFT_BACKWARD, GPIO.HIGH)
        GPIO.output(RobotHardware.LEFT_FORWARD, GPIO.LOW)

    if direction_right == "F":
        GPIO.output(RobotHardware.RIGHT_BACKWARD, GPIO.LOW)
        GPIO.output(RobotHardware.RIGHT_FORWARD, GPIO.HIGH)
    elif direction_right == "B":
        GPIO.output(RobotHardware.RIGHT_BACKWARD, GPIO.HIGH)
        GPIO.output(RobotHardware.RIGHT_FORWARD, GPIO.LOW)

    pwm_right = GPIO.PWM(RobotHardware.RIGHT_ENABLE, 1000)
    pwm_left = GPIO.PWM(RobotHardware.LEFT_ENABLE, 1000)


    pwm_right.start(0)
    pwm_left.start(0)

    pwm_right.ChangeDutyCycle(right_power)
    pwm_left.ChangeDutyCycle(left_power)

    encoderA.run()
    encoderB.run()

    idx = 0

    while encoderA.running.value:
        time.sleep(0.2)

        ul = -encoderB.get_w()
        ur = encoderA.get_w()

        if idx >= 500:
            encoderA.stop()
            break

        idx += 20

        print(idx)


        wrobotl = -RobotHardware.RAY * ul / ( RobotHardware.DISTANCE_WHEELS)
        wrobotr = RobotHardware.RAY * ur / ( RobotHardware.DISTANCE_WHEELS)

        wrobot = wrobotl + wrobotr


        vrobotl = RobotHardware.RAY * ul / 2
        vrobotr = RobotHardware.RAY * ur / 2

        vrobot = (vrobotl + vrobotr)

        if direction_left == "F" and direction_right == "F":
            data.append([time.time(), vrobot, wrobot, ul, ur])
        elif direction_left == "B" and direction_right == "B":
            data.append([time.time(), vrobot, wrobot, -ul, -ur])
        elif direction_left == "F" and direction_right == "B":
            data.append([time.time(), vrobot, wrobot, ul, -ur])
        elif direction_left == "B" and direction_right == "F":
            data.append([time.time(), vrobot, wrobot, -ul, ur])

    with open(f'./asterix-data/data_left_{left_power}_{direction_left}_right_{right_power}_{direction_right}_robot.csv', 'w') as arquivo_csv:
        writter = csv.writer(arquivo_csv)
        writter.writerow(['Timestamp', 'Velocidade linear', 'Velocidade angular', 'Velocidade Roda Esquerda', 'Velocidade Roda Direita'])
        for row in data:
            writter.writerow(row)

if __name__ == '__main__':
    direction_left = input('Digite a direção da roda esquerda [F, B]:')
    direction_right = input('Digite a direção da roda direita [F, B]:')
    left_power = int(input('Digite quantos porcentos da roda esquerda [0-100]:'))
    right_power = int(input('Digite quantos porcentos da roda direita [0-100]:'))
    init(left_power, right_power, direction_left, direction_right)