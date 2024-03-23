import math
import RPi.GPIO as GPIO
from time import sleep
import time
from robot_hardware import RobotHardware
from encoder import Encoder
import csv
from threading import Semaphore

class Robot:
    def __init__(self, semaphore: Semaphore):

        self.encoderA = Encoder(RobotHardware.RIGHT_ENCODED_FRONT, RobotHardware.RIGHT_ENCODED_BACK)
        self.encoderB = Encoder(RobotHardware.LEFT_ENCODED_FRONT, RobotHardware.LEFT_ENCODED_BACK)
        self.__pwm_right = None
        self.__pwm_left = None

        self.sampling_time = 4.5
        self.ready_to_run_model = False
        
        self.data = []
        self.__semaphore = semaphore

    def init(self, left_power, right_power):
        
        GPIO.setwarnings(True)
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(RobotHardware.LEFT_ENABLE, GPIO.OUT)
        GPIO.setup(RobotHardware.RIGHT_ENABLE, GPIO.OUT)

        GPIO.setup(RobotHardware.LEFT_BACKWARD, GPIO.OUT)
        GPIO.setup(RobotHardware.LEFT_FORWARD, GPIO.OUT)

        GPIO.setup(RobotHardware.RIGHT_BACKWARD,   GPIO.OUT)
        GPIO.setup(RobotHardware.RIGHT_FORWARD, GPIO.OUT)

        GPIO.output(RobotHardware.RIGHT_BACKWARD,   GPIO.LOW)
        GPIO.output(RobotHardware.LEFT_BACKWARD,   GPIO.LOW)

        GPIO.output(RobotHardware.LEFT_FORWARD, GPIO.HIGH)
        GPIO.output(RobotHardware.RIGHT_FORWARD, GPIO.HIGH)

        self.__pwm_right = GPIO.PWM(RobotHardware.RIGHT_ENABLE, 1000)
        self.__pwm_left = GPIO.PWM(RobotHardware.LEFT_ENABLE, 1000)

        self.__pwm_right.start(0)
        self.__pwm_left.start(0)

        self.__pwm_right.ChangeDutyCycle(right_power)
        self.__pwm_left.ChangeDutyCycle(left_power)

        self.encoderA.run()
        self.encoderB.run()


        now = time.time()

        while self.encoderA.running.value:
            time.sleep(0.2)

            ul = -self.encoderB.get_w()
            ur = self.encoderA.get_w()

            wrobotl = -RobotHardware.RAY * ul / ( RobotHardware.DISTANCE_WHEELS)
            wrobotr = RobotHardware.RAY * ur / ( RobotHardware.DISTANCE_WHEELS)

            wrobot = wrobotl + wrobotr

            vrobotl = RobotHardware.RAY * ul / 2
            vrobotr = RobotHardware.RAY * ur / 2

            vrobot = (vrobotl + vrobotr)

            self.data.append([time.time(), vrobot, wrobot, ul, ur])


            current = time.time()

            if current - now >= self.sampling_time:
                now = current
                with open(f'./data/data_left_{left_power}_right_{right_power}_robot_{time.time()}.csv', 'w') as arquivo_csv:
                    writer = csv.writer(arquivo_csv)
                    writer.writerow(['Timestamp', 'Velocidade linear', 'Velocidade angular', 'Velocidade Roda Esquerda', 'Velocidade Roda Direita'])
                    for row in self.data:
                        writer.writerow(row)
                self.__semaphore.acquire(blocking=True)
                self.ready_to_run_model = True
                self.__semaphore.release()
              
                
    def change_duty_cycle(self, left_power, right_power):
        self.__pwm_right.ChangeDutyCycle(right_power)
        self.__pwm_left.ChangeDutyCycle(left_power)

    def is_prepare_to_train_again(self): 
        return self.ready_to_run_model

    def model_job_done(self):
        self.ready_to_run_model = False

    def get_dt(self):
        return self.encoderA.get_dt()

    def get_w_robot(self):
        print("oi")

    def get_velocity_robot(self):
        print("oi")

if __name__ == '__main__':
    robot_controller = Robot(Semaphore())
    left_power = int(input('Digite quantos porcentos da roda esquerda [0-100]:'))
    right_power = int(input('Digite quantos porcentos da roda direitas [0-100]:'))
    robot_controller.init(left_power, right_power)
