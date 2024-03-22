from roboto import Robot
from identifier import Identifier
from data_handler import DataHandler
from threading import Thread, Semaphore
import RPi.GPIO as GPIO
from lidar import LidarSensor
import time
import math


class RobotAngleParser:

    def get_w(self, theta: int, time: float):
        return theta / time
    

class Controller:

    def __init__(self, robot: Robot, 
                       model: Identifier, 
                       data_handler: DataHandler, 
                       semaphore: Semaphore, 
                       lidar: LidarSensor,
                       parser_angle: RobotAngleParser
                       ) -> None:
        self.__robot = robot
        self.__model = model
        self.__data_handler = data_handler
        self.__read_semaphore = semaphore
        self.__lidar = lidar
        self.__parser_angle = parser_angle

    def stop(self):
        GPIO.cleanup()
        self.__robot.__pwm_left.ChangeDutyCycle(0)
        self.__robot.__pwm_right.ChangeDutyCycle(0)
        self.__lidar.stop()

    def run_data_handler(self):
        self.__data_handler.remove_outliers() 
        self.__data_handler.add_direction("./cleaned-data")
        self.__data_handler.concat('./direction')

    def run_training_model(self):
        self.__model.run()
        print("oia")

    def start(self): 
        while True:
            print("[CONTROLLER] Running!")
            self.__robot.change_duty_cycle(100, 100)
            
            lidar_angle = self.__lidar.get_angle()
            time = self.__robot.get_dt()
            if time == 0:
                time += 1
                continue
            w = self.__parser_angle.get_w(lidar_angle, time)
            velocity = 0.09

            self.__robot.change_duty_cycle(0, 0)

            predictions = self.__model.predict_new_values(velocity, w)
            [left_wheels, right_weels] = predictions[0]

            pwm_left = (left_wheels / math.pi * 2) * 100
            pwm_right = (right_weels / math.pi * 2) * 100

            print(f"[wheels]: {pwm_left}, {pwm_right}")


            self.__robot.change_duty_cycle(pwm_left, pwm_right)
            current_theta = 0
            while current_theta <= abs(lidar_angle):
                current_theta += w * time
    

            print(f"[LidarSensor]: this is my angle to rotate: {lidar_angle}")
            print(f"[Velocidade angular]: this is my angle to rotate: {w}")


            if self.__robot.ready_to_run_model:
                self.run_data_handler()
                self.run_training_model()

            time.sleep(0.5)


if __name__ == "__main__":
    try:
        semaphore = Semaphore(1)
        data = DataHandler("./data", semaphore)
        model = Identifier("./direction/robot_data.csv", semaphore)
        robot = Robot(semaphore)
        lidar = LidarSensor()
        parser_angle = RobotAngleParser()

        controller = Controller(robot, model, data, semaphore, lidar, parser_angle)
        
        threads = []
        threads.append(Thread(target=model.run))
        threads.append(Thread(target=lidar.start_scan))
        threads.append(Thread(target=robot.init, args=(100, 100)))
        threads.append(Thread(target=controller.start))

        time.sleep(2)
        for i, thread in list(enumerate(threads)):
            print(f"Starting thread {i}...")
            time.sleep(2)
            thread.start()

        # for i, thread in enumerate(threads):
        #     print(f"Joining thread {i}...")
        #     thread.join()
    
    except KeyboardInterrupt:
        controller.stop()
        




