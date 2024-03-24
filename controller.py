from roboto import Robot
from identifier import Identifier
from data_handler import DataHandler
from threading import Thread, Semaphore, Condition
import RPi.GPIO as GPIO
from lidar import LidarSensor
import time
import numpy as np
import math
from enum import Enum

class AngularVelocity:

    TO_FORWARD = 0.0083952626592779
    TO_LEFT = 0.5515732129705852
    TO_RIGHT = -0.6223631569559024
    TO_BRUTE_LEFT = 0.6535551878625795
    TO_BRUTE_RIGHT = -0.5826899932232819

class LinearVelocity:
    TO_FORWARD = 0.1383949456214905
    TO_LEFT = 0.0834387719631195
    TO_RIGHT = 0.102476768702268606
    TO_BRUTE_LEFT = 0
    TO_BRUTE_RIGHT = 0
    VELOCITY_STOPPED = 0


class StateMachine(Enum): 

    FORWARD = 1
    LEFT = 2
    RIGHT = 3
    BRUTE_LEFT = 4
    BRUTE_RIGHT = 5
    STOPPED = 6

    def give_angular_velocity(self):
        if self == StateMachine.FORWARD:
            return LinearVelocity.TO_FORWARD, AngularVelocity.TO_FORWARD
        elif self == StateMachine.LEFT:
            return LinearVelocity.TO_LEFT, AngularVelocity.TO_LEFT
        elif self == StateMachine.RIGHT:
            return LinearVelocity.TO_RIGHT, AngularVelocity.TO_RIGHT
        elif self == StateMachine.BRUTE_LEFT:
            return LinearVelocity.TO_BRUTE_LEFT, AngularVelocity.TO_BRUTE_LEFT
        elif self == StateMachine.STOPPED:
            return LinearVelocity.VELOCITY_STOPPED, AngularVelocity.TO_FORWARD

        return LinearVelocity.TO_BRUTE_RIGHT, AngularVelocity.TO_BRUTE_RIGHT

        
        
class Controller:

    def __init__(self, robot: Robot, 
                       model: Identifier, 
                       data_handler: DataHandler, 
                       semaphore: Semaphore, 
                       lidar: LidarSensor,
                       ) -> None:

        self.__robot = robot
        self.__data_handler = data_handler
        self.__read_semaphore = semaphore
        self.__lidar = lidar
        self.__model = model
        self.__condition = Condition()
        self.__initial_angular_speed = 0.5
        self.__state = StateMachine.FORWARD

    def stop(self):
        GPIO.cleanup()
        self.__robot.__pwm_left.ChangeDutyCycle(0)
        self.__robot.__pwm_right.ChangeDutyCycle(0)
        self.__lidar.stop()

    def __run_data_handler(self):
        self.__data_handler.remove_outliers() 
        self.__data_handler.concat('asterix-data')

        with self.__condition:
            self.__condition.notify_all()

    def __run_training_model(self):
        self.__model.run()
        self.__robot.model_job_done()

    def start_walk(self):
        safe_distance = 0.5
        safe_distance_wall = 0.2

        while True:
            self.__robot.change_duty_cycle(0, 0)
            time.sleep(0.4)

            left_wheels, right_wheels = self.__model.predict_velocity(self.__state)

            pwm_left = (left_wheels / (2 * math.pi)) * 100
            pwm_right = (right_wheels / (2 * math.pi)) * 100
            original_pwm_left = pwm_left
            original_pwm_right = pwm_right

            if pwm_left > 100:
                pwm_left = 100
            elif 60 > pwm_left > 20:
                pwm_left = 60
            elif pwm_left < 0:
                pwm_left = 0
            if pwm_right > 100:
                pwm_right = 100
            elif 60 > pwm_right > 20:
                pwm_right = 60
            elif pwm_right < 0:
                pwm_right = 0

            self.__robot.change_duty_cycle(pwm_left, pwm_right)

            front_distance = self.__lidar.get_front()
            # left_distance, right_distance = self.__lidar.get_side_distances()
            left_distance, right_distance = self.__lidar.get_diagonal_distances()

            if front_distance < safe_distance:
                # if left_distance < safe_distance and right_distance < safe_distance:
                #     self.__robot.change_duty_cycle(0, 0)

                if left_distance > right_distance:
                    print(f"[robot]: avoiding obstacle, turning left | wheels {left_wheels} {right_wheels} | [orig pwm] {original_pwm_left} {original_pwm_right} | [pwm] {pwm_left} {pwm_right} | [Dist]: {left_distance} {front_distance} {right_distance}")
                    self.__state = StateMachine.BRUTE_LEFT

                else:
                    print(f"[robot]: avoiding obstacle, Turning right | wheels {left_wheels} {right_wheels} | [orig pwm] {original_pwm_left} {original_pwm_right} | [pwm] {pwm_left} {pwm_right} | [Dist]: {left_distance} {front_distance} {right_distance}")
                    self.__state = StateMachine.BRUTE_RIGHT


            elif abs(left_distance - right_distance) > 0.3:
                if left_distance < right_distance:
                    print(f"[robot]: adjusting myself to right | wheels {left_wheels} {right_wheels} | [orig pwm] {original_pwm_left} {original_pwm_right} | [pwm] {pwm_left} {pwm_right} | [Dist]: {left_distance} {front_distance} {right_distance}")
                    self.__state = StateMachine.RIGHT
                    # self.__robot.change_duty_cycle(pwm_left, pwm_right)
                    # time.sleep(0.1)
                    # self.__robot.change_duty_cycle(0, 0)
                    # self.__state = StateMachine.FORWARD
                    # self.__robot.change_duty_cycle(pwm_left, pwm_right)
                    # time.sleep(0.2)
                    # self.__robot.change_duty_cycle(0, 0)
                    # time.sleep(0.2)
                    # self.__state = StateMachine.LEFT
                    # self.__robot.change_duty_cycle(pwm_left, pwm_right)
                    # time.sleep(0.2)
                    # self.__robot.change_duty_cycle(0, 0)
                    # time.sleep(0.2)
                    # self.__state = StateMachine.FORWARD
                    # self.__robot.change_duty_cycle(pwm_left, pwm_right)
                else:
                    print(f"[robot]: adjusting myself to left | wheels {left_wheels} {right_wheels} | [orig pwm] {original_pwm_left} {original_pwm_right} | [pwm] {pwm_left} {pwm_right} | [Dist]: {left_distance} {front_distance} {right_distance}")
                    self.__state = StateMachine.LEFT

                    # time.sleep(0.1)
                    # self.__robot.change_duty_cycle(0, 0)
                    # self.__state = StateMachine.FORWARD
                    # self.__robot.change_duty_cycle(pwm_left, pwm_right)
                    # time.sleep(0.2)
                    # self.__robot.change_duty_cycle(0, 0)
                    # time.sleep(0.2)
                    # self.__state = StateMachine.RIGHT
                    # self.__robot.change_duty_cycle(pwm_left, pwm_right)
                    # time.sleep(0.2)
                    # self.__robot.change_duty_cycle(0, 0)
                    # time.sleep(0.2)
                    # self.__state = StateMachine.FORWARD
                    # self.__robot.change_duty_cycle(pwm_left, pwm_right)

            # elif left_distance < safe_distance_wall or right_distance < safe_distance_wall:
            #     if left_distance < right_distance:
            #         print(f"[robot]: avoiding left wall | wheels {left_wheels} {right_wheels} | [orig pwm] {original_pwm_left} {original_pwm_right} | [pwm] {pwm_left} {pwm_right} | [Dist]: {left_distance} {front_distance} {right_distance}")
            #         self.__state = StateMachine.RIGHT
            #     else:
            #         print(f"[robot]: avoiding right wall | wheels {left_wheels} {right_wheels} | [orig pwm] {original_pwm_left} {original_pwm_right} | [pwm] {pwm_left} {pwm_right} | [Dist]: {left_distance} {front_distance} {right_distance}")
            #         self.__state = StateMachine.LEFT

            else:
                print(f"[robot]: Going forward | wheels {left_wheels} {right_wheels} | [orig pwm] {original_pwm_left} {original_pwm_right} | [pwm] {pwm_left} {pwm_right} | [Dist]: {left_distance} {front_distance} {right_distance}")
                self.__state = StateMachine.FORWARD
                # self.__robot.change_duty_cycle(60, 60)
                # time.sleep(0.3)

            self.__robot.change_duty_cycle(pwm_left, pwm_right)
            time.sleep(0.17)
if __name__ == "__main__":
    try:
        semaphore = Semaphore(1)
        data = DataHandler("./asterix-data", semaphore)
        model = Identifier("asterix-data/robot_data.csv", semaphore)


        print("[CONTROLLER] Running Model...")
        model.run()
        print("[CONTROLLER] Done model")
        robot = Robot(semaphore)
        lidar = LidarSensor()
        # pid = PIDController(1.0, 0.01, 0.1)

        controller = Controller(robot, model, data, semaphore, lidar)
        
        threads = []
        # threads.append(Thread(target=model.run))
        threads.append(Thread(target=lidar.scan))
        threads.append(Thread(target=robot.init, args=(0, 0)))
        # threads.append(Thread(target=controller.start_walk))
    
        time.sleep(0.1)
        print(f"Starting thread {0}...")
        threads[0].start()

        time.sleep(0.9)
        print(f"Starting thread {1}...")
        threads[1].start()


        controller.start_walk()


        # time.sleep(0.6)
        # print(f"Starting thread {3}...")
        # threads[3].start()

    
    except KeyboardInterrupt:
        controller.stop()
        




