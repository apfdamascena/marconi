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
    TO_LEFT = -0.7623631569559024
    TO_RIGHT = 0.8115732129705852
    TO_BRUTE_LEFT = 0.6535551878625795
    TO_BRUTE_RIGHT = -0.5826899932232819

class LinearVelocity:
    TO_FORWARD = 0.1383949456214905
    TO_LEFT = 0.062476768702268606
    TO_RIGHT = 0.06834387719631195
    TO_BRUTE_LEFT = 0.0653555187862579
    TO_BRUTE_RIGHT = 0.06
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

        desired_distance = 0.5
        safe_distance = 0.5
        safe_distance_wall = 0.1

        while True:
            # if self.__lidar.is_not_ready():
            #     self.__robot.change_duty_cycle(0, 0)
            #     continue

            # if self.__robot.is_prepare_to_train_again():
            #     self.__robot.change_duty_cycle(0,0)
            #     self.__run_data_handler()
            #
            #     with self.__condition:
            #         self.__condition.wait()
            #
            #     self.__run_training_model()

            left_wheels, right_wheels = self.__model.predict_velocity(self.__state)

            print(f"[left-wheels]: {left_wheels}")
            print(f"[right-wheels]: {right_wheels}")

            pwm_left = (left_wheels / (2 * math.pi)) * 100
            pwm_right = (right_wheels / (2 * math.pi)) * 100
            if pwm_left > 100:
                pwm_left = 100
            elif pwm_left < 0:
                pwm_left = 0
            if pwm_right > 100:
                pwm_right = 100
            elif pwm_right < 0:
                pwm_right = 0



            print(f"[pwm-left]: {pwm_left}")
            print(f"[pwm-right]: {pwm_right}")

            self.__robot.change_duty_cycle(pwm_left, pwm_right)

            front_distance = self.__lidar.get_front()
            left_distance, right_distance = self.__lidar.get_side_distances()
            left_diagonal, right_diagonal = self.__lidar.get_diagonal_distances()

            angle_to_move = self.__lidar.get_angle()

            print(f"[Front-Distance]: {front_distance}")
            print(f"[Side-Distance]: {left_distance} - {right_distance}")
            
            if front_distance < safe_distance:
                if left_distance < safe_distance and right_distance < safe_distance:
                    self.__robot.change_duty_cycle(0, 0)

                # self.__robot.change_duty_cycle(0, 0)

                # front_now = self.__lidar.get_front()
                # if front_now - safe_distance < 0.01:
                #     print("[Robot]: go back because it is to close")

                # if front_now - safe_distance <= 1.2:
                #     ## call function to go back
                #     print("[Robot]: going back")
                #     front_now = self.__lidar.get_front()
                #     time.sleep(0.1)

                print("left", left_distance, "right", right_distance)

                if left_distance > right_distance:
                    print("Avoiding obstacle: Turning left")
                    # self.__robot.change_duty_cycle(0, 75)
                    self.__state = StateMachine.BRUTE_LEFT
                else:
                    print("Avoiding obstacle: Turning right")
                    self.__state = StateMachine.BRUTE_RIGHT
                    # self.__robot.change_duty_cycle(75, 0)
                
            elif left_distance < safe_distance_wall or right_distance < safe_distance_wall:
                if left_distance < right_distance:
                    self.__state = StateMachine.RIGHT
                else:
                    self.__state = StateMachine.LEFT

                # print("[Roboot]: too close to left wall")
                # self.__robot.change_duty_cycle(85, 45)
                # self.__state = StateMachinne.LEFT
            # elif right_distance < safe_distance_wall:
            #     print("[Robot]: too close to right wall")
            #     # self.__robot.change_duty_cycle(45, 85)
            #     self.__state = StateMachine.RIGHT
            # elif left_distance < desired_distance:
            #     print("[Robot]: out of desire distance left wall")
            #     # self.__robot.change_duty_cycle(85, 60)
            #     self.__state = StateMachine.LEFT
            # elif right_distance < desired_distance:
            #     print("[Robot]: out of desire distance right wall")
            #     # self.__robot.change_duty_cycle(60, 85)
            #     self.__state = StateMachine.RIGHT
            else:
                #se o robo ficar em duvida para onde ele tem que ir porque left and right ta muito perto 
                if abs(left_distance - right_distance) < 0.1:
                    print("[robot]: where should I go?")
                    if left_diagonal > right_diagonal:
                        print("[robot]: I decided to go right")
                        # self.__robot.change_duty_cycle(55, 90)
                        self.__state = StateMachine.BRUTE_RIGHT
                    else:
                        print("[robot]: I decided to go left")
                        # self.__robot.change_duty_cycle(90, 55)
                        self.__state = StateMachine.BRUTE_LEFT
                else:
                    print("[robot]: happy happy happy, go forward")
                    # self.__robot.change_duty_cycle(65, 65)
                    self.__state = StateMachine.FORWARD

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
        threads.append(Thread(target=controller.start_walk))
    
        time.sleep(0.1)
        print(f"Starting thread {0}...")
        threads[0].start()

        time.sleep(0.9)
        print(f"Starting thread {1}...")
        threads[1].start()


        time.sleep(0.2)
        print(f"Starting thread {2}...")
        threads[2].start()


        # time.sleep(0.6)
        # print(f"Starting thread {3}...")
        # threads[3].start()

    
    except KeyboardInterrupt:
        controller.stop()
        




