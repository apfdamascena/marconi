from roboto import Robot
from identifier import Identifier
from data_handler import DataHandler
from threading import Thread, Semaphore, Condition
import RPi.GPIO as GPIO
from lidar import LidarSensor
import time
import numpy as np
import math


# class PIDController:

#     def __init__(self, KP: float, KI: float, KD: float) -> None:
#         self.KP = KP  
#         self.KI = KI  
#         self.KD = KD  
#         self.prev_error = 0  
#         self.integral = 0  

#     def calculate_control_signal(self, error: float, dt: float):
    
#         proportional_term = self.KP * error
        
#         self.integral += error * dt
#         integral_term = self.KI * self.integral
        
#         derivative_term = self.KD * (error - self.prev_error) / dt
        
#         control_signal = proportional_term + integral_term + derivative_term
        
#         self.prev_error = error
        
#         return control_signal

    
class Controller:

    def __init__(self, robot: Robot, 
                       model: Identifier, 
                       data_handler: DataHandler, 
                       semaphore: Semaphore, 
                       lidar: LidarSensor,
                    #    pid: PIDController
                       ) -> None:

        self.__robot = robot
        self.__model = model
        self.__data_handler = data_handler
        self.__read_semaphore = semaphore
        self.__lidar = lidar
        # self.__parser_angle = parser_angle
        # self.__pid = pid
        self.__condition = Condition()
        self.__initial_angular_speed = 0.5

    def stop(self):
        GPIO.cleanup()
        self.__robot.__pwm_left.ChangeDutyCycle(0)
        self.__robot.__pwm_right.ChangeDutyCycle(0)
        self.__lidar.stop()

    def __run_data_handler(self):
        self.__data_handler.remove_outliers() 
        self.__data_handler.concat('./cleaned-data')

        with self.__condition:
            self.__condition.notify_all()

    def __run_training_model(self):
        self.__model.run()

    def start_walk(self):

        desired_distance = 0.5  # Distância desejada à parede
        safe_distance = 0.7    # Distância de segurança para esquivar de obstáculos

        while True:

            #time.sleep(0.1)

            # if self.__robot.is_prepare_to_train_again():
            #     self.__robot.change_duty_cycle(0,0)
            #     self.__run_data_handler()

            #     with self.__condition:
            #         self.__condition.wait()

            #     self.__run_training_model()
            #     self.__robot.model_job_done()

            
            # desired_linear_speed = 1.1
            # angular_speed = self.__initial_angular_speed  

            # linear_speed = 0.7 # precisa calcular depois

            # ## calucla left_pwm and right_pwm. precisa calcular depois

            # self.__robot.change_duty_cycle(left_pwm, right_pwm)
            # time.sleep(0.2)

            # corrections = self.__model.predict_new_values(linear_speed, angular_speed)
            # error = desired_linear_speed - linear_speed

            # control_signal = self.__pid.calculate_control_signal(error, 0.2)  
            # corrected_angular_speed = angular_speed + control_signal
            # self.__initial_angular_speed = corrected_angular_speed

        
            front_distance = self.__lidar.get_front()
            left_distance, right_distance = self.__lidar.get_side_distances()
            left_diagonal, right_diagonal = self.__lidar.get_diagonal_distances()

            angle_to_move = self.__lidar.get_angle()

            print(f"[Front-Distance]: {front_distance}")
            print(f"[Side-Distance]: {left_distance} - {right_distance}")
            
            if front_distance < safe_distance:
                print("[Robot]: stopped because is to close")
                self.__robot.change_duty_cycle(0, 0)
                print("[Robot]: stopping ")
            #     front_now = self.__lidar.get_front()
            #     if np.abs(front_now - safe_distance) < 0.01:
            #         print("[Robot]: go back because it is to close")
            #     # while np.abs(front_now - safe_distance) <= 1.2:
            #     #     ## call function to go back
            #     #     print("[Robot]: going back")
            #     #     front_now = self.__lidar.get_front()
            #     #     time.sleep(0.1)

            #     print("[Robot]: avaiable to turn left or right")

            #     if left_distance > right_distance:
            #         print("Avoiding obstacle: Turning left")
            #         self.__robot.change_duty_cycle(0, 75)
            #     else:
            #         print("Avoiding obstacle: Turning right")
            #         self.__robot.change_duty_cycle(75, 0)
                
            # elif left_distance < safe_distance:
            #     print("[Robot]: too close to left wall")
            #     self.__robot.change_duty_cycle(85, 45)
            # elif right_distance < safe_distance:
            #     print("[Robot]: too close to right wall")
            #     self.__robot.change_duty_cycle(45, 85)
            # elif left_distance < desired_distance: 
            #     print("[Robot]: out of desire distance left wall")
            #     self.__robot.change_duty_cycle(85, 60)
            # elif right_distance < desired_distance:
            #     print("[Robot]: out of desire distance right wall")
            #     self.__robot.change_duty_cycle(60, 85)
            else:
                #se o robo ficar em duvida para onde ele tem que ir porque left and right ta muito perto 
                if abs(left_distance - right_distance) < 0.1:
                    print("[robot]: where should I go?")
                    if left_diagonal > right_diagonal:
                        print("[robot]: I decided to go right")
                        self.__robot.change_duty_cycle(55, 90)
                    else:
                        print("[robot]: I decided to go left")
                        self.__robot.change_duty_cycle(90, 55)
                else:
                    print("[robot]: happy happy happy, go forward")
                    self.__robot.change_duty_cycle(65, 65)

if __name__ == "__main__":
    try:
        semaphore = Semaphore(1)
        data = DataHandler("./data", semaphore)
        model = Identifier("./cleaned-data/robot_data.csv", semaphore)
        robot = Robot(semaphore)
        lidar = LidarSensor()
        # pid = PIDController(1.0, 0.01, 0.1)

        controller = Controller(robot, model, data, semaphore, lidar)
        
        threads = []
        # threads.append(Thread(target=model.run))
        threads.append(Thread(target=lidar.scan))
        threads.append(Thread(target=robot.init, args=(50, 50)))
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
        




