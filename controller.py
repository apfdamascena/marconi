from roboto import Robot
from identifier import Identifier
from data_handler import DataHandler
from threading import Thread, Semaphore
import RPi.GPIO as GPIO
from lidar import LidarSensor
import time
import math


class RobotAngleParser:

    def get_w(self, theta: int, time_value: float):
        print("Getting W - Theta = {theta} --- Time = {time_value}")
        return theta / time_value
    
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
    
    def walk(self):
        [0.19425773436261207, 0.2531785706856421, 0.18814101948952064, 0.25891525361497525, 0.18736363811926407]
        lidar_angle = self.__lidar.get_front()
        while True:
            if (self.__lidar.get_front() < 0.5):

                self.__robot.change_duty_cycle(100,100)
                w = 0.5426597420629157

                angle = self.__lidar.bigger_point()
                print(f"Angle is {angle}")

                t = abs(math.radians(angle)) / w
                print(f"Time is {t}")

                
                if angle > 0:
                    print(f"Angle is positive going left...")
                    self.__robot.change_duty_cycle(0, 100)
                elif angle < 0:
                    print(f"Angle is NEGATIVE going right...")
                    self.__robot.change_duty_cycle(100, 0)
                else:
                    print(f"Angle is zero going front...")
                    self.__robot.change_duty_cycle(100, 100)


                start_time = time.time()
                while (time.time() - start_time) < t:
                    pass
            
                # print(f"Changing duty cycle back to 0")
                # self.__robot.change_duty_cycle(0,0)

            else:
                self.__robot.change_duty_cycle(100, 100)



    def start(self): 
        print("ENTROU NO START")
        now = time.time()
        self.__robot.change_duty_cycle(100, 100)

        while True:
            print("COMEÇØU A RODAR O CONTROLLER")
            time.sleep(0.5)
            
            lidar_angle = self.__lidar.get_angle()
            print(f"[Lidar Angle]: {lidar_angle}")

            time_robot = 2

            w = self.__parser_angle.get_w(lidar_angle, time_robot)
            print(f"[w]: {w}")
            velocity = 0.1220753045082092

            self.__robot.change_duty_cycle(0, 0)

            predictions = self.__model.predict_new_values(velocity, w)
            [left_wheels, right_weels] = predictions[0]
            print(f"[wheels before pwm]: {left_wheels}, {right_weels}")

            pwm_left = (left_wheels / (math.pi * 2)) * 100
            pwm_right = (right_weels / (math.pi * 2)) * 100

            print(f"[wheels]: {pwm_left}, {pwm_right}")

            if pwm_left < 55:
                pwm_left = pwm_left * 1.5
            if pwm_right < 55:
                pwm_right = pwm_right * 1.5

            print(f"[wheels]: {pwm_left}, {pwm_right}")


            self.__robot.change_duty_cycle(pwm_left, pwm_right)
            current_theta = 0
            while current_theta <= abs(lidar_angle):
                current_theta += w * time_robot
    

            print(f"[LidarSensor]: this is my angle to rotate: {lidar_angle}")
            print(f"[Velocidade angular]: this is my angle to rotate: {w}")


            if self.__robot.ready_to_run_model:
                self.run_data_handler()
                self.run_training_model()




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
        # threads.append(Thread(target=model.run))
        threads.append(Thread(target=lidar.start_scan))
        threads.append(Thread(target=robot.init, args=(100, 100)))
        threads.append(Thread(target=controller.start))
    
        time.sleep(0.1)
        print(f"Starting thread {0}...")
        threads[0].start()

        time.sleep(0.9)
        print(f"Starting thread {1}...")
        threads[1].start()


        time.sleep(0.2)
        print(f"Starting thread {2}...")
        threads[2].start()


        time.sleep(0.6)
        print(f"Starting thread {3}...")
        threads[3].start()


        # for i, thread in enumerate(threads):
        #     print(f"Joining thread {i}...")
        #     thread.join()
    
    except KeyboardInterrupt:
        controller.stop()
        




