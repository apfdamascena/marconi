from roboto import Robot
from identifier import Identifier
from data_handler import DataHandler
from threading import Thread, Semaphore
import RPi.GPIO as GPIO


class Controller:

    def __init__(self, robot: Robot, model: Identifier, data_handler: DataHandler, semaphore: Semaphore) -> None:
        self.__robot = robot
        self.__model = model
        self.__data_handler = data_handler
        self.__read_semaphore = semaphore
        self.__lidar = None

    def stop(self):
        GPIO.cleanup()
        self.__robot.__pwm_left.ChangeDutyCycle(0)
        self.__robot.__pwm_right.ChangeDutyCycle(0)

    def run_data_handler(self):
        self.__data_handler.remove_outliers() 
        self.__data_handler.add_direction("./cleaned-data")
        self.__data_handler.concat('./direction')

    def run_training_model(self):
        self.__model.run()
        print("oia")

    def start(self): 

        while True:
            self.__robot.init(100, 100)

            if self.__robot.ready_to_run_model:
                self.run_data_handler()
                self.run_training_model()


if __name__ == "__main__":
    try:
        semaphore = Semaphore(1)
        data = DataHandler("./data", semaphore)
        model = Identifier("./direction/robot_data.csv", semaphore)
        robot = Robot(semaphore)
        controller = Controller(robot, model, data, semaphore)
        
        threads = []
        threads.append(Thread(target=robot.init, args=(100, 100)))
        threads.append(Thread(target=controller.start))
        threads.append(Thread(target=model.run))

        for i, thread in list(enumerate(threads)):
            print(f"Starting thread {i}...")
            thread.start()

        for i, thread in enumerate(threads):
            print(f"Joining thread {i}...")
            thread.join()
    
    except KeyboardInterrupt:
        controller.stop()




