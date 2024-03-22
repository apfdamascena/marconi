import RPi.GPIO as GPIO
import time
import math
import multiprocessing


class Encoder(multiprocessing.Process):
    def __init__(self, pinA, pinB) -> None:
        super().__init__()
        self.ENCA = pinA
        self.ENCB = pinB

        self._w = multiprocessing.Value('f', 0)

        self.can_get_data = multiprocessing.Value('b', False)
        
        # these parameters are specific for cm25-370 gear motor encoders
        self.step_angle = 2*math.pi/12

        self.d_theta = 0
        self.reduction = 38

        self.counter = 0


        self.sampling_time = 0.04
        self.dt = multiprocessing.Value('f', 0.0)
        self.last_time = time.time()
        self._delta_theta = 0
        self.running = multiprocessing.Value('b', True)

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.ENCA, GPIO.IN)
        GPIO.setup(self.ENCB, GPIO.IN)
        
    
    def run(self):
        GPIO.add_event_detect(self.ENCA, GPIO.RISING, callback=self._angle_callback, bouncetime=5)

    
    def stop(self):
        self.running.value = False

    def _angle_callback(self, ENCA):
        
        Switch_A = GPIO.input(ENCA)
        Switch_B = GPIO.input(self.ENCB)
    
        if (Switch_A == 1) and (Switch_B == 0):
            if(self._delta_theta > 0):
                self._delta_theta = 0

            self._delta_theta -= self.step_angle/self.reduction

            #print("direction <-")

            now = time.time()
            if((now - self.last_time) >= self.sampling_time):
                self.dt.value = now - self.last_time
                self._w.value = self._delta_theta/self.dt.value

                
                self.can_get_data.value = True
                while self.can_get_data == True:
                    time.sleep(0.001)
                self._delta_theta = 0  
                self.last_time = time.time()

                

            while Switch_B == 0:
                Switch_B = GPIO.input(self.ENCB)
            while Switch_B == 1:
                Switch_B = GPIO.input(self.ENCB)
            return
    
        elif (Switch_A == 1) and (Switch_B == 1):
            if(self._delta_theta < 0):
                self._delta_theta = 0
            #print("direction ->")
            self._delta_theta += self.step_angle/self.reduction


            now = time.time()
            if((now - self.last_time) >= self.sampling_time):
                self.dt.value = now - self.last_time
                self._w.value = self._delta_theta/self.dt.value

                #print(self.dt.value)
                self.can_get_data.value = True

                while self.can_get_data == True:
                    time.sleep(0.001)
                
                self._delta_theta = 0

                self.last_time = time.time()
            while Switch_A == 1:
                Switch_A = GPIO.input(self.ENCA)
            return
        else:
            return


    def data_collected(self):
        self.can_get_data.value = False
    def new_data_flag(self):
        return self.can_get_data.value
    def get_w(self):
        return 2 * self._w.value
    def get_d_theta(self):
        return self._delta_theta
    def get_dt(self):
        return self.dt.value
