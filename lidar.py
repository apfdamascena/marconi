import os
import ydlidar
import time
from robot import init
from robot import change

class LidarSensor:

    def __init__(self):
        self.__laser = None
        self.__setup()
        self.__angle = 0

    
    def __setup(self):
        ydlidar.os_init()
        ports = ydlidar.lidarPortList()
        port = "/dev/ydlidar"

        for key, value in ports.items():
            port = value

        self.__laser = ydlidar.CYdLidar()
        self.__laser.setlidaropt(ydlidar.LidarPropSerialPort,port)
        self.__laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 115200)
        self.__laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
        self.__laser.setlidaropt( ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
        self.__laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
        self.__laser.setlidaropt(ydlidar.LidarPropSampleRate, 3)
        self.__laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)
        self.__laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
        self.__laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
        self.__laser.setlidaropt(ydlidar.LidarPropMaxRange, 16.0)
        self.__laser.setlidaropt(ydlidar.LidarPropMinRange, 0.08)
        self.__laser.setlidaropt(ydlidar.LidarPropIntenstiy, False)


    def ____adjust_angle(self, angle1, angle2):
        start = 2.1

        if angle1 < 0:
            angle1 = angle1 + 360 
        if angle2 < 0:
            angle2 = angle2 + 360 
        if start < 0:
            start = start + 6.28
            
        angle1 = (start+(angle1*3.14/180))%6.28
        angle2 = (start+(angle2*3.14/180))%6.28

        maximum = max([angle1,angle2])
        minimum = min([angle1,angle2])

        if maximum > 3.14:
            maximum = -3.14 + maximum%3.14
        if minimum > 3.14:
            minimum = -3.14 + minimum%3.14

        if abs(maximum - minimum) > abs(maximum) and abs(minimum - maximum) > abs(minimum):
            return [maximum, minimum]
        
        else:
            return [minimum, maximum]
    
    def get_angle(self) -> int:
        return self.__angle

    def start_scan(self):
        initialized = self.__laser.initialize()
        if initialized:
            initialized = self.__laser.turnOn()
        try:
            while initialized and ydlidar.os_isOk():
                scan = ydlidar.LaserScan()
                r = self.__laser.doProcessSimple(scan)

                startPoints = 0
                pointsQuantity = 0
              
                if r:
                    distanceFromAngles = [0, 0, 0, 0, 0]
                    quantity = [0, 0, 0, 0, 0]

                    for n in range(scan.points.size()): 
                        if scan.points[n].angle > self.__adjust_angle(-95,-85)[0] and scan.points[n].angle < self.__adjust_angle(-95,-85)[1]:
                            if (scan.points[n].range!=0):
                                distanceFromAngles[0] += scan.points[n].range
                                quantity[0] += 1

                        elif scan.points[n].angle > self.__adjust_angle(-50,-40)[0] and scan.points[n].angle < self.__adjust_angle(-50,-40)[1]:
                            if (scan.points[n].range!=0):
                                distanceFromAngles[1] += scan.points[n].range
                                quantity[1] += 1

                        elif scan.points[n].angle > self.__adjust_angle(-5,5)[0] and scan.points[n].angle < self.__adjust_angle(-5,5)[1]:
                            if (scan.points[n].range!=0):
                                print(scan.points[n].range)
                                distanceFromAngles[2] += scan.points[n].range
                                quantity[2] += 1
                        elif scan.points[n].angle > self.__adjust_angle(50,40)[0] and scan.points[n].angle < self.__adjust_angle(50,40)[1]:
                            if (scan.points[n].range!=0):
                                distanceFromAngles[3] += scan.points[n].range 
                                quantity[3] += 1
                        elif scan.points[n].angle > self.__adjust_angle(95,85)[0] and scan.points[n].angle < self.__adjust_angle(95,85)[1]:
                            if (scan.points[n].range!=0):
                                distanceFromAngles[4] += scan.points[n].range 
                                quantity[4] += 1

                    for x in range(len(distanceFromAngles)):
                        if (quantity[x]!=0):
                            print(distanceFromAngles[x]/ quantity[x])
                            distanceFromAngles[x] =  distanceFromAngles[x]/ quantity[x]


                    maxi = 0
                    imax = 0

                    for x in range(len(distanceFromAngles)):
                        maxi = max(maxi, distanceFromAngles[x])
                        imax = x if distanceFromAngles[x] == maxi else imax

                    

                    if imax == 0:
                        self.__angle = -90
                    elif imax == 1:
                        self.__angle = -45
                    elif imax == 2:
                        self.__angle = 0
                    elif imax == 3:
                        self.__angle = 45
                    elif imax == 3:
                        self.__angle = 90
                    else:
                        print(imax)


                else: 
                    print("Failed to get Lidar Data")
                time.sleep(0.25)
        except KeyboardInterrupt:
            self.__laser.turnOff()
            self.__laser.disconnecting()

if __name__ == "__main__":
    lidar = LidarSensor()
    lidar.start_scan()
