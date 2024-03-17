import os
import ydlidar
import time

class LidarSensor:

    def __init__(self):
        self.__laser = None
        self.__setup()

    
    def __setup(self):
        ydlidar.os_init()
        ports = ydlidar.lidarPortList()
        port = "/dev/ydlidar"

        for key, value in ports.items():
            port = value
            print(port)

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


    def init(self):
        ret = laser.initialize()
        if ret:
            ret = laser.turnOn()
            scan = ydlidar.LaserScan()
            while ret and ydlidar.os_isOk():
                r = laser.doProcessSimple(scan) 
                if r:
                    print("Scan received [" , scan.stamp," ]: " , scan.points.size(),
                        "ranges is [ " ,1.0/ scan.config.scan_time," ] Hz") 
                else:
                    print(" Failed to get Lidar Data ")
                time.sleep(0.05)
            laser.turnOff()
            laser.disconnecting() 