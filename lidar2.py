import os
import ydlidar
import time
import csv

NAME_OF_FILE = "lidarPointCloud.csv"

def makeRow(stamp, distance, angle):
    row = {
            "SCAN_STAMP": stamp,
            "DISTANCE": distance, 
            "ANGLE": angle
        }
    return row

def saveDataInACSV(data_rows):
    fileExists = os.path.isfile(NAME_OF_FILE) and os.path.getsize(NAME_OF_FILE) > 0
    with open(NAME_OF_FILE, 'w', newline='') as file:
        fieldnames = ["SCAN_STAMP", "DISTANCE", "ANGLE"]
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        
        if not fileExists:
            writer.writeheader()
            
        for row in data_rows:
            writer.writerow(row)
def setup_LiDAR():
    laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 115200)
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
    laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
    laser.setlidaropt(ydlidar.LidarPropSampleRate, 3)
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)
    laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
    laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
    laser.setlidaropt(ydlidar.LidarPropMaxRange, 16.0)
    laser.setlidaropt(ydlidar.LidarPropMinRange, 0.08)
    laser.setlidaropt(ydlidar.LidarPropIntenstiy, False)

# front_distance = None
# front_angle = None
# back_distance = None
# back_angle = None
# for point in scan.points:
#     if front_distance is None or point.range < front_distance:
#         front_distance = point.range
#         front_angle = point.angle
# for point in scan.points:
#     if back_distance is None or point.range > back_distance:
#         back_distance = point.range
#         back_angle = point.angle

# print("[GPT] Front distance:", front_distance)
# print("[GPT] Front angle:", front_angle)
# print("[GPT] Back distance:", back_distance)
# print("[GPT] Back angle:", back_angle)

if __name__ == "__main__":
    ydlidar.os_init()
    ports = ydlidar.lidarPortList()
    port = "/dev/ydlidar"
    for key, value in ports.items():
        port = value
        print(port)

    laser = ydlidar.CYdLidar()
    setup_LiDAR()
    initialized = laser.initialize()

    
    if initialized:
        initialized = laser.turnOn()
        scan_data = []
        try:
            while initialized and ydlidar.os_isOk():
                scan = ydlidar.LaserScan()
                r = laser.doProcessSimple(scan)

                if r:
                    print("================= Scan received [",scan.stamp,"]: ", "size: ", scan.points.size(), " =================")
                    for n in range(1080):
                        print(f'line {n} range = {scan.points[n].range} and angle = {scan.points[n].angle}')
                else: 
                    print("Failed to get Lidar Data")
                time.sleep(0.25)
        except KeyboardInterrupt:
            laser.turnOff()
            laser.disconnecting()
