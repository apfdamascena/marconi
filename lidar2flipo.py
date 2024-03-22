import os
import ydlidar
import time
import csv

start = 2.1

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

def getAngle(angle1, angle2):
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
        try:
            while initialized and ydlidar.os_isOk():
                scan = ydlidar.LaserScan()
                r = laser.doProcessSimple(scan)

                startPoints = 0
                pointsQuantity = 0
              
                if r:
                    print("================= Scan received [",scan.stamp,"]: ", "size: ", scan.points.size(), " =================")
                    distanceFromAngles = [0, 0, 0, 0, 0]
                    quantity = [0, 0, 0, 0, 0]

                    for n in range(scan.points.size()): 
                        if scan.points[n].angle > getAngle(-95,-85)[0] and scan.points[n].angle < getAngle(-95,-85)[1]:
                            distanceFromAngles[0] += distanceFromAngles[0] + scan.points[n].range
                            quantity[0] += 1

                        elif scan.points[n].angle > getAngle(-50,-40)[0] and scan.points[n].angle < getAngle(-50,-40)[1]:
                            distanceFromAngles[1] += distanceFromAngles[1] + scan.points[n].range
                            quantity[1] += 1

                        elif scan.points[n].angle > getAngle(-5,5)[0] and scan.points[n].angle < getAngle(-5,5)[1]:
                            distanceFromAngles[2] += distanceFromAngles[2] + scan.points[n].range
                            quantity[2] += 1
                        elif scan.points[n].angle > getAngle(50,40)[0] and scan.points[n].angle < getAngle(50,40)[1]:
                            distanceFromAngles[3] += distanceFromAngles[3] + scan.points[n].range 
                            quantity[3] += 1
                        elif scan.points[n].angle > getAngle(95,85)[0] and scan.points[n].angle < getAngle(95,85)[1]:
                            distanceFromAngles[4] += distanceFromAngles[4] + scan.points[n].range 
                            quantity[4] += 1

                    for x in range(len(distanceFromAngles)-1):
                        distanceFromAngles[x] /= quantity[x]
                    print(distanceFromAngles)

                else: 
                    print("Failed to get Lidar Data")
                time.sleep(0.25)
        except KeyboardInterrupt:
            laser.turnOff()
            laser.disconnecting()