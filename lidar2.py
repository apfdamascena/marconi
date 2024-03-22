import os
import ydlidar
import time
import csv

start = 2.1
import math

# def converter_radianos_para_graus(angulo_radianos, angulo_referencia_graus):
#     # Calcule a diferença entre o ângulo fornecido em radianos e o ângulo de referência em radianos
#     diferenca_radianos = angulo_radianos - angulo_referencia_radianos
    
#     # Converta a diferença em radianos para graus
#     diferenca_graus = math.degrees(diferenca_radianos)
    
#     # Ajuste o ângulo de acordo com o sentido anti-horário
#     angulo_resultante = angulo_referencia_graus + diferenca_graus
    
#     # Certifique-se de que o ângulo resultante esteja dentro do intervalo [0, 360)
#     angulo_resultante = angulo_resultante % 360
    
#     # Retorne o ângulo resultante
#     return angulo_resultante

def converter_radianos_para_graus(angulo_radianos):

    # Calcule a diferença entre o ângulo fornecido em radianos e o ângulo de referência em radianos
    diferenca_radianos = angulo_radianos 
    
    # Converta a diferença em radianos para graus
    diferenca_graus = math.degrees(diferenca_radianos)
    
    # Retorne o ângulo resultante
    return diferenca_graus

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

# def getAngle(angle1, angle2):
#     start = -2.55

#     if angle1 < 0:
#         angle1 = angle1 + 360 
#     if angle2 < 0:
#         angle2 = angle2 + 360 
#     if start < 0:
#         start = start + 6.28
        
#     angle1 = (start+(angle1*3.14/180))%6.28
#     angle2 = (start+(angle2*3.14/180))%6.28

#     maximum = max([angle1,angle2])
#     minimum = min([angle1,angle2])

#     if abs(maximum - minimum) > abs(maximum - 6.28) and abs(maximum - minimum) > abs(minimum - 0):
#         return [maximum, minimum]
    
#     else:
#         return [minimum, maximum]

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
    print(converter_radianos_para_graus(2.65))
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
                        startgrau = converter_radianos_para_graus(start)
                        pointGrau = converter_radianos_para_graus(scan.points[n].angle)

                        if pointGrau > startgrau-95 and pointGrau < startgrau-85:

                            distanceFromAngles[0] += distanceFromAngles[0] + scan.points[n].range
                            quantity[0] += 1
                        elif pointGrau > startgrau-50 and pointGrau < startgrau-40:
                            distanceFromAngles[1] += distanceFromAngles[1] + scan.points[n].range
                            quantity[1] += 1
                        elif pointGrau > startgrau-5 and pointGrau < startgrau + 5 :
                            print(pointGrau)
                            print('aq',scan.points[n].angle)
                            distanceFromAngles[2] += distanceFromAngles[2] + scan.points[n].range
                            quantity[2] += 1
                        elif pointGrau > startgrau+40 and pointGrau < startgrau+50:
                            distanceFromAngles[3] += distanceFromAngles[3] + scan.points[n].range 
                            quantity[3] += 1
                        elif pointGrau > startgrau+85 and pointGrau < startgrau+95:
                            distanceFromAngles[4] += distanceFromAngles[4] + scan.points[n].range 
                            quantity[4] += 1

                    # for x in range(len(distanceFromAngles)-1):
                    #     distanceFromAngles[x] /= quantity[x]

                    print(distanceFromAngles)
                    # found = false
                    # maxi = 0

                    # for x in range(len(distanceFromAngles)-1):
                    #     if distanceFromAngles[x] == 0:
                    #         # mudarVelRobo(floor(x-2)*vel Angular Necessaria pra cada angulo)
                    #         found = True
                    #     else:
                    #         maxi = max(maxi, distanceFromAngles[x])



                else: 
                    print("Failed to get Lidar Data")
                time.sleep(0.25)
        except KeyboardInterrupt:
            laser.turnOff()
            laser.disconnecting()