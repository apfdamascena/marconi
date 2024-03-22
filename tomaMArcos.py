from ydlidar import CYdLidar, LaserConfig, LaserScan

# Initialize LiDAR object
lidar = CYdLidar()
lidar.initialize()

# Configure LiDAR
config = LaserConfig()
config.min_angle = -180
config.max_angle = 180
lidar.setlidaropt(LaserConfig().max_range, 16)

# Turn on LiDAR
lidar.turnOn()

# Read LiDAR data
scan = LaserScan()
lidar.doProcessSimple(scan)

# Find front of LiDAR
front_distance = None
front_angle = None
for point in scan.points:
    if front_distance is None or point.range < front_distance:
        front_distance = point.range
        front_angle = point.angle

print("Front distance:", front_distance)
print("Front angle:", front_angle)

# Turn off LiDAR
lidar.turnOff()
