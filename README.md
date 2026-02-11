python3 ~/map_ws/src/imu_driver.py #Imu code launch file

source ~/map_ws/install/setup.bash
ros2 launch sllidar_ros2 sllidar_c1_launch.py serial_port:=/dev/ttyUSB1 #lidar launch code

# 1. Glue the Lidar to the Robot
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser &
# 2. Glue the IMU to the Robot
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link imu_link &
# 3. Start Cartographer (Your exact command)
ros2 run cartographer_ros cartographer_node \
     -configuration_directory /home/arman/map_ws/src/cartographer_config \
     -configuration_basename lidar_imu_2d.lua \
     --ros-args -r scan:=/scan -r imu:=/imu
     
ros2 run cartographer_ros cartographer_occupancy_grid_node -resolution 0.05 #cartographer res

sudo chmod 666 /dev/ttyUSB1

ros2 daemon stop
rm -rf /dev/shm/fastrtps*
ros2 daemon start #IMU ghost

# Set the port to the correct baud rate (Change USB0 to USB1 if needed)
stty -F /dev/ttyUSB0 115200 raw
# Read the raw output
cat /dev/ttyUSB0 #PUBLISHING CODE FOR IMU

ls /dev/ttyUSB* /dev/ttyACM* #list USB devices

cat /dev/ttyUSB1 #USB listener
