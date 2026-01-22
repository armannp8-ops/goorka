#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import time

class ImuSerialNode(Node):
    def __init__(self):
        super().__init__('imu_serial_node')
        # CHANGE PORT HERE IF NEEDED
        self.declare_parameter('port', '/dev/ttyUSB0') 
        self.port = self.get_parameter('port').value
        
        self.publisher_ = self.create_publisher(Imu, '/imu', 10)
        self.serial_port = serial.Serial(self.port, 115200, timeout=1)
        
        # Calibration Variables
        self.gyro_x_offset = 0.0
        self.gyro_y_offset = 0.0
        self.gyro_z_offset = 0.0
        self.calibrated = False

        self.get_logger().info(f"Connected to {self.port}. Starting Auto-Calibration...")
        self.calibrate_gyro()
        
        self.timer = self.create_timer(0.01, self.timer_callback)

    def calibrate_gyro(self):
        self.get_logger().info("KEEP DEVICE STILL! Calibrating for 3 seconds...")
        samples = 0
        x_sum = 0.0
        y_sum = 0.0
        z_sum = 0.0
        
        # Read 300 samples to find the 'bias'
        while samples < 300:
            if self.serial_port.in_waiting > 0:
                try:
                    line = self.serial_port.readline().decode('utf-8').strip()
                    data = line.split(',')
                    if len(data) == 6:
                        # Sum up the Gyro values (Indices 3, 4, 5)
                        x_sum += float(data[3])
                        y_sum += float(data[4])
                        z_sum += float(data[5])
                        samples += 1
                except:
                    pass
        
        # Calculate Average Offset
        self.gyro_x_offset = x_sum / samples
        self.gyro_y_offset = y_sum / samples
        self.gyro_z_offset = z_sum / samples
        
        self.get_logger().info(f"Calibration Complete! Offsets -> X:{self.gyro_x_offset:.3f} Y:{self.gyro_y_offset:.3f} Z:{self.gyro_z_offset:.3f}")
        self.calibrated = True

    def timer_callback(self):
        if self.serial_port.in_waiting > 0 and self.calibrated:
            try:
                line = self.serial_port.readline().decode('utf-8').strip()
                data = line.split(',')
                
                if len(data) == 6:
                    msg = Imu()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "imu_link"
                    
                    # Accelerometer (Indices 0, 1, 2)
                    msg.linear_acceleration.x = float(data[0])
                    msg.linear_acceleration.y = float(data[1])
                    msg.linear_acceleration.z = float(data[2])
                    
                    # Gyroscope (Indices 3, 4, 5) - SUBTRACT THE OFFSET
                    msg.angular_velocity.x = float(data[3]) - self.gyro_x_offset
                    msg.angular_velocity.y = float(data[4]) - self.gyro_y_offset
                    msg.angular_velocity.z = float(data[5]) - self.gyro_z_offset # <--- This fixes the drift!
                    
                    msg.orientation_covariance[0] = -1
                    self.publisher_.publish(msg)
            except Exception as e:
                pass 

def main(args=None):
    rclpy.init(args=args)
    node = ImuSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
