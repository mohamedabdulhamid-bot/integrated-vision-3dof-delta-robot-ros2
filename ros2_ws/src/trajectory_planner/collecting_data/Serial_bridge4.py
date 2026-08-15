#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import serial
import threading
import time

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')
        
        # --- Subscriptions ---
        self.subscription = self.create_subscription(
            Float32MultiArray, '/target_angles', self.angles_callback, 10)
            
        self.valve_sub = self.create_subscription(
            String, '/valve_cmd', self.valve_callback, 10)
        
        # --- Publishers ---
        self.current_angles_pub = self.create_publisher(Float32MultiArray, '/current_angles', 10)
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        self.valve_status_pub = self.create_publisher(String, '/valve_status', 10)
        
        self.serial_port = '/dev/ttyUSB0'
        self.baud_rate = 115200
        
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            self.get_logger().info(f'Connected to ESP32 on {self.serial_port}')
            
            self.read_thread = threading.Thread(target=self.read_from_serial, daemon=True)
            self.read_thread.start()
            
        except Exception as e:
            self.get_logger().error(f'Failed to connect to ESP32: {e}')
            self.ser = None

    def angles_callback(self, msg):
        """Sends target angles to ESP32."""
        if self.ser and self.ser.is_open:
            t1, t2, t3 = msg.data
            command = f"{t1:.2f},{t2:.2f},{t3:.2f}\n"
            self.ser.write(command.encode('utf-8'))

    def valve_callback(self, msg):
        """Sends valve ON/OFF commands (V1/V0) to ESP32."""
        if self.ser and self.ser.is_open:
            command = f"{msg.data}\n"
            self.ser.write(command.encode('utf-8'))

    def read_from_serial(self):
        """Reads incoming serial data and publishes to respective topics."""
        while rclpy.ok():
            if self.ser and self.ser.in_waiting > 0:
                try:
                    line = self.ser.readline().decode('utf-8').strip()
                    
                    # --- تعديل جديد: التقاط سطر البيانات DATA وتمريره كما هو للـ Data Logger ---
                    if line.startswith("DATA:"):
                        msg = String()
                        msg.data = line
                        self.status_pub.publish(msg)
                    
                    # Parse current angles (if you ever revert to ANG:)
                    elif line.startswith("ANG:"):
                        data_str = line.replace("ANG:", "").strip()
                        angles = [float(x) for x in data_str.split(',')]
                        
                        msg = Float32MultiArray()
                        msg.data = angles
                        self.current_angles_pub.publish(msg)
                    
                    # Parse motion completion status
                    elif line == "DONE":
                        msg = String()
                        msg.data = "REACHED_TARGET"
                        self.status_pub.publish(msg)
                        
                        # Also publish the raw "DONE" for the data logger event flag
                        msg_raw = String()
                        msg_raw.data = "DONE"
                        self.status_pub.publish(msg_raw)
                        
                        self.get_logger().info('Robot reached target!')
                        
                    # Parse Solenoid Valve ON confirmation
                    elif "Solenoid Valve ON" in line:
                        msg = String()
                        msg.data = "ON"
                        self.valve_status_pub.publish(msg)
                        self.get_logger().info('Valve Status: ON')
                        
                    # Parse Solenoid Valve OFF confirmation
                    elif "Solenoid Valve OFF" in line:
                        msg = String()
                        msg.data = "OFF"
                        self.valve_status_pub.publish(msg)
                        self.get_logger().info('Valve Status: OFF')
                        
                except Exception as e:
                    pass
            time.sleep(0.01) 

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()