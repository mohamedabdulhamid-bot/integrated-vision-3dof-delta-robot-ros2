import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import serial
import threading
import sys

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')
        
        # Serial Port Configuration
        self.port = '/dev/ttyUSB0'  
        self.baudrate = 115200
        
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.get_logger().info(f"Successfully connected to ESP32 on {self.port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to {self.port}. Is the ESP32 plugged in?")
            sys.exit(1)

        # Topic Subscriptions
        self.angle_sub = self.create_subscription(
            Float32MultiArray,
            '/target_angles',
            self.angle_callback,
            10)
            
        self.valve_sub = self.create_subscription(
            String,
            '/valve_cmd',
            self.valve_callback,
            10)

        # Background thread to continuously read incoming messages from ESP32
        self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
        self.read_thread.start()

        self.get_logger().info("Serial Bridge Node is running and listening for commands...")

    def angle_callback(self, msg):
        """Receives angles from IK node and sends them as a comma-separated string to ESP32."""
        if len(msg.data) == 3:
            # Format exactly as expected by the ESP32 parsing logic: "M1,M2,M3\n"
            command = f"{msg.data[0]:.2f},{msg.data[1]:.2f},{msg.data[2]:.2f}\n"
            self.serial_conn.write(command.encode('utf-8'))

    def valve_callback(self, msg):
        """Receives valve commands (V1/V0) and sends them to ESP32."""
        # Format exactly as expected by the ESP32 String evaluation
        command = f"{msg.data}\n"
        self.serial_conn.write(command.encode('utf-8'))

    def serial_read_loop(self):
        """Reads incoming serial data from ESP32 (like DONE states and angles) and logs it."""
        while rclpy.ok():
            try:
                if self.serial_conn.in_waiting > 0:
                    incoming = self.serial_conn.readline().decode('utf-8').strip()
                    if incoming:
                        # Log the ESP32 output to the ROS 2 terminal for monitoring
                        self.get_logger().info(f"[ESP32] {incoming}")
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
                break

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanly close the serial port upon shutdown
        if hasattr(node, 'serial_conn') and node.serial_conn.is_open:
            node.serial_conn.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()