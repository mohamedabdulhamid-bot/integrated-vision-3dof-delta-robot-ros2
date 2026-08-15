#!/usr/bin/env python3
#ABDULHAMID
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math
import threading
import sys

class KinematicsNode(Node):
    def __init__(self):
        super().__init__('kinematics_node')
        
        # Publisher to match your Serial Bridge subscription
        self.angles_pub = self.create_publisher(Float32MultiArray, '/target_angles', 10)
        
        # KINEMATIC in (mm) 
        self.f = 602.86  # Base triangle side
        self.e = 207.85  # End effector triangle side
        self.rf = 320.0  # Bicep length
        self.re = 900.0  # Forearm length
        
        # OFFSET CONFIGURATION 
        # the upper arm is 50 degrees ABOVE horizontal.
        # horizontal = 0 deg.
        self.horizontal_offset = 50.0
        
        self.get_logger().info("Interactive Kinematics Node Started.")
        
        # Start CLI in a background thread so ROS can spin
        self.cli_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.cli_thread.start()

    def input_loop(self):
        """Interactive loop to enter X, Y, Z continuously."""
        print("\n=================================================")
        print(" DELTA ROBOT KINEMATICS")
        print("=================================================")
        print("Enter target coordinates (or type 'q' to quit)")
        
        while rclpy.ok():
            try:
                user_input = input("\nEnter X Y Z (e.g., 0 0 -500): ")
                if user_input.lower().strip() in ['q', 'quit', 'exit', 'cancel']:
                    print("Exiting...")
                    rclpy.shutdown()
                    sys.exit(0)
                
                
                numbers = [float(val.strip()) for val in user_input.split(' ')]
                if len(numbers) != 3:
                    print("[!] Invalid format. Please enter exactly 3 numbers separated by Space.")
                    continue
                    
                target_x, target_y, target_z = numbers
                
                # RUN INVERSE KINEMATICS
                status, t1_kin, t2_kin, t3_kin = self.inverse_kinematics(target_x, target_y, target_z)
                
                if status != 0:
                    print(f"\n[ERROR] Target X:{target_x} Y:{target_y} Z:{target_z} is OUT OF BOUNDS.")
                    continue
                
                print(f"\n IK RESULTS (True Mathematical Angles) ---")
                print(f"Theta 1: {t1_kin:.2f}° | Theta 2: {t2_kin:.2f}° | Theta 3: {t3_kin:.2f}°")
                
                # RUN FORWARD KINEMATICS TO VERIFY
                fk_status, fk_x, fk_y, fk_z = self.forward_kinematics(t1_kin, t2_kin, t3_kin)
                
                if fk_status != 0:
                    print("\n[ERROR] FK Failed to resolve position from these angles!")
                    continue
                print('----------------------')
                print(f" FK VERIFICATION")
                print(f"Expected: X:{target_x:.2f}, Y:{target_y:.2f}, Z:{target_z:.2f}")
                print(f"Actual FK: X:{fk_x:.2f}, Y:{fk_y:.2f}, Z:{fk_z:.2f}")
                
                # Check error margin
                error = math.sqrt((target_x-fk_x)**2 + (target_y-fk_y)**2 + (target_z-fk_z)**2)
                if error > 1.0:
                    print(f"[!] WARNING: Kinematics mismatch! Error margin: {error:.4f}mm")
                else:
                    print("[SUCCESS] FK perfectly matches target IK parameters!")
                
                # 3. APPLY PHYSICAL OFFSETS
                # Convert kinematic angles to ESP hardware angles
                esp_t1 = t1_kin + self.horizontal_offset
                esp_t2 = t2_kin + self.horizontal_offset
                esp_t3 = t3_kin + self.horizontal_offset
                print('----------------------')
                print(f" HARDWARE COMMAND (ESP32 Mapping) ---")
                print(f"Mapped ESP Angles: M1:{esp_t1:.2f} | M2:{esp_t2:.2f} | M3:{esp_t3:.2f}")
                
                # Safety checks (Cannot rotate negative)
                if esp_t1 < 0 or esp_t2 < 0 or esp_t3 < 0:
                    print("\n[BLOCKED] Move violates limit switch bounds (ESP angle < 0)!")
                    print("The robot cannot physically reach this mathematical coordinate.")
                    continue
                
                # 4. PUBLISH TO BRIDGE NODE
                msg = Float32MultiArray()
                msg.data = [esp_t1, esp_t2, esp_t3]
                self.angles_pub.publish(msg)
                print('----------------------')
                print(">> Published angles to /target_angles successfully.")
                
            except ValueError:
                print("[!] Invalid input. Only numbers are allowed.")
            except Exception as e:
                print(f"[!] Unexpected error: {e}")

    def inverse_kinematics(self, x0, y0, z0):
        sqrt3 = math.sqrt(3.0)
        sin120 = sqrt3 / 2.0
        cos120 = -0.5
        
        status1, theta1 = self._calc_angle(x0, y0, z0)
        
        # Rotate 120 degrees for joint 2
        x2 = x0 * cos120 + y0 * sin120
        y2 = y0 * cos120 - x0 * sin120
        status2, theta2 = self._calc_angle(x2, y2, z0)
        
        # Rotate -120 degrees for joint 3
        x3 = x0 * cos120 - y0 * sin120
        y3 = y0 * cos120 + x0 * sin120
        status3, theta3 = self._calc_angle(x3, y3, z0)
        
        if status1 != 0 or status2 != 0 or status3 != 0:
            return -1, 0, 0, 0
            
        return 0, theta1, theta2, theta3

    def _calc_angle(self, x0, y0, z0):
        """IK Helper."""
        tan30 = 1.0 / math.sqrt(3.0)
        y1 = -0.5 * tan30 * self.f 
        y0_offset = y0 - 0.5 * tan30 * self.e 
        
        a = (x0**2 + y0_offset**2 + z0**2 + self.rf**2 - self.re**2 - y1**2) / (2.0 * z0)
        b = (y1 - y0_offset) / z0
        
        d = -(a + b * y1)**2 + self.rf**2 * (b**2 + 1)
        
        if d < 0:
            return -1, 0
            
        yj = (y1 - a * b - math.sqrt(d)) / (b**2 + 1)
        zj = a + b * yj
        
        theta = math.degrees(math.atan(-zj / (y1 - yj)))
        if yj > y1:
            theta += 180.0
            
        return 0, theta

    def forward_kinematics(self, theta1, theta2, theta3):
        t1 = math.radians(theta1)
        t2 = math.radians(theta2)
        t3 = math.radians(theta3)
        
        # Distance calculation variables from documented formulas
        t = (self.f - self.e) * math.tan(math.radians(30)) / 2.0 
        
        # Find coordinates of J1', J2', J3'
        x1 = 0.0
        y1 = -(t + self.rf * math.cos(t1))
        z1 = -self.rf * math.sin(t1)
        
        x2 = (t + self.rf * math.cos(t2)) * math.cos(math.radians(30))
        y2 = (t + self.rf * math.cos(t2)) * math.sin(math.radians(30))
        z2 = -self.rf * math.sin(t2)
        
        x3 = -(t + self.rf * math.cos(t3)) * math.cos(math.radians(30))
        y3 = (t + self.rf * math.cos(t3)) * math.sin(math.radians(30))
        z3 = -self.rf * math.sin(t3)
        
        w1 = x1**2 + y1**2 + z1**2
        w2 = x2**2 + y2**2 + z2**2
        w3 = x3**2 + y3**2 + z3**2
        
        d = (y2 - y1) * x3 - (y3 - y1) * x2
        if d == 0:
            return -1, 0, 0, 0
            
        a1 = ((z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)) / d
        a2 = -((z2 - z1) * x3 - (z3 - z1) * x2) / d
        b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / (2.0 * d)
        b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / (2.0 * d)
        
        A = a1**2 + a2**2 + 1.0
        B = 2.0 * (a1 * (b1 - x1) + a2 * (b2 - y1) - z1)
        C = (b1 - x1)**2 + (b2 - y1)**2 + z1**2 - self.re**2
        
        discriminant = B**2 - 4.0 * A * C
        if discriminant < 0:
            return -1, 0, 0, 0
            
        # We take the negative root because the end effector is below the base
        z0 = (-B - math.sqrt(discriminant)) / (2.0 * A)
        x0 = a1 * z0 + b1
        y0 = a2 * z0 + b2
        
        return 0, x0, y0, z0

def main(args=None):
    rclpy.init(args=args)
    node = KinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()