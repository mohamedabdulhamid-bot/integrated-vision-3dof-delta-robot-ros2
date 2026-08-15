#!/usr/bin/env python3
# ABDULHAMID
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math
import threading
import sys
import time

class KinematicsNode(Node):
    def __init__(self):
        super().__init__('kinematics_node')
        
        self.angles_pub = self.create_publisher(Float32MultiArray, '/target_angles', 10)
        
        # KINEMATIC in (mm) 
        self.f = 602.86  
        self.e = 207.85  
        self.rf = 320.0  
        self.re = 900.0  
        self.horizontal_offset = 50.0
        
        # --- TRAJECTORY SETTINGS ---
        # INCREASE THIS VALUE TO MAKE THE ROBOT MOVE FASTER (e.g., 500.0, 800.0)
        self.ROBOT_SPEED_MM_S = 400.0 
        self.PUBLISH_RATE_HZ = 50.0 
        
        # STATE VARIABLES FOR TRAJECTORY
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = -600.0 
        self.is_first_move = True
        
        self.get_logger().info("Interactive Kinematics Node Started with Trajectory Planning.")
        
        self.cli_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.cli_thread.start()

    def input_loop(self):
        print("\n=================================================")
        print(" DELTA ROBOT KINEMATICS & TRAJECTORY")
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
                
                # PRE-CHECK: Ensure final target is physically reachable
                status, t1_kin, t2_kin, t3_kin = self.inverse_kinematics(target_x, target_y, target_z)
                if status != 0:
                    print(f"\n[ERROR] Target X:{target_x} Y:{target_y} Z:{target_z} is OUT OF BOUNDS.")
                    continue
                
                print(f"\n[TARGET IK] Theta 1: {t1_kin:.2f} | Theta 2: {t2_kin:.2f} | Theta 3: {t3_kin:.2f}")
                
                # VERIFY WITH FORWARD KINEMATICS
                fk_status, fk_x, fk_y, fk_z = self.forward_kinematics(t1_kin, t2_kin, t3_kin)
                if fk_status != 0:
                    print("[ERROR] FK Failed to resolve position from these angles!")
                    continue
                
                error = math.sqrt((target_x-fk_x)**2 + (target_y-fk_y)**2 + (target_z-fk_z)**2)
                if error > 1.0:
                    print(f"[!] WARNING: Kinematics mismatch! Error margin: {error:.4f}mm")
                
                print('----------------------')
                print(f">> Starting smooth movement to X:{target_x} Y:{target_y} Z:{target_z} at {self.ROBOT_SPEED_MM_S} mm/s")
                
                # EXECUTE SMOOTH TRAJECTORY
                self.move_smoothly(target_x, target_y, target_z)
                
                print(">> Movement Complete.")
                
            except ValueError:
                print("[!] Invalid input. Only numbers are allowed.")
            except Exception as e:
                print(f"[!] Unexpected error: {e}")

    def move_smoothly(self, target_x, target_y, target_z):
        """Generates a smooth Cartesian straight-line trajectory using Cubic Smoothstep."""
        if self.is_first_move:
            self.current_x = target_x
            self.current_y = target_y
            self.current_z = target_z
            self.is_first_move = False
            return self.publish_ik_point(target_x, target_y, target_z)

        x0, y0, z0 = self.current_x, self.current_y, self.current_z
        
        # Calculate total Cartesian distance
        distance = math.sqrt((target_x - x0)**2 + (target_y - y0)**2 + (target_z - z0)**2)
        
        if distance < 1.0:
            return True

        # Calculate required time and steps based on desired speed
        duration = distance / self.ROBOT_SPEED_MM_S
        steps = int(duration * self.PUBLISH_RATE_HZ)
        if steps < 1: 
            steps = 1
            
        dt = 1.0 / self.PUBLISH_RATE_HZ 

        print(f"[TRAJECTORY] Generating {steps} points. Est. time: {duration:.2f}s")

        for i in range(1, steps + 1):
            t_norm = i / steps 
            
            # Cubic Ease-in-out (Smoothstep) interpolation profile
            s = (t_norm ** 2) * (3.0 - 2.0 * t_norm)
            
            # Calculate intermediate Cartesian point
            ix = x0 + (target_x - x0) * s
            iy = y0 + (target_y - y0) * s
            iz = z0 + (target_z - z0) * s
            
            success = self.publish_ik_point(ix, iy, iz)
            if not success:
                print(f"[!] Trajectory aborted at intermediate point: X:{ix:.1f} Y:{iy:.1f} Z:{iz:.1f}")
                break
                
            time.sleep(dt)

        # Update internal tracking to current position
        self.current_x = target_x
        self.current_y = target_y
        self.current_z = target_z
        return True

    def publish_ik_point(self, x, y, z):
        """Runs IK for a single point, checks hardware bounds, and publishes to ESP32."""
        status, t1, t2, t3 = self.inverse_kinematics(x, y, z)
        if status != 0:
            return False 

        esp_t1 = t1 + self.horizontal_offset
        esp_t2 = t2 + self.horizontal_offset
        esp_t3 = t3 + self.horizontal_offset

        # Hardware safety check
        if esp_t1 < 0 or esp_t2 < 0 or esp_t3 < 0:
            return False 

        msg = Float32MultiArray()
        msg.data = [esp_t1, esp_t2, esp_t3]
        self.angles_pub.publish(msg)
        return True

    def inverse_kinematics(self, x0, y0, z0):
        sqrt3 = math.sqrt(3.0)
        sin120 = sqrt3 / 2.0
        cos120 = -0.5
        
        status1, theta1 = self._calc_angle(x0, y0, z0)
        
        x2 = x0 * cos120 + y0 * sin120
        y2 = y0 * cos120 - x0 * sin120
        status2, theta2 = self._calc_angle(x2, y2, z0)
        
        x3 = x0 * cos120 - y0 * sin120
        y3 = y0 * cos120 + x0 * sin120
        status3, theta3 = self._calc_angle(x3, y3, z0)
        
        if status1 != 0 or status2 != 0 or status3 != 0:
            return -1, 0, 0, 0
            
        return 0, theta1, theta2, theta3

    def _calc_angle(self, x0, y0, z0):
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
        
        t = (self.f - self.e) * math.tan(math.radians(30)) / 2.0 
        
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