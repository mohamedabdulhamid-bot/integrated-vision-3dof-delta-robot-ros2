#!/usr/bin/env python3
#ABDULHAMID 
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
        self.ROBOT_SPEED_MM_S = 100.0
        self.PUBLISH_RATE_HZ = 50.0 
        
        # STATE VARIABLES FOR TRAJECTORY
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = -600.0 
        self.is_first_move = True
        
        self.get_logger().info("Interactive Kinematics Node Started with Trajectory Planning.")
        
        self.cli_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.cli_thread.start()

    # ==========================================
    # (Square) 
    # ==========================================
    def run_rectangle_path(self, z_height):
        vertices = [
            (200.0, 200.0),
            (200.0, -200.0),
            (-200.0, -200.0),
            (-200.0, 200.0),
            (200.0, 200.0) 
        ]
        
        print(f"\n[SQUARE TEST] Starting 3 iterations at Z: {z_height}")
       
        self.move_smoothly(200.0, 200.0, z_height)
        time.sleep(1.0)
        
        for iteration in range(1, 10): 
            print(f"\n--- Square Iteration {iteration}/3 ---")
            for px, py in vertices:
                print(f" -> Moving to Vertex: X:{px}, Y:{py}")
                success = self.move_smoothly(px, py, z_height)
                if not success:
                    print("[!] Path aborted due to hardware bounds.")
                    return
                time.sleep(0.5) 
        print("\n[SQUARE TEST] Complete!")

    # ==========================================
    # 2. circle
    # ==========================================
    def run_circle_path(self, z_height, radius=150.0):
        print(f"\n[CIRCLE TEST] Starting 3 iterations at Z: {z_height} with Radius {radius}")
        
        
        self.move_smoothly(radius, 0.0, z_height)
        time.sleep(1.0)
        
        for iteration in range(1, 10): #10 ilterations
            print(f"\n--- Circle Iteration {iteration}/3 ---")
            points = 24 
            for i in range(1, points + 1):
                angle = math.radians(i * (360.0 / points))
                px = radius * math.cos(angle)
                py = radius * math.sin(angle)
                
                success = self.move_smoothly(px, py, z_height)
                if not success:
                    print("[!] Path aborted due to hardware bounds.")
                    return
        print("\n[CIRCLE TEST] Complete!")

    # ==========================================
    # (Hypotenuse) 
    # ==========================================
    def run_hypotenuse_path(self, z_height):
        print(f"\n[HYPOTENUSE TEST] Starting 3 iterations at Z: {z_height}")
        
        for iteration in range(1, 4):  
            print(f"\n--- Hypotenuse Iteration {iteration}/3 ---")
            print(" -> Moving to Center (0, 0)")
            self.move_smoothly(0.0, 0.0, z_height)
            time.sleep(0.5)
            
            print(" -> Moving to Far Point (200, 200)")
            success = self.move_smoothly(200.0, 200.0, z_height)
            if not success:
                print("[!] Path aborted due to hardware bounds.")
                return
            time.sleep(0.5)
            
        # return to origin
        self.move_smoothly(0.0, 0.0, z_height)
        print("\n[HYPOTENUSE TEST] Complete!")

    # ==========================================
    def input_loop(self):
        print("\n=================================================")
        print(" DELTA ROBOT: SHAPES & TRAJECTORY TEST")
        print("=================================================")
        print("COMMANDS:")
        print("  - 'run_all Z'  : Runs Square, Circle, and Hypotenuse (3x each).")
        print("  - 'square Z'     : Runs only the Square (10x).")
        print("  - 'circle Z'   : Runs only the Circle (10x).")
        print("  - 'line Z'     : Runs only the Hypotenuse (4x).")
        print("  - X Y Z        : Move to a specific coordinate.")
        print("  - 'q'          : Quit.")
        
        while rclpy.ok():
            try:
                user_input = input("\nEnter Command: ").strip()
                
                if user_input.lower() in ['q', 'quit', 'exit', 'cancel']:
                    print("Exiting...")
                    rclpy.shutdown()
                    sys.exit(0)
                
                parts = user_input.lower().split(' ')
                
                if parts[0] == 'run_all' and len(parts) == 2:
                    z_height = float(parts[1])
                    self.run_rectangle_path(z_height)
                    time.sleep(2.0)
                    self.run_circle_path(z_height)
                    time.sleep(2.0)
                    self.run_hypotenuse_path(z_height)
                    print("\n⭐⭐ ALL TESTS COMPLETED SUCCESSFULLY ⭐⭐")
                    continue
                
                # square
                elif parts[0] == 'square' and len(parts) == 2:
                    self.run_rectangle_path(float(parts[1]))
                    continue
                    
                # circle
                elif parts[0] == 'circle' and len(parts) == 2:
                    self.run_circle_path(float(parts[1]))
                    continue
                    
                # hyp
                elif parts[0] == 'line' and len(parts) == 2:
                    self.run_hypotenuse_path(float(parts[1]))
                    continue
                
                # point to point
                numbers = [float(val.strip()) for val in parts]
                if len(numbers) != 3:
                    print("[!] Invalid format.")
                    continue
                    
                target_x, target_y, target_z = numbers
                
                status, t1_kin, t2_kin, t3_kin = self.inverse_kinematics(target_x, target_y, target_z)
                if status != 0:
                    print(f"\n[ERROR] Target X:{target_x} Y:{target_y} Z:{target_z} is OUT OF BOUNDS.")
                    continue
                
                print(f">> Moving smoothly to X:{target_x} Y:{target_y} Z:{target_z}")
                self.move_smoothly(target_x, target_y, target_z)
                print(">> Movement Complete.")
                
            except ValueError:
                print("[!] Invalid input. Only numbers are allowed.")
            except Exception as e:
                print(f"[!] Unexpected error: {e}")

    # ==========================================
    # traj
    # ==========================================
    def move_smoothly(self, target_x, target_y, target_z):
        if self.is_first_move:
            self.current_x = target_x
            self.current_y = target_y
            self.current_z = target_z
            self.is_first_move = False
            return self.publish_ik_point(target_x, target_y, target_z)

        x0, y0, z0 = self.current_x, self.current_y, self.current_z
        
        distance = math.sqrt((target_x - x0)**2 + (target_y - y0)**2 + (target_z - z0)**2)
        
        if distance < 1.0:
            return True

        duration = distance / self.ROBOT_SPEED_MM_S
        steps = int(duration * self.PUBLISH_RATE_HZ)
        if steps < 1: 
            steps = 1
            
        dt = 1.0 / self.PUBLISH_RATE_HZ 

        for i in range(1, steps + 1):
            t_norm = i / steps 
            s = (t_norm ** 2) * (3.0 - 2.0 * t_norm)
            
            ix = x0 + (target_x - x0) * s
            iy = y0 + (target_y - y0) * s
            iz = z0 + (target_z - z0) * s
            
            success = self.publish_ik_point(ix, iy, iz)
            if not success:
                return False
                
            time.sleep(dt)

        self.current_x = target_x
        self.current_y = target_y
        self.current_z = target_z
        return True

    def publish_ik_point(self, x, y, z):
        status, t1, t2, t3 = self.inverse_kinematics(x, y, z)
        if status != 0:
            return False 

        esp_t1 = t1 + self.horizontal_offset
        esp_t2 = t2 + self.horizontal_offset
        esp_t3 = t3 + self.horizontal_offset

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