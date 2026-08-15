#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import math
import threading
import sys
import time
import csv
from datetime import datetime

class KinematicsNode(Node):
    def __init__(self):
        super().__init__('kinematics_node')
        
        self.angles_pub = self.create_publisher(Float32MultiArray, '/target_angles', 10)
        self.status_sub = self.create_subscription(String, '/robot_status', self.status_callback, 10)
        
        self.robot_done_event = threading.Event()
        
        # --- Data Logging Variables ---
        self.is_logging = False
        self.csv_file = None
        self.csv_writer = None
        self.log_start_time = 0.0
        
        # KINEMATIC in (mm) 
        self.f = 602.86  
        self.e = 207.85  
        self.rf = 320.0  
        self.re = 900.0  
        self.horizontal_offset = 50.0
        
        self.ROBOT_SPEED_MM_S = 100.0
        self.PUBLISH_RATE_HZ = 50.0 
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = -600.0 
        self.is_first_move = True
        
        self.get_logger().info("Ultimate Kinematics Node Started (Auto-Logging Shapes).")
        
        self.cli_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.cli_thread.start()

    def status_callback(self, msg):
        """Listens for DONE signal and DATA records from SerialBridge."""
        text = msg.data.strip()
        
        if "DONE" in text.upper():
            self.robot_done_event.set()
            
        elif text.startswith("DATA:") and self.is_logging:
            try:
                values_str = text.replace("DATA:", "").split(',')
                if len(values_str) == 6 and self.csv_writer is not None:
                    elapsed_time = time.time() - self.log_start_time
                    row = [f"{elapsed_time:.3f}"] + [float(v) for v in values_str]
                    
                    self.csv_writer.writerow(row)
                    self.csv_file.flush() 
                    
                    # Print slightly formatted so it doesn't clutter the screen too much
                    print(f"  [LOG] T:{row[0]:>5}s | M1:{row[2]:>6} | M2:{row[4]:>6} | M3:{row[6]:>6}", end='\r')
            except Exception as e:
                pass

    # ==========================================
    # دوال التسجيل (Start & Stop Logging)
    # ==========================================
    def start_logging(self, shape_name):
        filename = f"delta_{shape_name}_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.csv_file = open(filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Time_s', 'Target_M1', 'Actual_M1', 'Target_M2', 'Actual_M2', 'Target_M3', 'Actual_M3'])
        
        print(f"\n[RECORDING STARTED] Saving data to: {filename}")
        self.log_start_time = time.time()
        self.is_logging = True

    def stop_logging(self):
        self.is_logging = False
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
        print("\n[RECORDING STOPPED] File saved successfully.\n")

    # ==========================================
    # الأشكال الهندسية مع التسجيل التلقائي
    # ==========================================
    def run_rectangle_path(self, z_height):
        print(f"\n[SQUARE] Moving to start point...")
        self.move_smoothly(200.0, 200.0, z_height)
        time.sleep(1.5) # Wait for settling
        
        self.start_logging("square")
        vertices = [(200.0, -200.0), (-200.0, -200.0), (-200.0, 200.0), (200.0, 200.0)]
        
        for iteration in range(1, 4):
            print(f"\n--- Square Iteration {iteration}/3 ---")
            for px, py in vertices:
                self.move_smoothly(px, py, z_height)
                time.sleep(0.5)
                
        time.sleep(1.0) # Record final settling
        self.stop_logging()

    def run_triangle_path(self, z_height):
        print(f"\n[TRIANGLE] Moving to start point...")
        self.move_smoothly(0.0, 200.0, z_height)
        time.sleep(1.5)
        
        self.start_logging("triangle")
        # Equilateral triangle vertices
        vertices = [(173.2, -100.0), (-173.2, -100.0), (0.0, 200.0)]
        
        for iteration in range(1, 4):
            print(f"\n--- Triangle Iteration {iteration}/3 ---")
            for px, py in vertices:
                self.move_smoothly(px, py, z_height)
                time.sleep(0.5)
                
        time.sleep(1.0)
        self.stop_logging()

    def run_circle_path(self, z_height, radius=200.0):
        print(f"\n[CIRCLE] Moving to start point...")
        self.move_smoothly(radius, 0.0, z_height)
        time.sleep(1.5)
        
        self.start_logging("circle")
        for iteration in range(1, 4):
            print(f"\n--- Circle Iteration {iteration}/3 ---")
            points = 24 
            for i in range(1, points + 1):
                angle = math.radians(i * (360.0 / points))
                px = radius * math.cos(angle)
                py = radius * math.sin(angle)
                self.move_smoothly(px, py, z_height)
                
        time.sleep(1.0)
        self.stop_logging()

    def run_hypotenuse_path(self, z_height):
        print(f"\n[LINE] Moving to center...")
        self.move_smoothly(0.0, 0.0, z_height)
        time.sleep(1.5)
        
        self.start_logging("line")
        for iteration in range(1, 4):
            print(f"\n--- Line Iteration {iteration}/3 ---")
            self.move_smoothly(-250.0, 250.0, z_height)
            time.sleep(0.5)
            self.move_smoothly(250, -250.0, z_height)
            time.sleep(0.5)
            
            
        time.sleep(1.0)
        self.stop_logging()

    # ==========================================
    # واجهة الأوامر (CLI)
    # ==========================================
    def input_loop(self):
        print("\n=================================================")
        print(" DELTA ROBOT: AUTO-LOGGING SHAPES MENU")
        print("=================================================")
        print("COMMANDS:")
        print("  - 'rect Z'     : Runs Square (3x) -> Saves CSV.")
        print("  - 'tri Z'      : Runs Triangle (3x) -> Saves CSV.")
        print("  - 'circle Z'   : Runs Circle (3x) -> Saves CSV.")
        print("  - 'line Z'     : Runs Hypotenuse (3x) -> Saves CSV.")
        print("  - 'run_all Z'  : Runs ALL shapes (generates 4 CSVs).")
        print("  - X Y Z        : Move to a specific coordinate (No logging).")
        print("  - 'q'          : Quit.")
        
        while rclpy.ok():
            try:
                user_input = input("\nEnter Command: ").strip()
                
                if user_input.lower() in ['q', 'quit', 'exit', 'cancel']:
                    self.stop_logging()
                    print("Exiting...")
                    rclpy.shutdown()
                    sys.exit(0)
                
                parts = user_input.lower().split(' ')
                
                if parts[0] == 'run_all' and len(parts) == 2:
                    z_height = float(parts[1])
                    self.run_rectangle_path(z_height)
                    time.sleep(2.0)
                    self.run_triangle_path(z_height)
                    time.sleep(2.0)
                    self.run_circle_path(z_height)
                    time.sleep(2.0)
                    self.run_hypotenuse_path(z_height)
                    print("\n⭐⭐ ALL TESTS COMPLETED SUCCESSFULLY ⭐⭐")
                    continue
                
                elif parts[0] == 'rect' and len(parts) == 2:
                    self.run_rectangle_path(float(parts[1]))
                    continue
                    
                elif parts[0] == 'tri' and len(parts) == 2:
                    self.run_triangle_path(float(parts[1]))
                    continue
                    
                elif parts[0] == 'circle' and len(parts) == 2:
                    self.run_circle_path(float(parts[1]))
                    continue
                    
                elif parts[0] == 'line' and len(parts) == 2:
                    self.run_hypotenuse_path(float(parts[1]))
                    continue
                
                numbers = [float(val.strip()) for val in parts]
                if len(numbers) != 3:
                    print("[!] Invalid format.")
                    continue
                    
                target_x, target_y, target_z = numbers
                self.move_smoothly(target_x, target_y, target_z)
                
            except ValueError:
                print("[!] Invalid input.")
            except Exception as e:
                print(f"[!] Unexpected error: {e}")

    # ==========================================
    # دوال الحركة والكينماتكس الأساسية
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
            
            self.publish_ik_point(ix, iy, iz)
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