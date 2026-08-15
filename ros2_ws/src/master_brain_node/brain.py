#!/usr/bin/env python3
#ABDULHAMID 01110458219
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import math
import threading
import time

class BrainKinematicsNode(Node):
    def __init__(self):
        super().__init__('brain_kinematics_node')
        
        # --- Publishers & Subscribers ---
        self.angles_pub = self.create_publisher(Float32MultiArray, '/target_angles', 10)
        self.valve_pub = self.create_publisher(String, '/valve_cmd', 10)
        self.camera_sub = self.create_subscription(String, '/detected_items', self.camera_callback, 10)
        
        # --- Robot State & Settings ---
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = -600.0 
        self.is_first_move = True
        
        self.f = 602.86; self.e = 207.85; self.rf = 320.0; self.re = 900.0; self.horizontal_offset = 50.0
        self.ROBOT_SPEED_MM_S = 100.0 
        self.PUBLISH_RATE_HZ = 50.0 
        
        # (Z Levels) 
        self.HOVER_Z = -650.0  
        self.PICK_Z = -750.0   
        self.DROP_Z = -800.0   #inside boxes of pakaging
        
        # (Boxes Positions) ---
        self.boxes = {
            "Square": (-200.0, -200.0),
            "Rectangle": (0.0, -200.0),
            "Circle": (200, -200),
            "Unknown": (0.0, 0.0) 
        }

        # (Memory Queue)
        self.items_queue = []
        
        self.get_logger().info("Brain & Kinematics Ready! Waiting for camera data...")
        
        # Start Worker Thread (The Brain logic)
        self.worker_thread = threading.Thread(target=self.process_queue, daemon=True)
        self.worker_thread.start()

    def camera_callback(self, msg):
        """collect and store data from the camera"""
        data = msg.data.split(',')
        if len(data) == 3:
            shape = data[0]
            x = float(data[1])
            y = float(data[2])
            
            #add the plate to the waiting 
            self.items_queue.append({"shape": shape, "x": x, "y": y})
            self.get_logger().info(f"[BRAIN] Added to Queue: {shape} at ({x}, {y}). Total items pending: {len(self.items_queue)}")

    def process_queue(self):
        """The Brain: collects the plates in order"""
        while rclpy.ok():
            if len(self.items_queue) > 0:
                # first plate
                current_item = self.items_queue.pop(0)
                self.execute_pick_and_place(current_item)
            else:
                # if there is no plates return to check again
                time.sleep(0.1)

    def set_valve(self, state):
        msg = String()
        msg.data = "V1" if state else "V0"
        self.valve_pub.publish(msg)

    def execute_pick_and_place(self, item):
        """one cycle for pick and drop the plate"""
        shape = item['shape']
        pick_x = item['x']
        pick_y = item['y']
        
        # recognize the box according to the shape
        drop_x, drop_y = self.boxes.get(shape, (0.0, 0.0))
        
        print(f"\n--- [EXECUTING TASK] Picking {shape} ---")
        
        
        self.move_smoothly(pick_x, pick_y, self.HOVER_Z)
        time.sleep(0.2)
        
        
        self.move_smoothly(pick_x, pick_y, self.PICK_Z)
        self.set_valve(True)
        time.sleep(0.8) 
        
        
        self.move_smoothly(pick_x, pick_y, self.HOVER_Z)
        
        
        self.move_smoothly(drop_x, drop_y, self.HOVER_Z)
        
        
        self.move_smoothly(drop_x, drop_y, self.DROP_Z)
        self.set_valve(False)
        time.sleep(0.5) 
        
        self.move_smoothly(drop_x, drop_y, self.HOVER_Z)
        
        print(f"--- [TASK DONE] {shape} placed in box! ---\n")

    # ==========================================
    # Movement & Kinematics Core
    # ==========================================
    def move_smoothly(self, target_x, target_y, target_z):
        if self.is_first_move:
            self.current_x = target_x; self.current_y = target_y; self.current_z = target_z
            self.is_first_move = False
            return self.publish_ik_point(target_x, target_y, target_z)

        x0, y0, z0 = self.current_x, self.current_y, self.current_z
        distance = math.sqrt((target_x - x0)**2 + (target_y - y0)**2 + (target_z - z0)**2)
        if distance < 1.0: return True

        duration = distance / self.ROBOT_SPEED_MM_S
        steps = int(duration * self.PUBLISH_RATE_HZ)
        if steps < 1: steps = 1
        dt = 1.0 / self.PUBLISH_RATE_HZ 

        for i in range(1, steps + 1):
            t_norm = i / steps 
            s = (t_norm ** 2) * (3.0 - 2.0 * t_norm) 
            ix = x0 + (target_x - x0) * s
            iy = y0 + (target_y - y0) * s
            iz = z0 + (target_z - z0) * s
            self.publish_ik_point(ix, iy, iz)
            time.sleep(dt)

        self.current_x = target_x; self.current_y = target_y; self.current_z = target_z
        return True

    def publish_ik_point(self, x, y, z):
        status, t1, t2, t3 = self.inverse_kinematics(x, y, z)
        if status != 0: return False 
        esp_t1 = t1 + self.horizontal_offset; esp_t2 = t2 + self.horizontal_offset; esp_t3 = t3 + self.horizontal_offset
        if esp_t1 < 0 or esp_t2 < 0 or esp_t3 < 0: return False 
        msg = Float32MultiArray(); msg.data = [esp_t1, esp_t2, esp_t3]; self.angles_pub.publish(msg)
        return True

    def inverse_kinematics(self, x0, y0, z0):
        sqrt3 = math.sqrt(3.0); sin120 = sqrt3 / 2.0; cos120 = -0.5
        status1, theta1 = self._calc_angle(x0, y0, z0)
        x2 = x0 * cos120 + y0 * sin120; y2 = y0 * cos120 - x0 * sin120
        status2, theta2 = self._calc_angle(x2, y2, z0)
        x3 = x0 * cos120 - y0 * sin120; y3 = y0 * cos120 + x0 * sin120
        status3, theta3 = self._calc_angle(x3, y3, z0)
        if status1 != 0 or status2 != 0 or status3 != 0: return -1, 0, 0, 0
        return 0, theta1, theta2, theta3

    def _calc_angle(self, x0, y0, z0):
        tan30 = 1.0 / math.sqrt(3.0)
        y1 = -0.5 * tan30 * self.f; y0_offset = y0 - 0.5 * tan30 * self.e 
        a = (x0**2 + y0_offset**2 + z0**2 + self.rf**2 - self.re**2 - y1**2) / (2.0 * z0)
        b = (y1 - y0_offset) / z0
        d = -(a + b * y1)**2 + self.rf**2 * (b**2 + 1)
        if d < 0: return -1, 0
        yj = (y1 - a * b - math.sqrt(d)) / (b**2 + 1)
        zj = a + b * yj
        theta = math.degrees(math.atan(-zj / (y1 - yj)))
        if yj > y1: theta += 180.0
        return 0, theta

def main(args=None):
    rclpy.init(args=args)
    node = BrainKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()