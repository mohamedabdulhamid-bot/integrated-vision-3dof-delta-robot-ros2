#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String 
import cv2
import time
import math

SCALE_FACTOR = 0.472
CX_CENTER = 640  
CY_CENTER = 360 

# Physical offsets between the camera center and the robot origin (mm)
ROBOT_OFFSET_X = 0.0
ROBOT_OFFSET_Y = 420.0

def pixels_to_mm(cX_pixel, cY_pixel):
    x_mm = (cX_pixel - CX_CENTER) * SCALE_FACTOR
    y_mm = -(cY_pixel - CY_CENTER) * SCALE_FACTOR
    final_x = x_mm + ROBOT_OFFSET_X
    final_y = y_mm + ROBOT_OFFSET_Y
    return round(final_x, 2), round(final_y, 2)

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('camera_sensor_node')

        
        self.publisher_ = self.create_publisher(String, '/detected_items', 10)

        self.cap = cv2.VideoCapture(2)  
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        if not self.cap.isOpened():
            self.get_logger().error("Error: Could not open camera.")

        self.TRIGGER_Y = 450
        self.TOLERANCE_Y = 15
        self.recently_triggered = []
        self.morph_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        self.timer = self.create_timer(0.033, self.timer_callback)

        self.get_logger().info("Camera Node Initialized. Waiting for objects...")

    def classify_shape_robust(self, contour):
        area = cv2.contourArea(contour)
        if area == 0: return "Unknown"
        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        hull_perimeter = cv2.arcLength(hull, True)
        if hull_perimeter == 0: return "Unknown"
        circularity = (4 * math.pi * hull_area) / (hull_perimeter * hull_perimeter)
        (x, y), radius = cv2.minEnclosingCircle(contour)
        perfect_circle_area = math.pi * (radius ** 2)
        fill_ratio = area / perfect_circle_area if perfect_circle_area > 0 else 0

        if circularity > 0.80 and fill_ratio > 0.80:
            return "Circle"
        rect = cv2.minAreaRect(contour)
        (box_x, box_y), (box_w, box_h), angle = rect
        if box_w == 0 or box_h == 0: return "Unknown"
        aspect_ratio = max(box_w, box_h) / min(box_w, box_h)
        if 1.0 <= aspect_ratio <= 1.15:
            return "Square"
        else:
            return "Rectangle"

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret: return
        current_time = time.time()
        self.recently_triggered = [pt for pt in self.recently_triggered if current_time - pt[1] < 2.0]

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)
        _, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, self.morph_kernel)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, self.morph_kernel)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cv2.line(frame, (0, self.TRIGGER_Y), (1280, self.TRIGGER_Y), (0, 255, 255), 2)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 15000:
                shape_name = self.classify_shape_robust(cnt)
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                else: continue

                cX_mm, cY_mm = pixels_to_mm(cX, cY)
                cv2.drawContours(frame, [cnt], 0, (0, 255, 0), 2)
                cv2.drawMarker(frame, (cX, cY), (255, 0, 0), cv2.MARKER_CROSS, 20, 2)

                if (self.TRIGGER_Y - self.TOLERANCE_Y) <= cY <= (self.TRIGGER_Y + self.TOLERANCE_Y):
                    already_triggered = False
                    for pt in self.recently_triggered:
                        if abs(cX - pt[0]) < 50:
                            already_triggered = True; break

                    if not already_triggered:
                        self.recently_triggered.append((cX, current_time))
                        cv2.circle(frame, (cX, cY), 25, (0, 0, 255), -1)

                        msg = String()
                        msg.data = f"{shape_name},{cX_mm},{cY_mm}"
                        self.publisher_.publish(msg)

                        self.get_logger().info(f"Sent to Brain -> Shape: {shape_name} | X: {cX_mm} | Y: {cY_mm}")

                cv2.putText(frame, f"{shape_name}", (cX + 15, cY - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
                cv2.putText(frame, f"X:{cX_mm} Y:{cY_mm}mm", (cX + 15, cY + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        cv2.imshow("Camera View", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    rclpy.spin(node)
    node.cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()