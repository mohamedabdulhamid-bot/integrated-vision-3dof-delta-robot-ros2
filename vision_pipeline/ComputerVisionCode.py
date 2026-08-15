import cv2
import numpy as np
import time
import math

# --- CALIBRATION CONSTANTS ---
SCALE_FACTOR = 0.472 # mm per pixel (Derived from physical calibration)
CX_CENTER = 640      # Center pixel X of 1280x720 frame
CY_CENTER = 360      # Center pixel Y of 1280x720 frame

# Physical offsets between the camera center and the robot origin (mm)
ROBOT_OFFSET_X = 0.0 
ROBOT_OFFSET_Y = 420.0 

def pixels_to_mm(cX_pixel, cY_pixel):
    """
    Transforms image pixel coordinates to physical robot coordinates in millimeters.
    """
    x_mm = (cX_pixel - CX_CENTER) * SCALE_FACTOR
    
    # Negative sign inverts the Image Y-axis to match the Robot's Cartesian Y-axis
    y_mm = -(cY_pixel - CY_CENTER) * SCALE_FACTOR 
    
    final_x = x_mm + ROBOT_OFFSET_X
    final_y = y_mm + ROBOT_OFFSET_Y
    
    return round(final_x, 2), round(final_y, 2)

def classify_shape_robust(contour):
    """
    Evaluates geometry using Convex Hull, Enclosing Circle, and Rotated Rectangles
    to handle jagged physical materials and orientation-independent shapes.
    """
    area = cv2.contourArea(contour)
    if area == 0:
        return "Unknown"
        
    # --- Smoothing (Coastline Paradox Fix) ---
    hull = cv2.convexHull(contour)
    hull_area = cv2.contourArea(hull)
    hull_perimeter = cv2.arcLength(hull, True)
    
    if hull_perimeter == 0:
        return "Unknown"
        
    circularity = (4 * math.pi * hull_area) / (hull_perimeter * hull_perimeter)
    
    # --- Minimum Enclosing Circle Ratio ---
    (x, y), radius = cv2.minEnclosingCircle(contour)
    perfect_circle_area = math.pi * (radius ** 2)
    fill_ratio = area / perfect_circle_area if perfect_circle_area > 0 else 0
    
    # 1. Check for Circle
    if circularity > 0.80 and fill_ratio > 0.80:
        return "Circle"
        
    # 2. Check for Square vs Rectangle (Rotational Invariance Fix)
    rect = cv2.minAreaRect(contour)
    (box_x, box_y), (box_w, box_h), angle = rect
    
    if box_w == 0 or box_h == 0:
        return "Unknown"
        
    # Ensure aspect ratio is always >= 1.0
    aspect_ratio = max(box_w, box_h) / min(box_w, box_h)
    
    if 1.0 <= aspect_ratio <= 1.15: 
        return "Square"
    else:
        return "Rectangle"

def main():
    # Remove cv2.CAP_DSHOW when moving to the Raspberry Pi
    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    # Lowered to 30 FPS to prevent CPU overload while still providing smooth tracking
    cap.set(cv2.CAP_PROP_FPS, 30) 

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    TRIGGER_Y = 360 
    TOLERANCE_Y = 15 
    
    # Memory array to prevent "Machine Gun" duplicate triggers
    recently_triggered = []

    # Morphological kernel for mask cleanup (removes jagged noise before contour detection)
    morph_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))

    print("Industrial Vision System Initialized. Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break
            
        current_time = time.time()

        # Clean up old plates from memory (older than 2 seconds)
        recently_triggered = [pt for pt in recently_triggered if current_time - pt[1] < 2.0]

        if len(frame.shape) == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame

        blurred = cv2.GaussianBlur(gray, (7, 7), 0)
        
        # Adaptive Thresholding (Handles ambient light changes)
        _, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        
        # --- Morphological Cleanup (removes jagged noise on contour edges) ---
        # Opening: strips small protrusions/speckles sticking OUT of the shape
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, morph_kernel)
        # Closing: fills small notches/holes cut INTO the shape
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, morph_kernel)
        
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw Trigger Line
        cv2.line(frame, (0, TRIGGER_Y), (1280, TRIGGER_Y), (0, 255, 255), 2)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            
            # Noise filter: Ignores dust and small debris
            if area > 15000: 
                shape_name = classify_shape_robust(cnt)
                
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                else:
                    continue 
                
                # Convert to physical coordinates for on-screen display
                cX_mm, cY_mm = pixels_to_mm(cX, cY)
                
                cv2.drawContours(frame, [cnt], 0, (0, 255, 0), 2)
                cv2.drawMarker(frame, (cX, cY), (255, 0, 0), cv2.MARKER_CROSS, 20, 2)
                
                # If plate is inside the trigger zone
                if (TRIGGER_Y - TOLERANCE_Y) <= cY <= (TRIGGER_Y + TOLERANCE_Y):
                    
                    # Check if we already triggered a plate in this X-lane recently
                    already_triggered = False
                    for pt in recently_triggered:
                        if abs(cX - pt[0]) < 50:
                            already_triggered = True
                            break
                    
                    if not already_triggered:
                        # 1. Calculate physical dimensions for sanity check
                        rect = cv2.minAreaRect(cnt)
                        (_, _), (box_w_px, box_h_px), _ = rect
                        dim1_mm = round(box_w_px * SCALE_FACTOR, 1)
                        dim2_mm = round(box_h_px * SCALE_FACTOR, 1)
                        long_side = max(dim1_mm, dim2_mm)
                        short_side = min(dim1_mm, dim2_mm)

                        # 2. Construct and print payload
                        robot_payload = f"{current_time:.3f},{shape_name},X:{cX_mm},Y:{cY_mm},Dims:{long_side}x{short_side}mm"
                        print(f"TRANSMIT TO ROBOT -> {robot_payload}")
                        
                        # Add to memory so it doesn't trigger again
                        recently_triggered.append((cX, current_time))
                        
                        # Flash a red circle to confirm visual trigger
                        cv2.circle(frame, (cX, cY), 25, (0, 0, 255), -1)

                cv2.putText(frame, f"{shape_name}", (cX + 15, cY - 15), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
                
                cv2.putText(frame, f"X:{cX_mm} Y:{cY_mm}mm", (cX + 15, cY + 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        cv2.imshow("Production Line Feed", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()