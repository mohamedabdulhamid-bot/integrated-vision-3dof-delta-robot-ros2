import math

# Robot Physical Parameters (in Meters)
sb = 0.60286
sp = 0.10392  
L = 0.32  
l = 0.9  

# Geometric Constants
wb = sb * math.sqrt(3) / 6.0
up = sp * math.sqrt(3) / 3.0
wp = sp * math.sqrt(3) / 6.0
a = wb - up
b = (sp / 2.0) - (math.sqrt(3) / 2.0) * wb
c = wp - 0.5 * wb
#-------------------------------------------------
#-----------------------------------------------
def calculate_ik( x, y, z):
        # Inverse Kinematics
        theta = [0.0, 0.0, 0.0]
        
        E1 = 2 * L * (y + a)
        F1 = 2 * z * L
        G1 = x**2 + y**2 + z**2 + a**2 + L**2 + 2*y*a - l**2
        
        E2 = -L * (math.sqrt(3) * (x + b) + y + c)
        F2 = 2 * z * L
        G2 = x**2 + y**2 + z**2 + b**2 + c**2 + L**2 + 2*(x*b + y*c) - l**2
        
        E3 = L * (math.sqrt(3) * (x - b) - y - c)
        F3 = 2 * z * L
        G3 = x**2 + y**2 + z**2 + b**2 + c**2 + L**2 + 2*(-x*b + y*c) - l**2

        try:
            # Minus root ensures standard Elbow-Out configuration for downward Z
            theta[0] = 2 * math.atan((-F1 - math.sqrt(E1**2 + F1**2 - G1**2)) / (G1 - E1))
            theta[1] = 2 * math.atan((-F2 - math.sqrt(E2**2 + F2**2 - G2**2)) / (G2 - E2))
            theta[2] = 2 * math.atan((-F3 - math.sqrt(E3**2 + F3**2 - G3**2)) / (G3 - E3))
            return theta
        except ValueError:
            return None
        
        
while True:
            try:
                user_input = input("Enter Target x y z like 0 0 -0.8 \n ")
                
                parts = user_input.strip().split()
                if len(parts) == 3:
                    X, Y, Z = map(float, parts)
                    theta = calculate_ik(X,Y,Z)
                    
                    # Added a check to prevent crashing if the position is unreachable (returns None)
                    if theta is not None:
                        theta[0] = math.degrees(theta[0])
                        theta[1] = math.degrees(theta[1])
                        theta[2] = math.degrees(theta[2])
                        print(theta)
                    else:
                        print("[ERROR] Unreachable coordinate.")
                    
                else:
                    print("[ERROR] Invalid format. Please enter exactly 3 numbers separated by spaces.")
            except ValueError:
                print("[ERROR] Please enter valid numbers.")
            except Exception:
                pass