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
#-------------------------------------------------
def calculate_fk(theta1, theta2, theta3):
        #look in the the delta_fk.pdf to understand these equations
        t = wb - up
        
        y1 = -(t + L * math.cos(theta1))
        z1 = -L * math.sin(theta1)
        
        y2 = (t + L * math.cos(theta2)) * math.sin(math.radians(30))
        x2 = (t + L * math.cos(theta2)) * math.cos(math.radians(30))
        z2 = -L * math.sin(theta2)
        
        y3 = (t + L * math.cos(theta3)) * math.sin(math.radians(30))
        x3 = -(t + L * math.cos(theta3)) * math.cos(math.radians(30))
        z3 = -L * math.sin(theta3)
        
        #cramer
        #---->Δ=(A1​×B2​)−(A2​×B1​)
        #---->x=(C1​×B2​)−(C2​×B1​)​/Δ
        #---->y=(A1​×C2​)−(A2​×C1​)​/Δ
        #A1=x2     , B1=(y2-y1)    ,c1=[(w2​−w1​)/2​−(z2​−z1​)z]
        #A2=x3     , B2=(y3-y1)    ,c2=[(w3​−w1​)/2​−(z3​−z1​)z]
        
        
        delta = (y2 - y1) * x3 - (y3 - y1) * x2
        
        w1 = y1**2 + z1**2   #x1=0
        w2 = x2**2 + y2**2 + z2**2
        w3 = x3**2 + y3**2 + z3**2
        
        #a1 coffectient of z ,,,,b1-->constants   --->for x
        a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
        b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0
        #x=(a1​⋅z+b1​​)/delta
        
        a2 = -(z2 - z1) * x3 + (z3 - z1) * x2
        b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0
        #y=(a2​⋅z+b2​​)/delta
        
        #x2+(y−y1​)2+(z−z1​)2=l2--->  x,yهنعوض مكان 
        #a⋅z2+b⋅z+c=0---> zعشان تبقي كلها في 
        a = a1**2 + a2**2 + delta**2
        b = 2 * (a1 * b1 + a2 * (b2 - y1 * delta) - z1 * delta**2)
        c = (b2 - y1 * delta)**2 + b1**2 + delta**2 * (z1**2 - l**2)
        
        d = b**2 - 4.0 * a * c
        if d < 0:
            return None
            
        z0 = -0.5 * (b + math.sqrt(d)) / a
        x0 = (a1 * z0 + b1) / delta
        y0 = (a2 * z0 + b2) / delta
        
        return [x0, y0, z0]
#-------------------------------------------------
#-------------------------------------------------
while True:
            try:
                user_input = input("Enter Target theta1 theta2 theta3 like 30 30 30 \n ")
                
                parts = user_input.strip().split()
                if len(parts) == 3:
                    theta1, theta2, theta3 = map(float, parts)
                    # Fixed: Convert inputs from degrees to radians before calculations
                    print(calculate_fk(math.radians(theta1), math.radians(theta2), math.radians(theta3)))
                else:
                    print("[ERROR] Invalid format. Please enter exactly 3 numbers separated by spaces.")
            except ValueError:
                print("[ERROR] Please enter valid numbers.")
            except Exception:
                pass