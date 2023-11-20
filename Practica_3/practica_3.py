from GUI import GUI
from HAL import HAL
import time
import math
# Enter sequential code!

def parse_data_laser(data_laser):
    
  values = data_laser.values # tuple
  clean_data =  []
    
  for i in range(180):
    angle = math.radians(i)
    val = values[i]
        
    if math.isfinite(val):
      if val > data_laser.maxRange:
        val = data_laser.maxRange
    else:
      if math.isinf(val):
        val = data_laser.maxRange
      else:
        val = data_laser.minRange
    
    clean_data.append((val, angle))
    
  return clean_data # [(value, angle), ...]
    
def check_car_orientation(yaw_robot, yaw_desired, err):
  return yaw_robot >= yaw_desired-err and yaw_robot <= yaw_desired+err

'''def check_separation_car(laser_values):
  separation = False
  count = 0
  for measure in laser_values:
    dist = measure[0]
    degree = math.degrees(measure[1])
    if degree >= 65 and degree <= 100:
      if dist  >= 3.2 and dist <= 3.8:
        count += 1
  
  return count >= 30'''

time.sleep(2)

# Estados
ALIGNMENT = 1
SEARCHING_HOLLOW = 2
SEARCHING_REFERENCE = 3
PARKING = 4

CURRENT_STATE = ALIGNMENT

# PARKING STATES
TURN = 5
BACKWARD = 6
PARKING_STATE = TURN


# Car
W_car = 2.6
H_car = 3.3

# Hollow
dist_car2car = 3.8
hollow_h = W_car + dist_car2car
hollow_w = H_car
gama = math.atan(hollow_h/(hollow_w/2))
gama = round(math.degrees(gama))

count_align = 0
count_align_left = 0

count_front_free = 0
count_back_free = 0
count_free_hollow = 0
count_near_car = 0
count_backward = 0

V = 0

while True:
    # Enter iterative code!
    x_car, y_car, yaw_car = HAL.getPose3d().x, HAL.getPose3d().y, HAL.getPose3d().yaw
    
    data_FrontLaser = HAL.getFrontLaserData()
    data_RightLaser = HAL.getRightLaserData()
    data_BackLaser = HAL.getBackLaserData()
    
    
    front_laser = parse_data_laser(data_FrontLaser)
    right_laser = parse_data_laser(data_RightLaser)
    back_laser = parse_data_laser(data_BackLaser)
    
    
    # print("POSE:", (x_car, y_car, yaw_car))
    V = 0.5
    if CURRENT_STATE == ALIGNMENT:
      # Comparar valores del laser derecho:
      # Parte izq y der, deben tener más o menos los mismos valores.
      # Parte izq más pequeña que parte der, coche inclinado a la izquierda
      # Parte der más pequeña que parte izq, coche inclinado a la derecha
      
      # Comparar valores del laser frontal y trasero:
      # Deben tener más o menos los mismos valores
      # Frontal más pequeña que trasera, coche inclinado a la izquierda
      # Trasero más pequeño que frontal, coche inclinado a la derecha
      
      # Frontal - 30º 5.2 min 45º 6.6 max
      # Back - 140º 5.5 max 155º 5.0
      print("ALIGNMENT")

      for i in range(65, 95):
        val_R = round(right_laser[i][0], 4)
        val_L = round(right_laser[i+20][0], 4)
        
        # 10 False y 10 True
        # print("\t R:", val_R, "L", val_L, "BOOL:", val_L > val_R, "-", val_L < val_R)
        if val_L > val_R:
          count_align += 1
        else:
          count_align -= 1
        # time.sleep(0.5)
      
      # Inclinado Derecha, countL mayor que countR, giro izquierda, dejar margen de error 2
      # print("\tCount R:", count_align)
      # Si hay mas lecturas por el lado derecho, el coche esta mirando hacia la derecha
      if count_align > 0:
        W = -0.15 # 2.0
      elif count_align < 0:
        W = 0.15
      
      if abs(count_align) <= 2:
        V = 0
        W = 0
        CURRENT_STATE = SEARCHING_HOLLOW
        
      count_align = 0
      
      '''if not check_car_orientation(yaw_car, 0.0, 0.001):
        W = (0.0 - yaw_car)
      else:
        CURRENT_STATE = SEARCHING_HOLLOW
        W = 0
        V = 0.7'''
      
      V = 0.07
      # W = 0
    elif CURRENT_STATE == SEARCHING_HOLLOW:
      print("SEARCHING_HOLLOW")
      
      for i in range(15): # Cambiando el parametro de range aumenta/disminuye el espacio
        if front_laser[i][0] >= 7.0:
          count_front_free += 1
        
        if back_laser[-i-1][0] >= 7.0:
          count_back_free += 1

      if count_front_free - count_back_free == 0:
        print("SEARCHING POSIBLE HOLLOW")
        for m in right_laser:
          angle = math.degrees(m[1])
          if 180 - gama > angle > gama:
            aprox_measure = hollow_h/math.sin(math.radians(angle))
          else:
            aprox_measure = (hollow_w/2)/math.cos(math.radians(angle))
            if aprox_measure <= 0:  
              aprox_measure = hollow_h + aprox_measure
              
          if m[0] >= aprox_measure:
            count_free_hollow += 1
        
        if count_free_hollow == 180:
          CURRENT_STATE = SEARCHING_REFERENCE
        
        count_free_hollow = 0
        
      count_front_free = 0
      count_back_free = 0
    elif CURRENT_STATE == SEARCHING_REFERENCE:
      print("SEARCHING REFERENCE")
      
      for i in range(2):
        if back_laser[-i-1][0] <= 5.0: # 5.0
          count_near_car += 1
          
      for i in range(75, 106):
        if right_laser[i][0] <= 5.0:
          count_near_car += 1
          
      if count_near_car >= 32:
        print("NEAR FRONT CAR FOUND")
        CURRENT_STATE = PARKING
        HAL.setV(0)
        time.sleep(2)
        
      count_near_car = 0
    elif CURRENT_STATE == PARKING:
      print("PARKING")
      
      # TURN 45º
      if PARKING_STATE == TURN:
        print("YAW:", yaw_car, "DESIRE:", math.pi/4)
        if not check_car_orientation(yaw_car, math.pi/4, 0.01):
          W = (math.pi/4 - yaw_car)*0.3
          V = -0.3
        else:
          PARKING_STATE = BACKWARD
          W = 0.0
          V = 0.0
      elif PARKING_STATE == BACKWARD:
        W = 0.0
        V = -0.2
        for i in range(15):
          if data_BackLaser.maxRange == back_laser[i][0]:
            count_backward += 1
          print((back_laser[i][0], math.degrees(back_laser[i][1])))
        
        if count_backward < 15:
          V = 0.0
          W = 0.0
          PARKING_STATE = 0
        
        count_backward = 0
        '''if not check_car_orientation(yaw_car, 0.0, 0.01):
          W = (0.0 - yaw_car)*0.2
          V = -0.3
        else:
          PARKING_STATE = 7
          W = 0.0
          V = 0.0'''
      
      
    
    
    
    
    
    HAL.setV(V)
    HAL.setW(W)
    
    
    
    
    
    
    
    
    
    
    
    
    print("")
    
    
    