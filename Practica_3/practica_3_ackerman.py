from GUI import GUI
from HAL import HAL
import time
import numpy as np
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
      elif math.isnan(val):
        val = data_laser.minRange
    
    clean_data.append((val, angle))
    
  return clean_data # [(value, angle), ...]
    
def check_car_orientation(yaw_robot, yaw_desired, err):
  return yaw_robot >= yaw_desired-err and yaw_robot <= yaw_desired+err

time.sleep(4)

# Estados
ALIGNMENT = 1
SEARCHING_HOLLOW = 2
SEARCHING_REFERENCE = 3
PARKING = 4

CURRENT_STATE = SEARCHING_HOLLOW

# PARKING STATES
TURN = 5
BACKING_UP = 6
MOVE_INTO_HOLLOW = 7
ADJUSTING = 8
PARKED = 9
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

# References
ref_car_front = False
ref_car_back = False
any_ref = False

# count_align_left = 0

count_front_free = 0
count_back_free = 0
count_free_hollow = 0

count_near_front_car = 0
count_near_back_car = 0

count_backward_bcklsr = 0
count_backward_frntlsr = 0

# count_lateral_parking = 0
same_distance = 0

start_angle = 2.42 # offset +-0.3

while True:
    # Enter iterative code!
    yaw_car = HAL.getPose3d().yaw
    data_BackLaser = HAL.getBackLaserData()
    data_FrontLaser = HAL.getFrontLaserData()
    data_RightLaser = HAL.getRightLaserData()
    
    
    front_laser = parse_data_laser(data_FrontLaser)
    right_laser = parse_data_laser(data_RightLaser)
    back_laser = parse_data_laser(data_BackLaser)
    
    V = 0.55
    W = 0.0
    
    if CURRENT_STATE == SEARCHING_HOLLOW:
      print("SEARCHING HOLLOW")
      
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
            aprox_measure = hollow_h/math.sin(m[1])
          else:
            aprox_measure = (hollow_w/2)/math.cos(m[1])
            if aprox_measure <= 0:  
              aprox_measure = hollow_h + aprox_measure
              
          if m[0] >= aprox_measure:
            count_free_hollow += 1
        
        if count_free_hollow == 180:
          HAL.setV(0)
          print("HOLLOW FOUNDED")
          time.sleep(5)
          CURRENT_STATE = SEARCHING_REFERENCE
        
        count_free_hollow = 0
        
      count_front_free = 0
      count_back_free = 0
    elif CURRENT_STATE == SEARCHING_REFERENCE:
      print("SEARCHING REFERENCE")
      V = 0.4
      # Primero detectar si hay coche trasero o delantero para saber qué referencia tomar
      if not ref_car_front and not ref_car_back and not any_ref: 
        
        count_ref_car_frnt = 0
        count_ref_car_bck = 0
        for i in range(15, 66):
          print("FR:", front_laser[i][0], "BCK:", back_laser[-i][0])
          if front_laser[i][0] < 6.0:
            count_ref_car_frnt += 1
          if back_laser[-i][0] < 6.0:
            count_ref_car_bck += 1
        
        ref_car_front = count_ref_car_frnt > 10
        ref_car_back = count_ref_car_bck > 10
        any_ref = not ref_car_front and not ref_car_back
        
      print(ref_car_front, ref_car_back, any_ref)
      
      # Qué hacer cuando no hay ninguna referencia
      
      # Looking for front car
      if ref_car_front:
        for i in range(2):
          print("BCK:", back_laser[-i-1][0])
          if back_laser[-i-1][0] <= 5.0: # 5.0
            count_near_front_car += 1
            
        for i in range(75, 106):
          print("RGHT:", right_laser[i][0])
          if right_laser[i][0] <= 5.0:
            count_near_front_car += 1
    
      # Looking for back car // Falta revisar si coge bien la referencia trasera
      if not ref_car_front and ref_car_back:
        for i in range(30, 61):
          if data_BackLaser.maxRange > back_laser[-i][0] > data_BackLaser.maxRange-2:
            count_near_back_car += 1    
      
      if count_near_front_car >= 32 or count_near_back_car >= 5:
        print("NEAR CAR FOUND")
        CURRENT_STATE = PARKING
        # Quitar abajo
        HAL.setV(0)
        time.sleep(60)
        
      count_near_front_car = 0
      count_near_back_car = 0
      
    elif CURRENT_STATE == PARKING:
      print("PARKING")
      
      # Solucionar antes orientacion
      # TURN 45º
      if PARKING_STATE == TURN:
        # Primera Opcion: Utilizando la orientacion del coche
        angle_desire = math.pi/4
        print("YAW CAR:", yaw_car, "DESIRE:", angle_desire)
        
        if not check_car_orientation(yaw_car, angle_desire, 0.01):
          W = (angle_desire - yaw_car)*0.3
          V = -0.3
        else:
          PARKING_STATE = BACKING_UP
          W = 0.0
          V = 0.0
    
    
  
    
    
    
    
    
    
    
    
    HAL.setV(V)
    HAL.setW(W)
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    print("")
    
    
    