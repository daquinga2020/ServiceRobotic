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

count_align = 0
count_align_left = 0

count_front_free = 0
count_back_free = 0
count_free_hollow = 0
count_near_front_car = 0
count_near_back_car = 0
count_backward_bcklsr = 0
count_backward_frntlsr = 0

# count_lateral_parking = 0
same_distance = 0

start_angle = 0

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
    V = 0.6
    W = 0.0
    
    if CURRENT_STATE == ALIGNMENT:
      
      # Comparar valores del laser frontal y trasero:
      # Deben tener más o menos los mismos valores
      # Frontal más pequeña que trasera, coche inclinado a la izquierda
      # Trasero más pequeño que frontal, coche inclinado a la derecha
      
      # Frontal - 30º 5.2 min 45º 6.6 max
      # Back - 140º 5.5 max 155º 5.0
      print("ALIGNMENT")
      V = 0.2
      
      for i in range(65, 95):
        val_R = round(right_laser[i][0], 4)
        val_L = round(right_laser[i+20][0], 4)
        
        if val_L > val_R:
          count_align += 1
        else:
          count_align -= 1
    
      if count_align > 0:
        W = -0.15
      elif count_align < 0:
        W = 0.15
      
      if abs(count_align) <= 2:
        V = 0
        W = 0
        start_angle = HAL.getPose3d().yaw
        CURRENT_STATE = SEARCHING_HOLLOW
        
      count_align = 0
      
    elif CURRENT_STATE == SEARCHING_HOLLOW:
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
          CURRENT_STATE = SEARCHING_REFERENCE
          HAL.setV(0)
          time.sleep(10)
        
        count_free_hollow = 0
        
      count_front_free = 0
      count_back_free = 0
    elif CURRENT_STATE == SEARCHING_REFERENCE:
      print("SEARCHING REFERENCE")
      
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
      
      # Looking for front car
      if (not ref_car_front and ref_car_back) or not any_ref:
        for i in range(30, 61):
          if data_BackLaser.maxRange > back_laser[-i][0] > data_BackLaser.maxRange-2:
            count_near_back_car += 1
      
      # Looking for back car
      if (not ref_car_back and ref_car_front) or not any_ref:
        for i in range(2):
          if back_laser[-i-1][0] <= 5.0: # 5.0
            count_near_front_car += 1
            
        for i in range(75, 106):
          if right_laser[i][0] <= 5.0:
            count_near_front_car += 1
          
      if count_near_front_car >= 32 or count_near_back_car >= 5:
        print("NEAR CAR FOUND")
        CURRENT_STATE = PARKING
        V = 0.0
        # Quitar abajo
        HAL.setV(0)
        time.sleep(2)
        
      count_near_front_car = 0
      count_near_back_car = 0
    elif CURRENT_STATE == PARKING:
      print("PARKING")
      
      # TURN 45º
      if PARKING_STATE == TURN:
        print("YAW:", yaw_car, "DESIRE:", math.pi/4)
        
        # Primera Opcion: Utilizando la orientacion del coche
        angle_desire = math.pi/4 + start_angle
        if not check_car_orientation(yaw_car, angle_desire, 0.01):
          W = (angle_desire - yaw_car)*0.3
          V = -0.3
        else:
          PARKING_STATE = BACKING_UP
          W = 0.0
          V = 0.0
        '''  
        # Segunda opcion: Utilizando las medidas de los laseres.
        # Girar hasta que en el laser derecho se hallen medidas muy bajas
        V = -0.45 # -0.4
        W = 0.1
        
        for i in range(30, 70):
          if right_laser[i][0] < 2.5:
            count_lateral_parking += 1
          
        if count_lateral_parking >= 30:
          W = 0.0
          V = 0.0
          PARKING_STATE = BACKWARD
          
        count_lateral_parking = 0'''
        
      elif PARKING_STATE == BACKING_UP:
        print("BACKING UP")
        V = -0.2
        
        # Referencia en coche trasero, hasta que el principio del laser Trasero
        # detecte algo
        for i in range(12):
          if data_BackLaser.maxRange == back_laser[i][0]:
            count_backward_bcklsr += 1
          
          # Referencia en coche delantero
          print(front_laser[i][0])
          if 9.0 < front_laser[i][0] and front_laser[i][0] != data_FrontLaser.maxRange:
            count_backward_frntlsr += 1

        if count_backward_frntlsr > 8 or count_backward_bcklsr < 12:
          V = 0.0
          PARKING_STATE = MOVE_INTO_HOLLOW
          # Quitar abajo
          HAL.setV(0)
          print(count_backward_frntlsr, count_backward_bcklsr)
          time.sleep(5)
        
        count_backward_bcklsr = 0
        count_backward_frntlsr = 0

      elif PARKING_STATE == MOVE_INTO_HOLLOW:
        print("MOVE INTO HOLLOW")
        W = -0.08
        V = -0.3
        
        for i in range(80, 101):
          if back_laser[i][0] < 1.5:
            count_backward_bcklsr += 1
        
        angle_desire = 0.0 + start_angle
        print("YAW:", yaw_car, "DESIRE:", angle_desire)
        if not check_car_orientation(yaw_car, 0.0, 0.05):
          W = (angle_desire - yaw_car)*0.3
        else:
          W = 0.0
          PARKING_STATE = ADJUSTING
        
        if count_backward_bcklsr != 0:
          V = 0.3
        
        # count_backward_bcklsr = 0
          
      elif PARKING_STATE == ADJUSTING:
        V = 0.0
        sum_fr = 0
        sum_bck = 0
        for i in range(85, 96):
          sum_bck = back_laser[i][0] + sum_bck
          sum_fr = front_laser[i][0] + sum_fr
          
          if sum_fr == 11.0:
            sum_fr = 1111111111
        
        if sum_fr > 12:
          PARKED = 9
        
        diff_frnt2bck = sum_fr/10 - sum_bck/10
        sense = (diff_frnt2bck)/abs(diff_frnt2bck)
        
        V = 0.2*sense
        print(diff_frnt2bck)
        
        if abs(diff_frnt2bck) < 0.5:
          PARKED = 9
      
      elif PARKING_STATE == PARKED:
        print("PARKED")
        V = 0
    
    
    
    
    
    HAL.setV(V)
    HAL.setW(W)
    
    
    
    
    
    
    
    
    
    
    
    
    print("")
    
    
    