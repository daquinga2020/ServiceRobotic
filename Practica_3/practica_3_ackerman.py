from GUI import GUI
from HAL import HAL
import time
import numpy as np
import math

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
FORWARD = 8
BACKWARD = 9
PARKED = 10
PARKING_STATE = TURN

# Car Measures
W_car = 2.6
H_car = 3.3

# Size of Hollow
dist_car2car = 3.8
hollow_h = W_car + dist_car2car
hollow_w = H_car
gama = math.atan(hollow_h/(hollow_w/2))
gama = round(math.degrees(gama))

# References
ref_car_front = False
ref_car_back = False
any_ref = False

posible_no_car = 0
no_sidecar = 0
safe_values_align = []
safe_degrees_align = []

# Counts for hollow state
count_front_free = 0
count_back_free = 0
count_free_hollow = 0

# Counts for Searching_Reference state
count_near_front_car = 0
count_near_back_car = 0

# Counts for Parking state
count_backward_bcklsr = 0
count_backward_frntlsr = 0
count_forward_frntlsr = 0

# Street Direction
start_angle = 0.0
street_direction = []

while True:
    # Enter iterative code!
    yaw_car = HAL.getPose3d().yaw
    data_BackLaser = HAL.getBackLaserData()
    data_FrontLaser = HAL.getFrontLaserData()
    data_RightLaser = HAL.getRightLaserData()
    
    
    front_laser = parse_data_laser(data_FrontLaser)
    right_laser = parse_data_laser(data_RightLaser)
    back_laser = parse_data_laser(data_BackLaser)
    
    V = 0.65
    W = 0.0
    
    if CURRENT_STATE != PARKING:
      if len(street_direction) == 0:
        values_align = []
        degrees_align = []

        for i in range(45, 135):
          if right_laser[i][0] != data_BackLaser.maxRange:
            values_align.append(right_laser[i][0])
            degrees_align.append(math.degrees(right_laser[i][1]))
          else:
            posible_no_car += 1
            if 120 > i >= 100:
              no_sidecar += 1
          
        if no_sidecar >= 18:
          values_align = safe_values_align
          degrees_align = safe_degrees_align
          print("NO LATERAL CAR, USING LAST LIST", no_sidecar)

        no_sidecar = 0
        
        # Si funciona igual comentando las listas safe, borrar
        if posible_no_car >= 75: # 89
          print("SAVING LAST LIST")
          posible_no_car = 0
          safe_values_align = values_align
          safe_degrees_align = degrees_align


        posible_no_car = 0
      else:
        print("USING LAST LIST WITH STREET DIRECTION")
        values_align = street_direction[0]
        degrees_align = street_direction[1]
      
      
      A = np.vstack([degrees_align, np.ones(len(degrees_align))]).T
      m, c = np.linalg.lstsq(A, values_align, rcond=None)[0]
      
      angle_radians = np.arctan(m)
      angle_degrees = np.degrees(angle_radians)
      
      if 0.001 > angle_degrees > -0.001:
        W = 0.0
      else:
        W = (0.0 - angle_radians)*10.5
    
    
    if CURRENT_STATE == SEARCHING_HOLLOW:
      print("SEARCHING HOLLOW")
      
      for i in range(3): # Cambiando el parametro de range aumenta/disminuye el espacio
        if front_laser[i][0] >= 7.0:
          count_front_free += 1
        
        if back_laser[-i-1][0] >= 7.0:
          count_back_free += 1

      if count_front_free - count_back_free == 0:
        print("SEARCHING POSIBLE HOLLOW")
        V = 0.45
        
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
        
        if count_free_hollow >= 175:
          print("HOLLOW FOUNDED")
          street_direction = [values_align, degrees_align]
          start_angle = HAL.getPose3d().yaw
          CURRENT_STATE = SEARCHING_REFERENCE
        
        count_free_hollow = 0
        
      count_front_free = 0
      count_back_free = 0
    elif CURRENT_STATE == SEARCHING_REFERENCE:
      print("SEARCHING REFERENCE")
      
      # First detect whether there is a rear or front car to know which reference to take
      if not ref_car_front and not ref_car_back and not any_ref: 
        count_ref_car_frnt = 0
        count_ref_car_bck = 0
        for i in range(15, 66):
          if front_laser[i][0] < 6.0:
            count_ref_car_frnt += 1
          if back_laser[-i][0] < 6.0:
            count_ref_car_bck += 1
        
        ref_car_front = count_ref_car_frnt > 10
        ref_car_back = count_ref_car_bck > 10
        any_ref = not ref_car_front and not ref_car_back
        
      print(ref_car_front, ref_car_back, any_ref)
      
      # Looking for front car
      if ref_car_front:
        for i in range(23):
          if back_laser[-i-1][0] <= 5.0:
            count_near_front_car += 1
            
        for i in range(75, 106):
          if right_laser[i][0] <= 5.0:
            count_near_front_car += 1
    
      # Looking for back car // Falta revisar si coge bien la referencia trasera
      if not ref_car_front and ref_car_back:
        for i in range(30, 61):
          if data_BackLaser.maxRange > back_laser[-i][0] > data_BackLaser.maxRange-2:
            count_near_back_car += 1    
      
      if count_near_front_car >= 53 or count_near_back_car >= 5:
        print("NEAR CAR FOUND")
        CURRENT_STATE = PARKING
        
      count_near_front_car = 0
      count_near_back_car = 0
      
    elif CURRENT_STATE == PARKING:
      print("PARKING")
      V = 0.0
      W = 0.0
      
      # TURN 45º
      if PARKING_STATE == TURN:
        print("PARKING_STATE: TURN")
        
        angle_desire = math.pi/4 + start_angle
        print("YAW CAR:", yaw_car, "DESIRE:", angle_desire)
        
        if not check_car_orientation(yaw_car, angle_desire, 0.01):
          W = (angle_desire - yaw_car)*25.5
          V = -0.35
        else:
          PARKING_STATE = BACKING_UP
      
      elif PARKING_STATE == BACKING_UP:
        print("PARKING_STATE: BACKING_UP")
        V = -0.35
        
        # Reference in rear car, until the rear laser principle detects something.
        for i in range(40): # 12 # 11
          if ref_car_back:
            if data_BackLaser.maxRange > back_laser[i][0]:
              count_backward_bcklsr += 1
          
          # Reference in front car//COMPROBAR
          if not ref_car_back and ref_car_front:
            if 9.0 < front_laser[i][0] and front_laser[i][0] != data_FrontLaser.maxRange:
              count_backward_frntlsr += 1

        if count_backward_frntlsr > 8 or count_backward_bcklsr > 0: # 12
          V = 0.0
          PARKING_STATE = MOVE_INTO_HOLLOW
          
      elif PARKING_STATE == MOVE_INTO_HOLLOW:
        print("PARKING_STATE: MOVE_INTO_HOLLOW")
        V = -0.23
        
        count_backward_bcklsr = 0
        
        # Searching the back laser
        for i in range(20): # 12
          if back_laser[i][0] < 1.0: # 0.8
            count_backward_bcklsr += 1
        
        # Si en el principio del laser traser no se detecta nada,
        # retroceder hasta que el centro del laser trasero detecte
        # un coche lo suficientemente cerca.
        if count_backward_bcklsr == 0:
          for i in range(87, 93): # 6
            if back_laser[i][0] < 0.8:
              count_backward_bcklsr += 1
        
        angle_desire = start_angle
        print("\tYAW:", yaw_car, "DESIRE:", angle_desire)
        
        # Si se alcanza la orientacion incial sin que se haya detectado ningun coche trasero
        # cercano, el coche esta aparcado
        if not check_car_orientation(yaw_car, start_angle, 0.03):
          W = -100
        else:
          V = 0.0
          W = 0.0
          PARKING_STATE = PARKED
        
        # Si se detecta un coche trasero cercano, cambio de estado a maniobrar
        if count_backward_bcklsr >= 2:
          PARKING_STATE = FORWARD
          V = 0.23
        
      elif PARKING_STATE == FORWARD:
        print("PARKING_STATE: FORWARD")
        V = 0.35
        W = -100
        
        # Searching the front laser // Revisar si quitar alguna condicion
        for i in range(88, 92): # 4
          if front_laser[i][0] < 0.5: # 0.8
            count_forward_frntlsr += 1
          elif count_forward_frntlsr != 0 and front_laser[i][0] >= 0.4: # 0.8
            count_forward_frntlsr = -1
            break
            
        if count_forward_frntlsr != 0:
          V = -0.35
          W = 100
          if count_forward_frntlsr == -1:
            PARKING_STATE = BACKWARD
      
      elif PARKING_STATE == BACKWARD:
        print("PARKING_STATE: BACKWARD")
        V = -0.35
        W = -100
        
        count_backward_bcklsr = 0
        
        # Searching the back laser
        for i in range(88, 92): # 4
          if back_laser[i][0] < 0.75: # 0.8
            count_backward_bcklsr += 1
            break
        
        if count_backward_bcklsr != 0:
          PARKING_STATE = PARKED
      elif PARKING_STATE == PARKED:
        print("PARKING_STATE: PARKED")
        V = 0.0
        W = 0.0
        
    
    
    
    
    
    
    
    HAL.setV(V)
    HAL.setW(W)
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    print("")
    
    
    