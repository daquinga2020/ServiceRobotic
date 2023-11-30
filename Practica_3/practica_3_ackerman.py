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
ADJUSTING = 8
PARKED = 9
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
    
    # Fase de pruebas
    if CURRENT_STATE != PARKING:
      if len(street_direction) == 0:
        values_align = []
        degrees_align = []

        for i in range(45, 135):
          if right_laser[i][0] != data_BackLaser.maxRange:
            values_align.append(right_laser[i][0])
            degrees_align.append(math.degrees(right_laser[i][1]))
          else:
            # Si no detecto nada en el centro del lateral, guardar los arrays y usarlos cuando no se detecte nada
            posible_no_car += 1
            if 120 > i >= 100:
              no_sidecar += 1
        
        safe_values_align = []
        safe_degrees_align = []
          
        if no_sidecar >= 18:# 28:
          no_sidecar = 0 # MOD
          values_align = safe_values_align
          degrees_align = safe_degrees_align
          print("SIN COCHE LATERAL", no_sidecar)
        else:
          no_sidecar = 0
        
        if posible_no_car >= 75: # 89
          print("GUARDO LISTAS PASADAS")
          posible_no_car = 0
          safe_values_align = values_align
          safe_degrees_align = degrees_align


        posible_no_car = 0
      else:
        print("DIRECCION DEL COCHE DEFINIDA")
        values_align = street_direction[0]
        degrees_align = street_direction[1]
      
      # Encuentra la recta que mejor se ajusta utilizando el método de mínimos cuadrados
      A = np.vstack([degrees_align, np.ones(len(degrees_align))]).T
      m, c = np.linalg.lstsq(A, values_align, rcond=None)[0]
      
      # Calcula el ángulo que forma la recta con el eje Y
      angle_radians = np.arctan(m)
      angle_degrees = np.degrees(angle_radians)
      # print("\tANGLE:", angle_degrees, "YAW:", yaw_car)
      # time.sleep(5)
      
      if 0.001 > angle_degrees > -0.001:
        W = 0.0
      else:
        W = (0.0 - angle_radians)*10.5  
    # Fin de fase de pruebas
    ########################
    
    
    if CURRENT_STATE == SEARCHING_HOLLOW:
      print("SEARCHING HOLLOW")
      for i in range(3): # Cambiando el parametro de range aumenta/disminuye el espacio
        # print("\tFR:", front_laser[i][0], "BC:", back_laser[-i-1][0])
        if front_laser[i][0] >= 7.0:
          count_front_free += 1
        
        if back_laser[-i-1][0] >= 7.0:
          count_back_free += 1

      if count_front_free - count_back_free == 0:
        V = 0.45
        
        print("SEARCHING POSIBLE HOLLOW")
        for m in right_laser:
          angle = math.degrees(m[1])
          
          if 180 - gama > angle > gama:
            aprox_measure = hollow_h/math.sin(m[1])
          else:
            aprox_measure = (hollow_w/2)/math.cos(m[1])
            if aprox_measure <= 0:  
              aprox_measure = hollow_h + aprox_measure
          # print("\tMSR:", aprox_measure, "REAL:", m[0])
          if m[0] >= aprox_measure:
            count_free_hollow += 1
            
        print("HOLLOW:", count_free_hollow)
        if count_free_hollow >= 175:
          HAL.setV(0)
          HAL.setW(0)
          print("HOLLOW FOUNDED")
          # Guardar el angulo fijo del coche y la lista de valores
          street_direction = [values_align, degrees_align]
          start_angle = HAL.getPose3d().yaw
          # time.sleep(5)
          CURRENT_STATE = SEARCHING_REFERENCE
        
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
          # print("FR:", front_laser[i][0], "BCK:", back_laser[-i][0])
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
          # print("BCK:", back_laser[-i-1][0])
          if back_laser[-i-1][0] <= 5.0: # 5.0
            count_near_front_car += 1
            
        for i in range(75, 106):
          # print("RGHT:", right_laser[i][0])
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
        # Quitar abajo
        HAL.setV(0)
        HAL.setW(0)
        time.sleep(1)
        
      count_near_front_car = 0
      count_near_back_car = 0
      
    elif CURRENT_STATE == PARKING:
      print("PARKING")
      V = 0.0
      W = 0.0
      
      # TURN 45º
      if PARKING_STATE == TURN:
        print("PARKING_STATE: TURN")
        # Primera Opcion: Utilizando la orientacion del coche
        angle_desire = math.pi/4 + start_angle
        print("YAW CAR:", yaw_car, "DESIRE:", angle_desire)
        
        if not check_car_orientation(yaw_car, angle_desire, 0.01):
          W = (angle_desire - yaw_car)*25.5 #16.5
          V = -0.35
        else:
          PARKING_STATE = BACKING_UP
          # W = 0.0
          # V = 0.0
      elif PARKING_STATE == BACKING_UP:
        print("PARKING_STATE: BACKING_UP")
        V = -0.35
        
        # Referencia en coche trasero, hasta que el principio del laser Trasero
        # detecte algo
        for i in range(40): # 12 # 11
          if ref_car_back:
            if data_BackLaser.maxRange > back_laser[i][0]:
              count_backward_bcklsr += 1
          
          # Referencia en coche delantero
          if not ref_car_back and ref_car_front:
            if 9.0 < front_laser[i][0] and front_laser[i][0] != data_FrontLaser.maxRange:
              count_backward_frntlsr += 1

        if count_backward_frntlsr > 8 or count_backward_bcklsr > 0: # 12
          V = 0.0
          PARKING_STATE = MOVE_INTO_HOLLOW
          # Quitar abajo
          HAL.setV(0)
          print(count_backward_frntlsr, count_backward_bcklsr)
          # time.sleep(5)
          # time.sleep(340)
          forward = False
      elif PARKING_STATE == MOVE_INTO_HOLLOW:
        print("PARKING_STATE: MOVE_INTO_HOLLOW")
        V = -0.23
        
        count_backward_bcklsr = 0
        
        for i in range(20): # 12
          if back_laser[i][0] < 1.0: # 0.8
            count_backward_bcklsr += 1
            print("\tBCK:", back_laser[i][0], i)
        
        if count_backward_bcklsr == 0:
          for i in range(87, 93): # 6
            if back_laser[i][0] < 0.8: # 0.8
              count_backward_bcklsr += 1
              print("\tBCK:", back_laser[i][0], i)
        
        angle_desire = start_angle
        print("YAW:", yaw_car, "DESIRE:", angle_desire)
        if not check_car_orientation(yaw_car, start_angle, 0.05):
          # W = (angle_desire - yaw_car)*50.5
          W = -80.5
        else:
          V = 0.0
          W = 0.0
          PARKING_STATE = ADJUSTING
          HAL.setV(0)
          HAL.setW(0)
          print("DENTRO")
          time.sleep(1056)
        
        if count_backward_bcklsr >= 2:
          PARKING_STATE = ALIGNMENT
          V = 0.23
          
        print("\tV:",V,"W:", W)
      elif PARKING_STATE == ALIGNMENT:
        V = 0.35
        W = -80
        
        for i in range(88, 92): # 4
          if front_laser[i][0] < 0.5: # 0.8
            count_forward_frntlsr += 1
            print("\tBCK:", front_laser[i][0], i)
          elif count_forward_frntlsr != 0 and front_laser[i][0] >= 0.8:
            count_forward_frntlsr = -1
            break
            
        if count_forward_frntlsr != 0:
          V = -0.35
          W = 80
          if count_forward_frntlsr == -1:
            PARKING_STATE = PARKED
        
        print("\tV:",V,"W:", W)
      elif PARKING_STATE == PARKED:
        print("PARKED")
        V = 0
        W = 0
        
    
    
    
    
    
    
    
    HAL.setV(V)
    HAL.setW(W)
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    print("")
    
    
    