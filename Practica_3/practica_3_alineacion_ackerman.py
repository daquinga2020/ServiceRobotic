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

start_angle = 2.42 # offset +-0.3
orientated = False

while True:
    # Enter iterative code!
    yaw_car = HAL.getPose3d().yaw
    data_BackLaser = HAL.getBackLaserData()
    data_FrontLaser = HAL.getFrontLaserData()
    data_RightLaser = HAL.getRightLaserData()
    
    
    front_laser = parse_data_laser(data_FrontLaser)
    right_laser = parse_data_laser(data_RightLaser)
    back_laser = parse_data_laser(data_BackLaser)
    
    V = 0.55 # 0.25
    W = 0.0
    
    
    
    values_align = []
    degrees_align = []

    for i in range(45, 135):
      if right_laser[i][0] != data_BackLaser.maxRange:
        # print("\t", right_laser[i][0], math.degrees(right_laser[i][1]))
        values_align.append(right_laser[i][0])
        degrees_align.append(math.degrees(right_laser[i][1]))
      else:
        # Si no detecto nada en el centro del lateral, guardar los arrays y usarlos cuando no se detecte nada
        count_align_left += 1
        if 120 > i >= 100:
          count_align += 1
    
    safe_values_align = []
    safe_degrees_align = []
      
    if count_align >= 18:# 28:
      count_align = 0 # MOD
      values_align = safe_values_align
      degrees_align = safe_degrees_align
      print("SIN COCHE LATERAL", count_align)
    else:
      count_align = 0
    
    if count_align_left >= 75: # 89
      print("GUARDO LISTAS PASADAS")
      count_align_left = 0
      safe_values_align = values_align
      safe_degrees_align = degrees_align
    
    
    count_align_left = 0
    
    # Encuentra la recta que mejor se ajusta utilizando el método de mínimos cuadrados
    A = np.vstack([degrees_align, np.ones(len(degrees_align))]).T
    m, c = np.linalg.lstsq(A, values_align, rcond=None)[0]
    
    # Calcula el ángulo que forma la recta con el eje Y
    angle_radians = np.arctan(m)
    angle_degrees = np.degrees(angle_radians)
    print("\tANGLE:", angle_degrees, "YAW:", yaw_car)
    # time.sleep(5)
    
    if 0.001 > angle_degrees > -0.001:
      W = 0.0
    else:
      W = (0.0 - angle_radians)*10.5
    
    
    
    print("W:", W)
    
    # W = 0
    # V = 0
    
    
    
    
    
    
    
    
    HAL.setV(V)
    HAL.setW(W)
    print("")
    
    
    