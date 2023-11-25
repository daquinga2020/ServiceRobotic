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

start_angle = 0

while True:
    # Enter iterative code!
    yaw_car = HAL.getPose3d().yaw
    data_BackLaser = HAL.getBackLaserData()
    data_FrontLaser = HAL.getFrontLaserData()
    data_RightLaser = HAL.getRightLaserData()
    
    
    front_laser = parse_data_laser(data_FrontLaser)
    right_laser = parse_data_laser(data_RightLaser)
    back_laser = parse_data_laser(data_BackLaser)
    
    V = 0.4
    W = 0.0
    
    values_align = []
    degrees_align = []

    for i in range(30, 120):
      if right_laser[i][0] != data_BackLaser.maxRange:
        values_align.append(right_laser[i][0])
        degrees_align.append(math.degrees(right_laser[i][0]))
    
    # Encuentra la recta que mejor se ajusta utilizando el método de mínimos cuadrados
    # A = np.vstack([values_align, np.ones(len(values_align))]).T
    A = np.vstack([values_align, np.ones(len(values_align))]).T
    m, c = np.linalg.lstsq(A, degrees_align, rcond=None)[0]
    
    # Calcula el ángulo que forma la recta con el eje Y
    angle_radians = np.arctan(m)
    angle_degrees = np.degrees(angle_radians)
    print("ANGLE:", angle_degrees)
    
    
    if CURRENT_STATE == ALIGNMENT:
      # Frontal - 30º 5.2 min 45º 6.6 max
      # Back - 140º 5.5 max 155º 5.0
      print("ALIGNMENT")
      V = 0.3
      
      val_R = 0
      val_L = 0
      
      values_align = []
      degrees_align = []
      
      for i in range(30, 120):
        if right_laser[i][0] != data_BackLaser.maxRange:
          values_align.append(right_laser[i][0])
          degrees_align.append(math.degrees(right_laser[i][0]))
        '''val_R = round(right_laser[i][0], 4) + val_R
        val_L = round(right_laser[i+15][0], 4) + val_L'''
        
        
        '''# Si se esta mas cerca de la izquierda, girar derecha +
        diff_lateral = val_L - val_R
        sense = 1
        
        if diff_lateral != 0:
          sense = diff_lateral/abs(diff_lateral)
        
        print("\tDIFF:", diff_lateral)
        
        if abs(diff_lateral) <= 0.1:
          count_align += 1'''
        
        # print("\tR:", (val_R,i), "L:", (val_L,i+15))
        # time.sleep(1)

      # np_val_align = np.array(values_align)
      # y = [0] * len(values_align)
      # x - grado / y - medida
      
      # Encuentra la recta que mejor se ajusta utilizando el método de mínimos cuadrados
      # A = np.vstack([values_align, np.ones(len(values_align))]).T
      A = np.vstack([values_align, np.ones(len(values_align))]).T
      m, c = np.linalg.lstsq(A, degrees_align, rcond=None)[0]

      # Calcula el ángulo que forma la recta con el eje Y
      angle_radians = np.arctan(m)
      angle_degrees = np.degrees(angle_radians)
      print("ANGLE:", angle_degrees)
      time.sleep(5)
      diff_lateral = val_L/15 - val_R/15
      
      if diff_lateral != 0:
          sense = diff_lateral/abs(diff_lateral)
      
      print("\tDIFF:", diff_lateral)
      # time.sleep(10)
      
      # W = -0.30 * sense
      
      if abs(diff_lateral) <= 0.02:
        start_angle = HAL.getPose3d().yaw
        W = 0.0
        count_align += 1
        if count_align > 4:
          # CURRENT_STATE = SEARCHING_HOLLOW
          count_align = 0
        # time.sleep(15)
      
      '''if count_align > 0:
        W = -0.15
      elif count_align < 0:
        W = 0.15'''

    if CURRENT_STATE != ALIGNMENT:
      print("YAW:", yaw_car, "DESIRE:", start_angle)
      if not check_car_orientation(yaw_car, start_angle, 0.05):
        W = (start_angle - yaw_car)*1 # 0.3
      else:
        W = 0.0

    print("W:", W)
    
    HAL.setV(V*0)
    HAL.setW(W*0)
    
    
    
    
    
    
    
    
    
    
    
    
    print("")
    
    
    