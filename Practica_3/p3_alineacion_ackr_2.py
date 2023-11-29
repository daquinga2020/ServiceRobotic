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


    if CURRENT_STATE == ALIGNMENT:

      for i in range(65, 95):
        val_R = round(right_laser[i][0], 4)
        val_L = round(right_laser[i+20][0], 4)

        if val_L > val_R:
          count_align += 1
        else:
          count_align -= 1

      if count_align > 0:
        W = -0.25
      elif count_align < 0:
        W = 0.25

      if abs(count_align) <= 2:
        W = 0.0
        start_angle = HAL.getPose3d().yaw
        CURRENT_STATE = SEARCHING_HOLLOW

      count_align = 0
    elif CURRENT_STATE == SEARCHING_HOLLOW:
      print("SEARCHING HOLLOW")
      time.sleep(60)























    HAL.setV(V*0)
    HAL.setW(W*0)
    print("")


