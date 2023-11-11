from GUI import GUI
from HAL import HAL
import cv2
import time
import math

height = 2.5

def choose_takeoff_land(action):
  
  current_state = HAL.get_landed_state()
  global height
  
  print("ESTADO: ", current_state)
  if action == "TakeOff" and current_state == 1:
    print("DRONE EN TIERRA DESPEGA")
    HAL.takeoff(height)
  elif action == "Land" and current_state == 2:
    print("DRONE EN EL AIRE ATERRIZA")
    HAL.land()
    
def check_goal(pose_drone, goal, err):
  
  x_drone = pose_drone[0]
  y_drone = pose_drone[1]
  
  x_goal = goal[0]
  y_goal = goal[1]
  
  check_xgoal = x_drone >= x_goal-err and x_drone <= x_goal+err
  check_ygoal = y_drone >= y_goal-err and y_drone <= y_goal+err
  
  return check_xgoal and check_ygoal

def get_corners_rect(start, h, w):
  
  x, y = start
  up_left = start
  down_left = (x-h,y)
  up_right = (x,y-w)
  down_right = (x-h,y-w)
    
  return up_right, down_right, up_left, down_left

def path_sweep(start, rect, n):

  x0, y0 = start
  h, w = rect
  
  n_passes = int(w/n)
  down_right = (x0-h,y0-w)
  
  down = []
  up = []
  path = []
  for i in range(y0, down_right[1]-n_passes, -n_passes):
    # Up -> x = x0
    point_up = (x0, i)
    # Down -> x = down_right[0]
    point_down = (down_right[0], i)

    up.append(point_up)
    down.append(point_down)
  
  state_up = 0
  path.append(up.pop(0))
  for i in range(len(down)-1):
    if not state_up:
      path.append(down[i])
      path.append(down[i+1])
      state_up = 1
    else:
      path.append(up[i-1])
      path.append(up[i])
      state_up = 0

  path.append(up.pop(-1))
  
  return path

'''def get_linvel(distance):
    min_distance = 0.5
    max_distance = 2
    min_vel = 0.4
    max_vel = 0.85

    # if distance < min_distance:
    #     return min_vel
    # elif distance >= min_distance and distance <= max_distance:
    slope = (max_vel - min_vel) / (max_distance - min_distance)
    vel = slope * (distance - min_distance) + min_vel
    return vel
    # else:
    #     return max_vel'''
def get_linvel(distance):
    min_distance = 0.1
    max_distance = 5.0
    min_vel = 0.15
    max_vel = 1.25
    
    slope = (max_vel - min_vel) / (max_distance - min_distance)
    vel = slope * (distance - min_distance) + min_vel
    
    return round(vel, 5)
      
def absolute2relative (x_abs, y_abs, robotx, roboty, robott):
    # robotx, roboty are the absolute coordinates of the robot
    # robott is its absolute orientation
    # Convert to relatives
    dx = x_abs - robotx
    dy = y_abs - roboty

    # Rotate with current angle
    x_rel = dx * math.cos (-robott) - dy * math.sin (-robott)
    y_rel = dx * math.sin (-robott) + dy * math.cos (-robott)

    return x_rel, y_rel
    
start = (0, 0)
rect = (15, 10)
n = 5

path = path_sweep(start, rect, n)

time.sleep(7)
# face_detector=cv.CascadeClassifier('haarcascade_frontalface_default.xml')

'''x0_goal = 15
y0_goal = 0
path =  []
for i in range(3, 15, 5):
  path.append((i, 0))
path.append((x0_goal, y0_goal))'''

x0, y0, z0 = HAL.get_position()
yaw0 = HAL.get_yaw()
print("POSICION INICIAL: ", [x0, y0, z0, yaw0])

choose_takeoff_land("TakeOff")
takeoff_time = time.time()
print("YA DESPEGUE")

vx, vy = 0, 0
on_axis_x = True # x:True - y:False
Kp = 0.35
reached_initial_goal = False
orientated = False
ind = 1

while True:
  
    state_dron = HAL.get_landed_state()
    ventral_img = HAL.get_ventral_image()
    frontal_img = HAL.get_frontal_image()
      
    GUI.showImage(frontal_img)
    GUI.showLeftImage(ventral_img)
    
    x, y, z = HAL.get_position()
    yaw = HAL.get_yaw()
    print("POSICION: ", [round(x, 4), round(y, 4), round(z, 4)], "YAW:", round(yaw, 4))
    vxd, vyd, vzd = HAL.get_velocity()
    yaw_rate = HAL.get_yaw_rate()
    print("VELOCIDADES:", [round(vxd, 4), round(vyd, 4), round(vzd, 4)], "YAW RATE:", round(yaw_rate, 4))
    
    if state_dron == 2 and (check_goal((x, y), (0, 0), 0.1) or reached_initial_goal):
      if not reached_initial_goal:
        time.sleep(3)
      
      reached_initial_goal = True
      x_goal, y_goal = path[ind]
      
      if check_goal((x, y), (x_goal, y_goal), 0.25): # Posiblemente cambiar comprobacion de llegada a la meta
        time.sleep(1)
        HAL.set_cmd_mix(0, 0, height, 0)
        ind+=1
        on_axis_x = not on_axis_x
        orientated = False
        # Si se termina el recorrido, se vuelve a empezar
        if ind >= len(path):
          ind = 0
          on_axis_x = True
          orientated = False
          # Comandar la posicion de inicio con set_cmd_pose
      else:
        # HAL.set_cmd_pos(x_goal, y_goal, height, 0)
      
        # Primero orientar hacia el objetivo, posteriormente, avanzar
        goal_rel_x, goal_rel_y = absolute2relative(x_goal, y_goal, x, y, yaw)
        dist_x = math.sqrt((goal_rel_x**2))
        dist_y = math.sqrt((goal_rel_y**2))
        w = math.atan2(goal_rel_y, goal_rel_x)*Kp
        
        sense_x = (goal_rel_x)/abs(goal_rel_x)
        sense_y = (goal_rel_y)/abs(goal_rel_y)
        
        vx = get_linvel(dist_x)
        vy = get_linvel(dist_y)
        
        print("Distance:", (round(dist_x, 4), round(dist_y, 4)))
        print("VX:", vx, "VY:", vy, "W:", w)
        
        # Saber hacia que eje me muevo, si en X o en Y
        # Asignar velocidad minima al eje en el que no se debe mover para evitar desviarse de la trayectoria
        
        if math.fabs(w) <= 0.01 and not orientated:
          print("ORIENTADO")
          orientated = True
          w = 0
        elif orientated:
          w = 0
          if on_axis_x:
            vx = vx * sense_x
            vy = 0.075 * sense_y
          else:
            vx = 0.075 * sense_x
            vy = vy * sense_y
        else:
          vx = 0
          vy = 0
          
        '''if x >= x_goal-0.1 and x <= x_goal+0.1:
          vx = 0
          vy = 0
          w = 0'''
        
        # print("VX:", vx, "VY:", vy, "W:", w)
        HAL.set_cmd_mix(vx, vy, height, w)
    else:
      HAL.set_cmd_pos(0, 0, height, 0)
    print("")
