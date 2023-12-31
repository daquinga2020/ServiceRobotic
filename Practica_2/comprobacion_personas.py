from GUI import GUI
from HAL import HAL
import cv2
import time
import math

height = 1.75

def choose_takeoff_land(action):
  
  current_state = HAL.get_landed_state()
  global height
  
  if action == "TakeOff" and current_state == 1:
    print("DRONE EN TIERRA DESPEGA")
    HAL.takeoff(height)
  elif action == "Land" and current_state == 2:
    print("DRONE EN EL AIRE ATERRIZA")
    HAL.land()


def check_position(drone_position, goal, err):
  
  x_drone = drone_position[0]
  y_drone = drone_position[1]
  
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

def get_path_sweep(start, rect, n):

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

def start_sweep(path, drone_position):
  x, y = drone_position
  global height
  ind = 0
  
  while ind < len(path):
    x_goal, y_goal = path[ind]
    
    if check_position(drone_position, (x_goal, y_goal), 0.25):
      ind+=1
    else:
      # Controlarlo con velocidad, por ahora comandar posicion
      HAL.set_cmd_pos(x_goal, y_goal, height, 0)
      # Con velocidades, comprobar que eje de velocidad poner a cero
      
def get_linvel(distance):
    min_distance = 0.1
    max_distance = 5.0
    min_vel = 0.3
    max_vel = 0.7
    
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
  
def rotate_img(img, angle):

    (h, w) = img.shape[:2]
    center = (w / 2, h / 2)
    scale = 1

    M = cv2.getRotationMatrix2D(center, angle, scale)
    rotated = cv2.warpAffine(img, M, (w, h))
    
    return rotated
  
def detect_body(img_gray):

    (h, w) = img_gray.shape[:2]
    
    for j in range(h):
      for i in range(w):
        if img_gray[j][i] <= 20:
          return True
            
    return False
  
def detect_face(img_gray, detector, position_faces, drone_position):

    x_drone = round(drone_position[0], 2)
    y_drone = round(drone_position[1], 2)
    angle = 0
    results = []
    counter_faces = 0
    
    while angle <= 360:
      img_gray_rotated = rotate_img(img_gray, angle)
      results = detector.detectMultiScale(img_gray_rotated)
      
      if len(results) != 0:
        # Comprobar si la cara no está repetida.
        # 1. Tener array con la posicion de las caras
        # 2. Comprobar dentro del for si alguna de las caras detectadas tiene una posicion
        # en el array de caras detectadas. Si hay dos caras detectadas, descartar la que ya
        # se conoce la posicion.
        # 3. Guardar la posicion de la nueva cara
        for (x,y,w,h) in results:
          # Comprobar la distancia euclidea
          if len(position_faces) == 0:
            position_faces.append((x_drone, y_drone))
          else:
            dist_new_face = math.sqrt(x_drone**2 + y_drone**2)
            for p in position_faces:
              dist_current_face = math.sqrt(p[0]**2 + p[1]**2)
              print("Distancia Entre Actual-Nueva: ", abs(dist_current_face-dist_new_face))
              if abs(dist_current_face-dist_new_face) > 2.0:
                counter_faces += 1
          # Dibujar rectangulo en la cara de la persona detectada
          cv2.rectangle(img_gray_rotated, (x,y), (x+w,y+h), 255, 2)
          GUI.showLeftImage(img_gray_rotated)
        
        break
      
      angle += 30
    
    if counter_faces == len(position_faces):
      position_faces.append((x_drone, y_drone))


time.sleep(6)
path_dataset = '/RoboticsAcademy/exercises/static/exercises/rescue_people_newmanager/haarcascade_frontalface_default.xml'
face_detector=cv2.CascadeClassifier(path_dataset)

angle = 0

x0, y0, z0 = HAL.get_position()
yaw0 = HAL.get_yaw()
print("POSICION INICIAL: ", [x0, y0, z0, yaw0])

x0_goal = 40
y0_goal = -30

path = get_path_sweep((x0_goal+5, y0_goal), (20, 15), 5)

time_battery = 60.0*7
choose_takeoff_land("TakeOff")
takeoff_time = time.time()
print("Took Off")

vx, vy = 0, 0
on_axis_x = True # x:True - y:False
Kp = 0.35

orientated = False
localizated_people = False
reached_initial_goal = False
count_initial_pose = 0

ind = 0

people = [(39.51, -30.39), (36.86, -30.48), (25.04, -32.92), (40.02, -32.99), (25.72, -35.98), (40.45, -38.96)]

while True:
    x, y, z = HAL.get_position()
    x_goal, y_goal = people[ind]
    
    if check_position((x, y), (x_goal, y_goal), 0.2):
        HAL.set_cmd_mix(0, 0, height, 0)
        time.sleep(3)
        ind+=1
        # Si se termina el recorrido, se vuelve a empezar
        if ind >= len(path):
          print("REVISADO")
          time.sleep(20)
    
    HAL.set_cmd_pos(x_goal, y_goal, height, 0)
    print("")