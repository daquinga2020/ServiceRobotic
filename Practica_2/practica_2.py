from GUI import GUI
from HAL import HAL
import cv2
import time
import math

height = 2.0

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
    max_vel = 0.6
    
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
        if img_gray[j][i] <= 30:
          return True
            
    return False
  
def detect_face(img_gray, detector, position_faces, drone_position):

    x_drone = round(drone_position[0], 2)
    y_drone = round(drone_position[1], 2)
    angle = 0
    results = []
    counter_faces = 0
    ind = 0
    
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
        counter_faces += 1
        for (x,y,w,h) in results:
          cv2.rectangle(img_gray_rotated, (x,y), (x+w,y+h), 255, 2)
          GUI.showLeftImage(img_gray_rotated)
        
        
        break
      
      angle += 20
    
    if counter_faces != 0:
      for p in position_faces:
        dist_new_face = math.sqrt((p[0] - x_drone)**2 + (p[1] - y_drone)**2)
        print("Dist Act-New: ", dist_new_face)
        if dist_new_face > 1.75:
          ind += 1
      
      if ind == len(position_faces):
        position_faces.append((x_drone, y_drone))
    #


time.sleep(6)
path_dataset = '/RoboticsAcademy/exercises/static/exercises/rescue_people_newmanager/haarcascade_frontalface_default.xml'
face_detector=cv2.CascadeClassifier(path_dataset)

angle = 0

x0, y0, z0 = HAL.get_position()
yaw0 = HAL.get_yaw()
print("POSICION INICIAL: ", [x0, y0, z0, yaw0])

x0_goal = 40
y0_goal = -30

path = get_path_sweep((x0_goal+3, y0_goal), (20, 11), 9)
x0_goal, y0_goal = path[0]

time_battery = 60.0*15
choose_takeoff_land("TakeOff")
takeoff_time = time.time()
print("Took Off")

vx, vy = 0, 0
on_axis_x = True # x:True - y:False
Kp = 0.55

orientated = False
localizated_people = False
reached_initial_goal = False
count_initial_pose = 0
ind = 1

people = []

while True:
  
    state_dron = HAL.get_landed_state()
    
    # Si esta en el aire hace cosas
    if state_dron == 2 and not localizated_people:
      x, y, z = HAL.get_position()
      yaw = HAL.get_yaw()
      print("POSICION: ", [round(x, 4), round(y, 4)])
      
      # Si se acaba la bateria regreso al punto inicial a recargar
      if time.time()-takeoff_time >= time_battery:
        # Acudir al punto inicial, hasta que no llegue no Aterrizar
        HAL.set_cmd_pos(0, 0, height, yaw0)
        # Aterrizar
        print("GO STATION CHARGE")
        if check_position((x,y), (0,0), 0.1):
          count_initial_pose += 1
          if count_initial_pose == 10:
            print("LANDING")
            count_initial_pose = 0
            choose_takeoff_land("Land")
            reached_initial_goal = False
            # Reseteo de parametros de la busqueda
            # on_axis_x = True
            orientated = False
            # ind = 1
      else:
        if check_position((x, y), (x0_goal, y0_goal), 0.1) or reached_initial_goal:
          # Analizar y mostrar imagen cuando se detecte un color negro
          if not reached_initial_goal:
            time.sleep(2)
            print("START SEARCH")
            
          if len(people) != 0:
            print("Personas Encontradas: ", len(people))
            for p in people:
              print("Persona Localizada  --------> ", p)
            
          ventral_img = HAL.get_ventral_image()
          h, w = ventral_img.shape[:2]
          center_x, center_y = w // 2, h//2
          mid_h, mid_w = int(h // 4), int(w // 4)

          start_y, end_y = center_y - mid_h, center_y + mid_h
          start_x, end_x = center_x - mid_w, center_x + mid_w

          recorted_img = ventral_img[start_y:end_y, start_x:end_x]
          img_gray = cv2.cvtColor(recorted_img, cv2.COLOR_BGR2GRAY)
          GUI.showImage(ventral_img)

          if detect_body(img_gray):
            # Proceso la imagen
            GUI.showLeftImage(img_gray)
            detect_face(img_gray, face_detector, people, (x, y))
            
            #
          
          reached_initial_goal = True
          x_goal, y_goal = path[ind]
          
          if check_position((x, y), (x_goal, y_goal), 0.25):
            HAL.set_cmd_mix(0, 0, height, 0)
            # time.sleep(1)
            ind+=1
            on_axis_x = not on_axis_x
            orientated = False
            # Si se termina el recorrido, se vuelve a empezar
            if ind >= len(path):
              ind = 0
              on_axis_x = True
              orientated = False
          else:
            goal_rel_x, goal_rel_y = absolute2relative(x_goal, y_goal, x, y, yaw)
            dist_x = math.sqrt((goal_rel_x**2))
            dist_y = math.sqrt((goal_rel_y**2))
            w = math.atan2(goal_rel_y, goal_rel_x)*Kp
            
            sense_x = (goal_rel_x)/abs(goal_rel_x)
            sense_y = (goal_rel_y)/abs(goal_rel_y)
            
            vx = get_linvel(dist_x)
            vy = get_linvel(dist_y)
            
            # print("Distance:", (round(dist_x, 4), round(dist_y, 4)))

            # Saber hacia que eje me muevo, si en X o en Y
            # Asignar velocidad minima al eje en el que no se debe mover para evitar desviarse de la trayectoria
            
            if math.fabs(w) <= 0.01 and not orientated:
              orientated = True
              w = 0
            elif orientated:
              w = 0
              vx = vx * sense_x
              vy = 0.075 * sense_y
            else:
              vx = 0
              vy = 0
            
            # print("VX:", vx, "VY:", vy, "W:", w)
            HAL.set_cmd_mix(vx, vy, height, w)
            #
        elif not reached_initial_goal:
          HAL.set_cmd_pos(x0_goal, y0_goal, height, math.pi)
    
    if state_dron == 4:
      print("LANDING")
      
    if state_dron == 1 and not localizated_people:
      print("Recharging Batteries")
      takeoff_time = time.time()
      choose_takeoff_land("TakeOff")
      print("Taking Off Again")
  
    
    print("")