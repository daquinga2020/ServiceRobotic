import queue
import time
from GUI import GUI
from HAL import HAL
import numpy as np
import cv2
import math
# Enter sequential code!

def draw_grid(img, hight, width, size_cell_x, size_cell_y):
    
    # Implement grids on image
    
    for i in range(0, hight, size_cell_y):
        cv2.line(img, (0, i), (width, i), (0, 0, 0), 1)


    for i in range(0, width, size_cell_x):
        cv2.line(img, (i, 0), (i, hight), (0, 0, 0), 1)


def check_obs(img, px, py, size_cell_x, size_cell_y):
    
    # Check if there any obstacle in the cell
    for j in range(py+1, py + size_cell_y):
        for i in range(px+1, px + size_cell_x):
            if img[j,i] == 0:
                fill_cell(img, px, py, size_cell_x, size_cell_y, 0)
                return 1
    return 0


def save_free_cells(img, hight, width, size_cell_x, size_cell_y):
    
    f_cells = []
    
    for j in range(0, hight, size_cell_y):
        for i in range(0, width, size_cell_x):
            if check_obs(img, i, j, size_cell_x, size_cell_y) != 1:
                f_cells = f_cells + [(i,j)]

    return f_cells

def fill_cell(img, px, py, size_cell_x, size_cell_y, color):
    
    # Rellenar celda si ha pasado por ahi
    max_x = px + size_cell_x
    max_y = py + size_cell_y
    
    if max_x >= img.shape[1]:
        max_x = img.shape[1]-1
    if max_y >= img.shape[0]:
        max_y = img.shape[0]-1

    for j in range(py+1, max_y): # se puede borrar el 1 si elimino el grid
        for i in range(px+1, max_x):
            img[j][i] = color


def find_cell(cells, px_find, py_find, size_cell_x, size_cell_y):
  
    for cell in cells:
        x_corner, y_corner = cell
        if px_find >= x_corner and px_find < x_corner + size_cell_x and py_find >= y_corner and py_find < y_corner + size_cell_y:
            return (x_corner, y_corner)

    return None

def get_celldirection(direction, size_cell_x, size_cell_y):
    
    dx = 0
    dy = 0
    
    if direction == "North":
        dy = -size_cell_y
    elif direction == "South":
        dy = size_cell_y
    elif direction == "East":
        dx = size_cell_x
    else:
        dx = -size_cell_x
    
    return (dx, dy)

def get_neighbors_cells(cells, current_cell, size_cell_x, size_cell_y):
    
    current_x = current_cell[0]
    current_y = current_cell[1]
    
    neighbors_cells = []
    
    for dir in ["North", "West", "South", "East"]:
        dx, dy = get_celldirection(dir,  size_cell_x, size_cell_y)
        nextx, nexty = int(current_x + dx), int(current_y + dy)
        nextCell = (nextx, nexty)
        if nextCell in cells:
            neighbors_cells.append( ( nextCell, dir) )
    
    return neighbors_cells


def get_pixel_center(cell, size_cell_x, size_cell_y):
    
    # Get center cell
    centre_y_big = size_cell_y // 2 + cell[1]
    centre_x_big = size_cell_x // 2 + cell[0]

    x_corner_small = centre_x_big - 1
    y_corner_small = centre_y_big - 1
    
    mean_x = (x_corner_small*2 + (x_corner_small+1)*2) / 4
    mean_y = (y_corner_small*2 + (y_corner_small+1)*2) / 4
    
    return (mean_x, mean_y)


def search_path2rtrnpnt(critical_point, goal_cell, f_cells, size_cell_x, size_cell_y):
    
    frontier_points = queue.PriorityQueue() # Queue to safe cells
    expanded_cells = [] # List to safe visited cells
    path_cells = {} # Dictionary to safe the path
    cost = {} # Dictionary to safe the cost of the path
    
    frontier_points.put((0, critical_point))

    while not frontier_points.empty():
        current_cost, current_cell = frontier_points.get()

        if current_cell not in expanded_cells:
            if current_cell == goal_cell:
                coordinates = [cell[0] for cell in path_cells[current_cell]]
                directions = [cell[1] for cell in path_cells[current_cell]]
                return coordinates, directions
            
            expanded_cells.append(current_cell)
            neighbors = get_neighbors_cells(f_cells, current_cell, size_cell_x, size_cell_y)

            for neighbor_cell, direction in neighbors:
                new_cost = current_cost + 1
                
                if neighbor_cell not in cost or new_cost < cost[neighbor_cell]:
                    cost[neighbor_cell] = new_cost
                    path = path_cells.get(current_cell, [])
                    path_with_direction = path + [(neighbor_cell, direction)]
                    path_cells[neighbor_cell] = path_with_direction
                    
                    frontier_points.put((new_cost, neighbor_cell))
    
    return []

def get_nearest_rtrnpnt(return_points, current_point):
    
    nearest_point = None
    nearest_distance = 1000000
    x, y = current_point
    
    for pnt in return_points:
        current_distance = math.sqrt(((pnt[0]-x)**2+(pnt[1]-y)**2))
        
        if current_distance <= nearest_distance:
            nearest_distance = current_distance
            nearest_point = pnt
        
    return nearest_point

def find_path(map, start, f_cells, size_cell_x, size_cell_y):
    
    return_points = []
    vstd_cells = []
    directions = []
    current_direction = ""
    last_direction = ""
    
    current_cell = start
    cop_f_cells = f_cells.copy()

    while len(f_cells) != 0:
        
        neighbors = get_neighbors_cells(f_cells, current_cell, size_cell_x, size_cell_y)
        
        f_cells.remove(current_cell)
        center_pxl_cell = get_pixel_center(current_cell, size_cell_x, size_cell_y)
        vstd_cells.append(center_pxl_cell)
        if len(vstd_cells) > 1:
            directions.append(current_direction)
        
        if current_cell in return_points:
            return_points.remove(current_cell)
        
        if len(neighbors) != 0:

            next_cell = neighbors[0][0]
            
            for cell_ngh in neighbors:
                if cell_ngh[0] not in return_points and cell_ngh[0] in f_cells:
                    return_points.append(cell_ngh[0])
                    last_direction = cell_ngh[1]
            
            current_cell = next_cell
            current_direction = neighbors[0][1]
        else:
            
            if len(return_points) != 0:
                current_direction = last_direction
                
                near_rtnr_pnt = get_nearest_rtrnpnt(return_points, current_cell)
                pth_return_point, pth_directions = search_path2rtrnpnt(current_cell, near_rtnr_pnt, cop_f_cells, size_cell_x, size_cell_y) # Cambiarlo a pixeles centrales

                pth_return_point.pop(-1)
                pth_directions.pop(-1)
                
                temp_pth_return_point = []
                for cell in pth_return_point:
                    temp_pth_return_point.append(get_pixel_center(cell, size_cell_x, size_cell_y))
                
                vstd_cells = vstd_cells + temp_pth_return_point
                directions = directions + pth_directions
                current_cell = near_rtnr_pnt
        
    return vstd_cells, directions

    

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

def convert_pxls2gz(coords_pxls, X_pxl2gz, Y_pxl2gz):
    
    coords_gz = []
    
    for pxl in coords_pxls:
        gz_x = round(X_pxl2gz[0] + X_pxl2gz[1]*pxl[0], 3)
        gz_y = round(Y_pxl2gz[0] + Y_pxl2gz[1]*pxl[1], 3)
        coords_gz.append((gz_x, gz_y))
    
    return coords_gz

def get_local_goal(dirs, path, index, n_points_skip):

    if index >= len(path):
        index = len(path)-1
        
    max_index = min(index + n_points_skip, len(path)-1)
    local_goal = path[index]
    x_start = path[index][0]
    y_start = path[index][1]
    dir_start = dirs[index]
    new_index = index + 1
    new_dir = dirs[index]
    
    if dir_start == dirs[min(index+1, len(path)-1)]:
        for i in range(index+1, max_index):
            if (x_start == path[i][0] or y_start == path[i][1]) and dir_start == dirs[i]:
                local_goal = path[i]
                new_index = i + 1
                new_dir = dirs[i]
        
    
    return new_index, local_goal, new_dir

# def go_start_position(start_abs):
    
#     localized = False
    
#     while not localized:
#         rob_x = HAL.getPose3d().x
#         rob_y = HAL.getPose3d().y
#         theta_rob = HAL.getPose3d().yaw
#         print("META GLOBAL: ", start_abs)
#         print("POS ACTUAL", (round(rob_x, 4), round(rob_y, 4)))
#         # Situar en (674.5, 584.5)
#         # Obtener punto relativo al sistema de coordenadas del robot
#         start_rel = absolute2relative(start_abs[0], start_abs[1], rob_x, rob_y, theta_rob)
#         distance_start = math.sqrt(start_rel[0]**2 + start_rel[1]**2)
        
#         if distance_start <= 0.0057:
#             localized = True
#             #time.sleep(5)
                
#         w = -math.atan2(start_rel[1], start_rel[0])
        
#         if math.fabs(w) <= 0.0025:
#             w = 0
#             v = 0.6
#         else:
#             v = 0.01
            
#         print("ANGULO: ", w, math.degrees(w), "->", w*1.9)
#         print("DISTANCIA:", distance_start)
#         print("")
            
#         HAL.setW(w*1.9)
#         HAL.setV(v)

# def turn(dir, robott):
    
#     if dir == "North":
#         desire_dir = -math.pi/2
#     elif dir == "East":
#         # Puede ser pi o -pi
#         if robott < 0:
#             desire_dir = -math.pi
#         else:
#             desire_dir = math.pi
#     elif dir == "South":
#         desire_dir = math.pi/2
#     elif dir == "West":
#         desire_dir = 0.0
    
#     error = round(robott - desire_dir, 4)
    
#     return error
    
def get_linvel(distance):
    min_distance = 0.2
    min_vel = 0.6
    max_vel = 2.7

    if distance < min_distance:
        return min_vel
    elif distance >= min_distance and distance <= 0.5:
        slope = (max_vel - min_vel) / (0.5 - min_distance)
        vel = slope * (distance - min_distance) + min_vel
        return vel
    else:
        return max_vel


map_img = cv2.imread('RoboticsAcademy/exercises/static/exercises/vacuum_cleaner_loc/resources/images/mapgrannyannie.png', 0) # Leer la imagen
map_img_rs = cv2.resize(map_img, (720, 720))

h, w = map_img_rs.shape

# Se aplica erosion, una operacion morfologica, a la imagen del mapa y asi dilatar los obstaculos
kernel = np.ones((3,3), np.uint8)
map_erosion = cv2.erode(map_img_rs, kernel, iterations=11)

cell_x = 18
cell_y = 18

draw_grid(map_erosion, h, w, cell_x, cell_y)

# Guardar las celdillas libres fuera de obstaculos
# Luego ubicar a nuestro robot en cual de esas celdillas esta.
free_cells = save_free_cells(map_erosion, h, w, cell_x, cell_y)

'''matrix = np.zeros((900, 900), dtype=np.uint8)
for j in range(900):
  for i in range(900):
    matrix[j][i] = map_erosion[j][i]
    
GUI.showNumpy(matrix)'''

time.sleep(3)
x0_rob = HAL.getPose3d().x # Si se avanza a la izquierda se suma en pos
y0_rob = HAL.getPose3d().y
yaw0_rob = HAL.getPose3d().yaw

# Pesos de Gazebo a Pixel
# Pesos para x_pixel a x_gz
X_gz2pxl_x1 = 412.07
X_gz2pxl_x2 = -70.854
# Pesos para y_gz a y_pixel
Y_gz2pxl_x1 = 295.16
Y_gz2pxl_x2 = 69.773


# Pesos de Pixel a Gazebo
# Pesos para x_pixel a x_gz
X_pxl2gz_x1 = 5.811
X_pxl2gz_x2 = -0.014097
# Pesos para y_pixel a y_gz
Y_pxl2gz_x1 = -4.2271
Y_pxl2gz_x2 = 0.014323


# Prediccion obtenida del modelo de regresion lineal
pix_x_start = round(X_gz2pxl_x1 + X_gz2pxl_x2*x0_rob)
pix_y_start = round(Y_gz2pxl_x1 + Y_gz2pxl_x2*y0_rob)

start_cell = find_cell(free_cells, pix_x_start, pix_y_start, cell_x, cell_y)

visited_cells, dirs = find_path(map_erosion, start_cell, free_cells, cell_x, cell_y)

# Ruta que va a realizar el robot por toda la casa
pth_absolute = convert_pxls2gz(visited_cells, (X_pxl2gz_x1, X_pxl2gz_x2), (Y_pxl2gz_x1, Y_pxl2gz_x2))

index = 0
n_points_skip = 12
local_reached = False

# Se ubica al robot en la primera celdilla, el inicio
# go_start_position(pth_absolute[0])
pth_absolute.pop(0)
# Se obtiene la primera posicion a la que dirigirse
index, local_goal_abs, dir = get_local_goal(dirs, pth_absolute, index, n_points_skip)
# 0 (oeste), -PI/2 (norte), PI (este) o PI/2 (sur). 

w = 0
v = 0
last_distance = -1
count = 0
while True:
    # Enter iterative code!
    rob_x = HAL.getPose3d().x
    rob_y = HAL.getPose3d().y
    theta_rob = HAL.getPose3d().yaw
    
    # Calculo del punto relativo al sistema de coordenadas del robot y su distancia al mismo
    local_goal_relative = absolute2relative(local_goal_abs[0], local_goal_abs[1], rob_x, rob_y, theta_rob)
    distance_goal = math.sqrt(local_goal_relative[0]**2 + local_goal_relative[1]**2)
    
    if last_distance == -1:
      last_distance = distance_goal
      
    
    # Si se alcanza la meta, motores a cero y se define una nueva meta local
    if local_reached:
      HAL.setW(0)
      HAL.setV(0)
      print("META GLOBAL ALCANZADA: ", pth_absolute[index])
      local_reached = False
      if index >= len(pth_absolute):
          print("RUTA FINALIZADA")
          time.sleep(60)
      index, local_goal_abs, dir = get_local_goal(dirs, pth_absolute, index, n_points_skip)
      local_goal_relative = absolute2relative(local_goal_abs[0], local_goal_abs[1], rob_x, rob_y, theta_rob)
      distance_goal = math.sqrt(local_goal_relative[0]**2 + local_goal_relative[1]**2)
      count = 0
      last_distance = distance_goal
      print("NUEVA META GLOBAL: ", pth_absolute[index], "POSICION ACTUAL: ", (rob_x, rob_y), "GIRO:", dir)
      #time.sleep(2)
    
    # Si la distancia es menor que la minima se considera alcanzada la meta local
    '''if last_distance > 1.0:
      if distance_goal <= 0.2:
        local_reached = True
    else:
      if distance_goal <= 0.05:
        local_reached = True'''
    if distance_goal <= 0.075:
      local_reached = True

  
    
    w = -math.atan2(local_goal_relative[1], local_goal_relative[0])*2.35 # 2.2
    print("META GLOBAL: ", pth_absolute[index])
    print("DISTANCIA:", round(distance_goal, 4))
    # Pruebas de giro
    #w = turn(dir, theta_rob)
    print("W:", w, "GIRO:", dir, "POS ACTUAL", (round(rob_x, 4), round(rob_y, 4)))
    print("")
    
    #if math.fabs(w) <= 0.002:
    if math.fabs(w) <= 0.007:
        w = 0
        v = get_linvel(distance_goal)
    else:
        v = 0.27
    
    HAL.setW(w)
    HAL.setV(v)
    
    if (last_distance - distance_goal) < 0 and w == 0:
      count+=1
      print("DISTANCIA INICIAL", last_distance, "DISTANCIA ACTUAL:", distance_goal)
      
      if count >= 35:
        print("############### TENGO QUE REUBICARME ###############")
        # go_start_position(pth_absolute[index])
    else:
      count = 0
    
    
    
    '''
    print("Posicion Movimiento: ", (round(rob_x, 5), round(rob_y, 5)))
    time.sleep(5)'''
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    