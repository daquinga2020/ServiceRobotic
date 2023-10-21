import time
import numpy as np
import cv2
import queue
import math

def draw_grid(img, hight, width, size_cell_x, size_cell_y):
    
    # Implement grids on image
    # Lineas horizontales - Altura variable y anchura fija
    for i in range(0, hight, size_cell_y):
        cv2.line(img, (0, i), (width, i), (0, 0, 0), 1)

    # Lineas verticales - Altura fija y anchura variable
    for i in range(0, width, size_cell_x):
        cv2.line(img, (i, 0), (i, hight), (0, 0, 0), 1)
        

def check_obs(img, px, py, size_cell_x, size_cell_y):
    
    # Comprobar si en la celdilla en la que se encuentra hay un obstaculo
    for j in range(py+1, py + size_cell_y): # se puede borrar el 1 si elimino el grid
        for i in range(px+1, px + size_cell_x):
            if img[j,i] == 0:
                fill_cell(img, px, py, size_cell_x, size_cell_y, 0)
                return 1 # Hay obstaculo
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
    max_x = px + size_cell_x + 1
    max_y = py + size_cell_y + 1
    
    if max_x >= img.shape[1]:
        max_x = img.shape[1]-2
    if max_y >= img.shape[0]:
        max_y = img.shape[0]-2

    for j in range(py, max_y): # se puede borrar el 1 si elimino el grid
        for i in range(px, max_x):
            img[j][i] = color


def find_cell(cells, px_find, py_find, size_cell_x, size_cell_y):
    # Función para encontrar la celda a la que pertenecen las coordenadas
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
    
    for dir in ["North", "East", "South", "West"]:
        dx, dy = get_celldirection(dir,  size_cell_x, size_cell_y)
        nextx, nexty = int(current_x + dx), int(current_y + dy)
        nextCell = (nextx, nexty)
        if nextCell in cells:
            neighbors_cells.append( ( nextCell, dir) )
    
    return neighbors_cells


def get_pixel_center(cell, size_cell_x, size_cell_y):
    
    # Calcula el centro de la celda grande
    centre_y_big = size_cell_y // 2 + cell[1]
    centre_x_big = size_cell_x // 2 + cell[0]

    x_corner_small = centre_x_big - 1
    y_corner_small = centre_y_big - 1
    
    mean_x = (x_corner_small*2 + (x_corner_small+1)*2) / 4
    mean_y = (y_corner_small*2 + (y_corner_small+1)*2) / 4
    
    return (mean_x, mean_y)

def search_path2rtrnpnt(critical_point, goal_cell, f_cells, size_cell_x, size_cell_y):
    
    frontier_points = queue.PriorityQueue() # Cola para guardar celdillas
    expanded_cells = [] # Lista para guardar celdillas visitadas
    path_cells = {} # Diccionario para almacenar el camino a cada celdilla
    cost = {} # Diccionario para almacenar el coste del camino a cada celdilla
    
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
                
                # Si el vecino no tiene coste asignado o el nuevo coste es menor que el anterior
                if neighbor_cell not in cost or new_cost < cost[neighbor_cell]:
                    # Actualizamos el coste y el camino al vecino
                    cost[neighbor_cell] = new_cost
                    path = path_cells.get(current_cell, [])
                    path_with_direction = path + [(neighbor_cell, direction)]
                    path_cells[neighbor_cell] = path_with_direction
                    
                    # Agregamos el vecino a la cola de prioridad con el nuevo coste
                    frontier_points.put((new_cost, neighbor_cell))
    
    return []


def find_path(map, start, f_cells, size_cell_x, size_cell_y):
    
    color_visited = 220
    color_retrpnts = 100


    return_points = []
    vstd_cells = [] # Lista para guardar celdillas visitadas y ruta para limpiar
    directions = []
    current_direction = ""
    last_direction = ""
    current_cell = start
    cop_f_cells = f_cells.copy()
    # Seguir avanzando mientras hayan celdillas sin limpiar

    while len(f_cells) != 0:
        
        fill_cell(map, current_cell[0], current_cell[1], size_cell_x, size_cell_y, color_visited)
        neighbors = get_neighbors_cells(f_cells, current_cell, size_cell_x, size_cell_y)
        
        # Eliminar la celda en la que estamos de las celdas libres y añadirla al camino
        f_cells.remove(current_cell)
        center_pxl_cell = get_pixel_center(current_cell, size_cell_x, size_cell_y)
        vstd_cells.append(center_pxl_cell) # Cambiar por pixel center
        if len(vstd_cells) > 1:
            directions.append(current_direction)
        print(center_pxl_cell, current_direction)
        # Si la celda actual en la que estoy era un punto de retorno, se actualiza la lista de puntos de retorno
        if current_cell in return_points:
            return_points.remove(current_cell)
        
        # El primer vecino que haya es el movimiento que se va a realizar, el resto son
        # considerados puntos de retorno. Si no hay vecinos nos encontramos en un punto critico
        # y se procede a buscar el punto de retorno mas cercano.
        
        if len(neighbors) != 0:

            next_cell = neighbors[0][0]
            
            # Guardamos los vecinos como posibles puntos de retorno
            for cell_ngh in neighbors:
                if cell_ngh[0] not in return_points and cell_ngh[0] in f_cells:
                    return_points.append(cell_ngh[0])
                    last_direction = cell_ngh[1]
                    fill_cell(map, cell_ngh[0][0], cell_ngh[0][1], size_cell_x, size_cell_y, color_retrpnts)
            
            # Coger la siguiente celdilla a la que ir
            current_cell = next_cell
            current_direction = neighbors[0][1]
        else:
            # Mientras haya puntos de retorno seguir recorriendo
            
            if len(return_points) != 0:
                current_direction = last_direction
                
                pth_return_point, pth_directions = search_path2rtrnpnt(current_cell, return_points[-1], cop_f_cells, size_cell_x, size_cell_y) # Cambiarlo a pixeles centrales
                
                # Elimino el punto de retorno de la meta porque va a ser el siguiente current_cell y se añadira a la lista
                pth_return_point.pop(-1)
                pth_directions.pop(-1)
                temp_pth_return_point = []
                for cell in pth_return_point:
                    temp_pth_return_point.append(get_pixel_center(cell, size_cell_x, size_cell_y))
                
                # Se actualiza la ruta añadiendo el camino a un punto de retorno
                vstd_cells = vstd_cells + temp_pth_return_point
                directions = directions + pth_directions
                current_cell = return_points[-1]

        cv2.imshow('MAP', map)
        cv2.waitKey(1) # 80 buen tiempo
        
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


cv2.namedWindow('MAP', cv2.WINDOW_NORMAL)

map_img = cv2.imread('mapgrannyannie.png', 0) # Leer la imagen
map_img_rs = cv2.resize(map_img,(990,990))
h, w = map_img.shape
# IMAGEN ALTURA: 1012 ANCHO 1013; img[fila][columna]

# First step: apply erosion function on the map image to dilate obstacles on the map

# Se aplica erosion, una operacion morfologica, a la imagen del mapa y asi dilatar los obstaculos
kernel = np.ones((4,4), np.uint8)

# The first parameter is the original image,
# kernel is the matrix with which image is
# convolved and third parameter is the number
# of iterations, which will determine how much
# you want to erode/dilate a given image.
map_erosion = cv2.erode(map_img, kernel, iterations=12)
# Utilizar celdillas de 32x32 o 36x36, con 28 parece rellenar bastantes sitios
cell_x = 18
cell_y = 18

draw_grid(map_erosion, h, w, cell_x, cell_y)

#594, 522)
# (684, 522)
# for i in range(594, 685, 18):
#     fill_cell(map_erosion, i, 522, cell_x, cell_y, 0)


# Guardar las celdillas libres fuera de obstaculos
# Luego ubicar a nuestro robot en cual de esas celdillas esta.
free_cells = save_free_cells(map_erosion, h, w, cell_x, cell_y)

# Luego ubicar a nuestro robot en cual de esas celdillas del mapa esta.
# Para localizar al robot en el mapa se tomaran puntos conocidos en el mundo de Gazebo
# y se relacionaran con los pixeles del mapa, posteriormente se emplea un script en matlab
# con el que hallaremos un modelo de regresion lineal, obteniendo de esta manera la posicion del robot este donde este.

# Obtener la posicion del robot inicial
x0_rob = -1 # Si se avanza a la izquierda se suma en pos
y0_rob = 1.5 # Si se avanza a la izquierda se suma en pos
yaw0_rob = 0 #-3.77
# 1290

# Prediccion obtenida del modelo de regresion lineal
x_start = 679
y_start = 577

# Una vez sabemos en que pixel se cree que esta, definir en que
# celdilla se encuentra.
start_cell = find_cell(free_cells, x_start, y_start, cell_x, cell_y)

# Una vez localizado al robot en la celdilla se procede a crear el path
# para recorrer todas las celdillas de la casa.
# Prioridad para avanzar, Norte, Este, Sur, Oeste

# Diferenciar en el mapa los Obstaculos(0), Obstaculos virtuales(celdas ya visitadas por el robot,c=210),
# Puntos de Retorno(Puntos de inicio para comenzar a limpiar de nuevo, 240) y Puntos Criticos(Puntos donde
# el robot se queda atascado entre obstaculos y obstaculos virtuales).

# Para trazar el path se empleara el algoritmo de busqueda en coste uniforme.
# Los puntos de retorno serám comprobados mientras que el robot se encuentra en movimiento. 
# Las celdas vacias se clasificaran como puntos de retorno. La lista de los puntos ha de ser actualizda continuamente.

# Los puntos criticos seran clasificados cuando el robot este atascado entre obstaculos o obstaculos virtuales y no se pueda mover.

# Para comprbar la llegada de la celda se consideraran varios offsets de errores en el mundo real. La llegada del robot deberia
# ser considerada con un pequeño margen de error, de la otra manera el robot comenzaria a oscilar.

# Algoritmo BSA
# Ira fuera del bucle while
# Obtener las celdillas vecinas de la celdilla actual y avanzar al norte si es posible.
time.sleep(8)
cop = free_cells.copy()
visited_cells, dirs = find_path(map_erosion, start_cell, cop, cell_x, cell_y)
visited_cells.pop(0)

# print(594, 522)
# fill_cell(map_erosion, 594, 522, cell_x, cell_y, 100)
# print(684, 522)
# fill_cell(map_erosion, 684, 522, cell_x, cell_y, 100)
# cv2.imshow('MAP', map_erosion)
print(len(visited_cells), len(dirs))
for i in range(len(visited_cells)):
    print(visited_cells[i], dirs[i])

# fill_cell(map_erosion, start_cell[0], start_cell[1], cell_x, cell_y, 100)
# pr_pth = search_path2rtrnpnt(start_cell, (936, 558), cop, cell_x, cell_y)

# for i in pr_pth:
#     fill_cell(map_erosion, i[0], i[1], cell_x, cell_y, 200)
#     cv2.imshow('MAP', map_erosion)
#     cv2.waitKey(1000)



# Se avanzan 3 al norte: (674.5, 566.5), (674.5, 548.5), (674.5, 530.5),
# print(visited_cells)
# x, y = 810, 954
# fill_cell(map_erosion, x, y, cell_x, cell_y, 200)
# cv2.imshow('MAP', map_erosion)

# x_map = [936, 936, 702, 576, 504, 342, 126, 72, 72, 234, 216, 72, 72, 72, 810]
# y_map = [558, 342, 342, 342, 342, 144, 144, 108, 540, 540, 792, 792, 918, 954, 954]
# a = []
# for x, y in zip(x_map, y_map):
#     a.append(get_pixel_center((x,y), cell_x, cell_y))
# print(a)
# Transformar las coordenadas de pixeles en coordenadas del mundo Gazebo
'''
# Pesos para x_gz a x_pixel
X_gz2pxl_x1 = 571.65
X_gz2pxl_x2 = -100.81

# Pesos para y_gz a y_pixel
Y_gz2pxl_x1 = 412.64
Y_gz2pxl_x2 = 99.373

# Modelo de regresion lineal para x_gz a x_pixel
# x_pixel = X_x1 + X_x2*x_gz

# Modelo de regresion lineal para y_gz a y_pixel
# y_pixel = Y_x1 + Y_x2*y_gz


# Pesos para x_pixel a x_gz
X_pxl2gz_x1 = 5.6697
X_pxl2gz_x2 = -0.0099174
# Pesos para y_pixel a y_gz
Y_pxl2gz_x1 = -4.1493
Y_pxl2gz_x2 = 0.010058

# Modelo de regresion lineal para x_gz a x_pixel
# x_pixel = X_x1 + X_x2*x_gz

# Modelo de regresion lineal para y_gz a y_pixel
# y_pixel = Y_x1 + Y_x2*y_gz

gz_pth = []

for i in range(len(visited_cells)):
    x_gz = round(X_pxl2gz_x1 + X_pxl2gz_x2*visited_cells[i][0], 3)
    y_gz = round(Y_pxl2gz_x1 + Y_pxl2gz_x2*visited_cells[i][1], 3)
    gz_pth.append((x_gz, y_gz))
    print("Coordenada en pixel: ", visited_cells[i], " -> Coordenada en gazebo: ", (x_gz, y_gz), "DIR:", dirs[i])
'''

'''def turn(dir):
    
    alineated = False
    current_dir = 0
    err = 0.002
    
    while not alineated:
        
        if dir == "North":
            orientation = round(-math.pi/2, 3)
        elif dir == "East":
            orientation = round(-math.pi, 3)
        elif dir == "South":
            orientation = round(math.pi/2, 3)
        else:
            orientation = 0.0

        difference = orientation - current_dir
        
        if -err <= difference <= err:
            HAL.setW(0.5)
            alineated = True'''

# x0_rob = -1 # Si se avanza a la izquierda se suma en pos
# y0_rob = 1.5 # Si se avanza a la izquierda se suma en pos
# yaw0_rob = 0 #-3.77

# Primitivas de movimiento:
# Orientacion inicial -> 0 mirando a la izquierda, oeste.
# -1.5 -> mirando hacia arriba norte
# -3.14 -> mirando hacia derecha este
# 1.5 -> mirando hacia abajo sur
# W+ -> giro derecha


def convert_pxls2gz(coords_pxls, X_pxl2gz, Y_pxl2gz):
    
    coords_gz = []
    
    for pxl in coords_pxls:
        gz_x = round(X_pxl2gz[0] + X_pxl2gz[1]*pxl[0], 3)
        gz_y = round(Y_pxl2gz[0] + Y_pxl2gz[1]*pxl[1], 3)
        coords_gz.append((gz_x, gz_y))
    
    return coords_gz

def get_local_goal(dirs, path, index, n_points_skip):

    if index > len(path)-1:
        index = len(path)-1
        
    max_index = min(index + n_points_skip, len(path))
    local_goal = path[index]
    x_start = path[index][0]
    y_start = path[index][1]
    dir_start = dirs[index]
    new_index = index + 1
    
    if dir_start == dirs[min(index+1, len(path)-1)]:
        for i in range(index+1, max_index):
            if (x_start == path[i][0] or y_start == path[i][1]) and dir_start == dirs[i]:
                local_goal = path[i]
                new_index = i + 1
        
    
    return new_index, local_goal

# Pesos de Gazebo a Pixel
# Pesos para x_pixel a x_gz
X_gz2pxl_x1 = 582.24
X_gz2pxl_x2 = -96.794
# Pesos para y_gz a y_pixel
Y_gz2pxl_x1 = 432.18
Y_gz2pxl_x2 = 96.804


# Pesos de Pixel a Gazebo
# Pesos para x_pixel a x_gz
X_pxl2gz_x1 = 5.991
X_pxl2gz_x2 = -0.010269
# Pesos para y_pixel a y_gz
Y_pxl2gz_x1 = -4.4614
Y_pxl2gz_x2 = 0.010324

pth_absolute = convert_pxls2gz(visited_cells, (X_pxl2gz_x1, X_pxl2gz_x2), (Y_pxl2gz_x1, Y_pxl2gz_x2))

index = 0
n_points_skip = 16
print(visited_cells)

while index < len(pth_absolute):
    print("CELDA:", visited_cells[index], dirs[index], "PUNTO: ", pth_absolute[index])
    index, local_goal_abs = get_local_goal(dirs, pth_absolute, index, n_points_skip)
    print("CELDA:", visited_cells[index-1])
    print("\tINDICE: ", index, "META LOCAL: ", local_goal_abs, dirs[index-1])
    print("")
print("SALGO")
'''# Mantener el robot alineado con el centro de la celda
# Avanzar las celdas disponibles en un mismo sentido siendo maximo tres
max_cells = 3
current_ind = 0

last_x = 0.
last_y = 0.'''
# Comprobar cuantas celdillas van en el mismo sentido
# for i in range(max_cells):
#     gz_pth[current_ind]

cv2.waitKey(0) # La imagen se sigue mostrando segun el parametro que se pase, si es 0 se muestra de forma infinita

