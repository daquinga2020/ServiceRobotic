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
    
    for dir in ["North", "West", "South", "East"]:
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
        # print(center_pxl_cell, current_direction)
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
                
                near_rtnr_pnt = get_nearest_rtrnpnt(return_points, current_cell)
                
                
                pth_return_point, pth_directions = search_path2rtrnpnt(current_cell, near_rtnr_pnt, cop_f_cells, size_cell_x, size_cell_y) # Cambiarlo a pixeles centrales
                for pnt in pth_return_point:
                    fill_cell(map, pnt[0], pnt[1], size_cell_x, size_cell_y, 180)
                    cv2.imshow('MAP', map)
                    cv2.waitKey(100)
                    
                # Elimino el punto de retorno de la meta porque va a ser el siguiente current_cell y se añadira a la lista
                pth_return_point.pop(-1)
                pth_directions.pop(-1)
                
                temp_pth_return_point = []
                for cell in pth_return_point:
                    temp_pth_return_point.append(get_pixel_center(cell, size_cell_x, size_cell_y))
                
                # Se actualiza la ruta añadiendo el camino a un punto de retorno
                vstd_cells = vstd_cells + temp_pth_return_point
                directions = directions + pth_directions
                current_cell = near_rtnr_pnt

        cv2.imshow('MAP', map)
        cv2.waitKey(100) # 80 buen tiempo
        
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


# cv2.namedWindow('MAP', cv2.WINDOW_NORMAL)

map_img = cv2.imread('mapgrannyannie.png', 0) # Leer la imagen
map_img_rs = cv2.resize(map_img, (720, 720))

h, w = map_img_rs.shape

# Se aplica erosion, una operacion morfologica, a la imagen del mapa y asi dilatar los obstaculos
kernel = np.ones((3,3), np.uint8)
map_erosion = cv2.erode(map_img_rs, kernel, iterations=11)

cell_x = 18
cell_y = 18
draw_grid(map_erosion, h, w, cell_x, cell_y)
cv2.imshow('MAP', map_erosion)
cv2.waitKey(5000)
free_cells = save_free_cells(map_erosion, h, w, cell_x, cell_y)

print(len(free_cells))

x0_rob = -1
y0_rob = 1.5
i = 97

# Prediccion obtenida del modelo de regresion lineal
x_start = 484 # free_cells[i][0] # free_cells[222][0]
y_start = 401 # free_cells[i][1] # free_cells[222][1]
'''fill_cell(map_erosion, x_start, y_start, cell_x, cell_y, 100)

center = get_pixel_center((x_start, y_start), cell_x, cell_y)
fill_cell(map_erosion, int(center[0]-0.5), int(center[1]-0.5), 2, 2, 200)

print(free_cells[i], center)
cv2.imshow('MAP', map_erosion)'''

# Una vez sabemos en que pixel se cree que esta, definir en que
# celdilla se encuentra.
start_cell = find_cell(free_cells, x_start, y_start, cell_x, cell_y)
# center = get_pixel_center(start_cell, cell_x, cell_y)
# fill_cell(map_erosion, start_cell[0], start_cell[1], cell_x, cell_y, 100)
# fill_cell(map_erosion, int(center[0]-0.5), int(center[1]-0.5), 2, 2, 200)
cv2.imshow('MAP', map_erosion)

cop = free_cells.copy()
visited_cells, dirs = find_path(map_erosion, start_cell, cop, cell_x, cell_y)

visited_cells.pop(0)



cv2.waitKey(0)

