import time
import numpy as np
import cv2
import util

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
            # cells.remove(cell)
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
    
    cost = 1
    neighbors_cells = []
    
    for dir in ["North", "East", "South", "West"]:
        dx, dy = get_celldirection(dir,  size_cell_x, size_cell_y)
        nextx, nexty = int(current_x + dx), int(current_y + dy)
        nextCell = (nextx, nexty)
        if nextCell in cells:
            neighbors_cells.append( ( nextCell, dir, cost) )
    
    return neighbors_cells

map_img = cv2.imread('mapgrannyannie.png', 0) # Leer la imagen
h, w = map_img.shape
# IMAGEN ALTURA: 1012 ANCHO 1013; img[fila][columna]
# cv2.imshow('ORIGINAL MAP', map_img)

# First step: apply erosion function on the map image to dilate obstacles on the map

# Se aplica erosion, una operacion morfologica, a la imagen del mapa y asi dilatar los obstaculos
kernel = np.ones((5,5), np.uint8)

# The first parameter is the original image,
# kernel is the matrix with which image is
# convolved and third parameter is the number
# of iterations, which will determine how much
# you want to erode/dilate a given image.
map_erosion = cv2.erode(map_img, kernel, iterations=3)

# Posicion del robot inicial
x0_rob = -1
y0_rob = -1
yaw0_rob = -3.77

# Utilizar celdillas de 32x32 o 36x36, con 28 parece rellenar bastantes sitios
cell_x = 18
cell_y = 18

draw_grid(map_erosion, h, w, cell_x, cell_y)

# Guardar las celdillas libres fuera de obstaculos
# Luego ubicar a nuestro robot en cual de esas celdillas esta.
free_cells = save_free_cells(map_erosion, h, w, cell_x, cell_y)

# Luego ubicar a nuestro robot en cual de esas celdillas del mapa esta.
# Para localizar al robot en el mapa se tomaran puntos conocidos en el mundo de Gazebo
# y se relacionaran con los pixeles del mapa, posteriormente se emplea un script en matlab
# con el que hallaremos un modelo de regresion lineal, obteniendo de esta manera la posicion del robot este donde este.

# Obtener la posicion del robot inicial
color_visited = 220
color_retrpnts = 100
x0_rob = -1 # Si se avanza a la izquierda se suma en pos
y0_rob = 1.5 # Si se avanza a la izquierda se suma en pos
yaw0_rob = 0 #-3.77
# 1290

# Prediccion obtenida del modelo de regresion lineal
x_start = 672
y_start = 562

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
# Obtener las celdillas vecinas de la celdilla actual y abvanzar al norte si es posible.
# neighbors = get_neighbors_cells(free_cells, start_cell, cell_x, cell_y)
# print("VECINOS: ", neighbors)
# fill_cell(map_erosion, neighbors[0][0][0], neighbors[0][0][1], cell_x, cell_y, 210)

# Crear una ventana de visualización
cv2.namedWindow('MAP IN REAL TIME', cv2.WINDOW_NORMAL)

return_points = []
visited_cells = [] # Lista para guardar celdillas visitadas y ruta para limpiar

current_cell = start_cell
vuelta = 0
# Seguir avanzando mientras hayan celdillas sin limpiar
while len(free_cells) != 0:
    
    fill_cell(map_erosion, current_cell[0], current_cell[1], cell_x, cell_y, color_visited)
    neighbors = get_neighbors_cells(free_cells, current_cell, cell_x, cell_y)
    last_cell = current_cell
    # El primer vecino que haya es el movimiento que se va a realizar, el resto son
    # considerados puntos de retorno. Si no hay vecinos nos encontramos en un punto critico
    # y se procede a buscar el punto de retorno mas cercano.
    
    if len(neighbors) != 0:
        
        # fill_cell(map_erosion, neighbors[0][0][0], neighbors[0][0][1], cell_x, cell_y, color_retrpnts)
        next_cell = neighbors[0][0]
        # Eliminar la celda en la que estamos de las celdas libres y añadirla al camino
        free_cells.remove(current_cell)
        visited_cells.append(current_cell)
        
        # Si la celda actual en la que estoy era un punto de retorno, se actualiza la lista de puntos de retorno
        if current_cell in return_points:
            return_points.remove(current_cell)

        # Guardamos los vecinos como posibles puntos de retorno
        for cell_ngh in neighbors:
            if cell_ngh[0] not in return_points and cell_ngh[0] in free_cells:
                # if neighbors.index(cell_ngh) == 0:
                #     current_cell = cell_ngh[0]
                return_points.append(cell_ngh[0])
                print("VECINO DE", current_cell, ":", cell_ngh)
                fill_cell(map_erosion, cell_ngh[0][0], cell_ngh[0][1], cell_x, cell_y, color_retrpnts)
        
        # Coger la siguiente celdilla a la que ir
        current_cell = next_cell
    else:
        vuelta += 1
        # Mientras haya puntos de retorno seguir recorriendo
        # if len(return_points) == 0:
        #     cv2.imshow('MAP IN REAL TIME', map_erosion)
        #     break
        # else:
        # Aqui iria el algoritmo para la busqueda del punto mas cercano
        if current_cell == return_points[-1]:
            return_points.remove(current_cell)
        
        if len(return_points) == 0:
            cv2.imshow('MAP IN REAL TIME', map_erosion)
            break
        else:
            current_cell = return_points[-1]
    
        print("Numero de puntos criticos: ", vuelta, " - Puntos de Retorno: ", return_points)
        
        # if vuelta > 21:
        #     fill_cell(map_erosion, current_cell[0], current_cell[1], cell_x, cell_y, 60)
        #     cv2.imshow('MAP IN REAL TIME', map_erosion)
        #     time.sleep(2)
        # cv2.imshow('MAP IN REAL TIME', map_erosion)
        # Se procederia a buscar un punto de retorno donde volver a empezar
    print("")
    cv2.imshow('MAP IN REAL TIME', map_erosion)
    cv2.waitKey(10) # 80 buen tiempo
    


'''
directions = ["North", "East", "South", "West"]
return_points = util.PriorityQueue() # Cola para guardar celdillas
pathtocell = util.PriorityQueue() # Cola para guardar los caminos a los nodos
visited_cells = [] # Lista para guardar celdillas visitadas
neighbors = [] # get_neighbors_cells(map_erosion, free_cells, start_cell, cell_x, cell_y) # Vecinos del nodo
path = []
cost = []                           #   Coste del camino a la celdilla

# Si no hay ninguna direccion, es posible que sea un punto critico o que ya ha terminado de limpiar.        
return_points.push(start_cell, 0)
while not return_points.isEmpty():
        current_cell = return_points.pop()
        
        if current_cell not in visited_cells:
            # if problem.isGoalState(current_cell):
            #     break
                
            visited_cells.append(current_cell)
            neighbors = get_neighbors_cells(map_erosion, free_cells, start_cell, cell_x, cell_y)

            for child in neighbors:
                #   Guardo las acciones y el coste para llegar al hijo
                temp_pth = path + [child[1]]
                # cost = problem.getCostOfActions(path) + neighbors[2]

                #   Guardo el camino al hijo
                pathtocell.push(temp_pth, cost)
                return_points.push(child[0], cost)

        path = pathtocell.pop()
'''

# cv2.imshow('ORIGINAL MAP', map_img)

cv2.waitKey(0) # La imagen se sigue mostrando segun el parametro que se pase, si es 0 se muestra de forma infinita

