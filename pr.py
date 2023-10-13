import queue
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

def find_path(punto_inicial, punto_final, celdas_libres, size_cell_x, size_cell_y):
    def get_neighbors(current_point):
        neighbors = []
        x, y = current_point

        # Define las cuatro posibles direcciones: Norte, Sur, Este y Oeste
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]

        for dx, dy in directions:
            new_x, new_y = x + dx, y + dy
            new_point = (new_x, new_y)

            # Verifica si la nueva posición está dentro de los límites y es una celda libre
            if (0 <= new_x < size_cell_x and 0 <= new_y < size_cell_y and new_point in celdas_libres):
                neighbors.append(new_point)

        return neighbors

    frontier = queue.PriorityQueue()
    frontier.put((0, punto_inicial))  # Inicializa la cola de prioridad con el punto inicial y costo cero
    came_from = {}  # Diccionario para almacenar cómo llegamos a cada punto
    cost_so_far = {punto_inicial: 0}  # Diccionario para almacenar el costo acumulado hasta cada punto

    while not frontier.empty():
        _, current_point = frontier.get()

        if current_point == punto_final:
            # Reconstruye el camino desde el punto final hasta el punto inicial
            path = []
            while current_point != punto_inicial:
                path.append(current_point)
                current_point = came_from[current_point]
            path.append(punto_inicial)
            path.reverse()
            return path
        print("ACTUAL:", current_point)
        for neighbor in get_neighbors(current_point):
            print("VECINOS:", neighbor)
            new_cost = cost_so_far[current_point] + 1  # Costo uniforme: 1 por movimiento

            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost
                frontier.put((priority, neighbor))
                came_from[neighbor] = current_point

    return None  # No se encontró un camino válido

# Ejemplo de uso:
punto_inicial = (54, 522)
punto_final = (288, 468)
size_cell_x = 18
size_cell_y = 18

cv2.namedWindow('MAP IN REAL TIME', cv2.WINDOW_NORMAL)

map_img = cv2.imread('mapgrannyannie.png', 0) # Leer la imagen
h, w = map_img.shape
# IMAGEN ALTURA: 1012 ANCHO 1013; img[fila][columna]

# First step: apply erosion function on the map image to dilate obstacles on the map

# Se aplica erosion, una operacion morfologica, a la imagen del mapa y asi dilatar los obstaculos
kernel = np.ones((5,5), np.uint8)

# The first parameter is the original image,
# kernel is the matrix with which image is
# convolved and third parameter is the number
# of iterations, which will determine how much
# you want to erode/dilate a given image.
map_erosion = cv2.erode(map_img, kernel, iterations=3)

# Utilizar celdillas de 32x32 o 36x36, con 28 parece rellenar bastantes sitios
cell_x = 18
cell_y = 18

draw_grid(map_erosion, h, w, cell_x, cell_y)

# Guardar las celdillas libres fuera de obstaculos
# Luego ubicar a nuestro robot en cual de esas celdillas esta.
celdas_libres = save_free_cells(map_erosion, h, w, cell_x, cell_y)

path = find_path(punto_inicial, punto_final, celdas_libres, size_cell_x, size_cell_y)
if path:
    print("Camino encontrado:", path)
    for i in path:
        fill_cell(map_erosion, i[0], i[1], cell_x, cell_y, 100)
        cv2.imshow('MAP IN REAL TIME', map_erosion)
        cv2.waitKey(1000)
else:
    print("No se encontró un camino válido.")
    
# 80 buen tiempo
    
cv2.waitKey(0)
