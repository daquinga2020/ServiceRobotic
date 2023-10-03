import numpy as np
import cv2
import time

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
    for i in range(px+1, px + size_cell_x): # se puede borrar el 1 si elimino el grid
        for j in range(py+1, py + size_cell_y):
            if img[i,j] == 0:
                return 1 # Hay obstaculo
    return 0


def save_cells_free(img, hight, width, size_cell_x, size_cell_y):
    
    f_cells = []
    
    for i in range(0, width, size_cell_x):
        for j in range(0, hight, size_cell_y):
            if check_obs(img, i, j, size_cell_x, size_cell_y) != 1:
                f_cells = f_cells + [(i,j)]

    return f_cells


map_img = cv2.imread('mapgrannyannie.png', 0) # Leer la imagen
h, w = map_img.shape
# IMAGEN ALTURA: 1012 ANCHO 1013
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
cell_x = 28
cell_y = 28

draw_grid(map_erosion, h, w, cell_x, cell_y)

# Guardar las celdillas libres fuera de obstaculos
# Luego ubicar a nuestro robot en cual de esas celdillas esta.
free_cells = save_cells_free(map_erosion, h, w, cell_x, cell_y)

print("N:", len(free_cells), " CELDAS LIBRES: ", free_cells)

x, y = free_cells[0]

for i in range(x, x + 1 + cell_x): # se puede borrar el 1 si elimino el grid
    for j in range(y, y + 1 + cell_y):
        map_erosion[i][j] = 200

        
cv2.imshow('EROSIONED MAP', map_erosion)

cv2.waitKey(0) # La imagen se sigue mostrando segun el parametro que se pase, si es 0 se muestra de forma infinita

