from GUI import GUI
from HAL import HAL
from ompl import base as ob
from ompl import geometric as og
import cv2 as cv
import numpy as np
import time
import math

def get_circle_coordinates(center, radius, image_shape):
    theta = np.linspace(0, 2*np.pi, 100)
    x_circle = center[0] + radius * np.cos(theta)
    y_circle = center[1] + radius * np.sin(theta)

    # Asegurarse de que las coordenadas estén dentro de los límites de la imagen
    x_circle = np.clip(x_circle, 0, image_shape[1]-1).astype(int)
    y_circle = np.clip(y_circle, 0, image_shape[0]-1).astype(int)

    return np.column_stack((x_circle, y_circle))

def has_black_pixel_in_radius(image, center, radius):
    circle_coordinates = get_circle_coordinates(center, radius, image.shape)
    for coord in circle_coordinates:
        x_coord, y_coord = coord
        if image[y_coord, x_coord] == 0:  # Verifica si el píxel es negro
            return True
    return False

# Especificar condición de estado válida
def isStateValid(state):
    x = min(int(state.getX()), w_map)
    y = min(int(state.getY()), h_map)

    value = map_img[y][x]
    # Verifica si el estado actual está dentro de un obstáculo circular con distancia euclidea
    # Medir distancia del robot al obstaculo(pixel negro)
    if value == 0:# and not has_black_pixel_in_radius(map_img, (x, y), 30):
        return False # El estado está dentro del obstáculo, por lo tanto, no es válido
    return True # El estado es válido si no está dentro del obstáculo

def create_numpy_path(states):
    # Dividir la cadena en líneas
    lines = states.splitlines()
    
    # Obtener la longitud de la matriz
    length = len(lines) - 1
    
    # Crear una matriz NumPy de ceros con forma (length, 2)
    array = np.zeros((length, 2))

    # Recorrer las líneas y llenar la matriz con las coordenadas
    for i in range(length):
        array[i][0] = round(float(lines[i].split(" ")[0]))
        array[i][1] = round(float(lines[i].split(" ")[1]))
    return array

def getPathLengthObjective(si):
    return ob.PathLengthOptimizationObjective(si)

def plan(rob_pose_map, goal_pose_map):
  
    # Construir el espacio de estados del robot en el que estamos planificando.
    # Estamos en [0,1]x[0,1], un subconjunto de R^2.
    # Espacio de Estados SE2.
    # Posicion en este espacio de estados: (x, y, theta) en un plano 2D
    space = ob.SE2StateSpace()
    
    # Establecer los límites inferior y superior del espacio de estado
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0, dimensions_warehouse_map[0])
    bounds.setLow(1, dimensions_warehouse_map[1])
    bounds.setHigh(0, dimensions_warehouse_map[2])
    bounds.setHigh(1, dimensions_warehouse_map[3])
    space.setBounds(bounds)
    
    x_rob, y_rob, theta_rob = rob_pose_map
    x_goal, y_goal = goal_pose_map
    theta_goal = np.pi
    # Construir una instancia de información espacial para este espacio de estado
    si = ob.SpaceInformation(space)
    # Establecer la comprobación de validez de estado para este espacio
    si.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))
    
    # Establecer el estado inicial y de meta de nuestro robot
    start = ob.State(space)
    start().setX(x_rob)
    start().setY(y_rob)
    start().setYaw(theta_rob)
    goal = ob.State(space)
    goal().setX(x_goal)
    goal().setY(y_goal)
    goal().setYaw(theta_goal)
    
    # Crear una instancia de problema
    pdef = ob.ProblemDefinition(si)

    # Establecer los estados de inicio y meta
    pdef.setStartAndGoalStates(start, goal)
    
    # Tratar de encontrar el camino mas corto
    # pdef.setOptimizationObjective(getPathLengthObjective(si))

    # Crear un planificador para el espacio definido
    planner = og.SORRTstar(si)
    planner.setRange(15)
    # Establecer el problema que estamos tratando de resolver para el planificador
    planner.setProblemDefinition(pdef)
    
    # Realizar los pasos de configuración del planificador
    planner.setup()

    # Resolver el problema e imprimir la solución si existe
    solved = planner.solve(10.0) # Se establece el limite de tiempo en seg para la búsqueda del plan
    if solved:
        print(pdef.getSolutionPath())
        sol_path = pdef.getSolutionPath()
        path = create_numpy_path(sol_path.printAsMatrix())
        GUI.showPath(path)
        return path
    return None 


time.sleep(3)

# Map Size
map_img = cv.imread("RoboticsAcademy/exercises/static/exercises/amazon_warehouse_newmanager/resources/images/map.png", cv.IMREAD_GRAYSCALE)
h_map, w_map = map_img.shape

for j in range(h_map):
    for i in range(w_map):
        if map_img[j][i] < 250:
            map_img[j][i] = 0

kernel = np.ones((3,3), np.uint8)
map_img = cv.erode(map_img, kernel, iterations=4)

warehouse_w = 20.62
warehouse_h = 13.6

scale_x = warehouse_w/w_map
scale_y = warehouse_h/h_map

# Initial state
x0_rob = HAL.getPose3d().x
y0_rob = HAL.getPose3d().y
yaw0_rob = HAL.getPose3d().yaw

pix_x_start = round(-y0_rob/scale_x + 209)
pix_y_start = round(-x0_rob/scale_y + 140)

# Dimensions of warehouse 1 map
dimensions_warehouse_map = [0, 0, w_map, h_map]

# The shelves coordinates to pixel coordinates
shelves = [(3.728, 0.579), (3.728, -1.242), (3.728, -3.039), (3.728, -4.827), (3.728, -6.781), (3.728, -8.665)]
shelves_map = []

for coord_gz in shelves:
    pix_x = round(-coord_gz[1]/scale_x + 209)
    pix_y = round(-coord_gz[0]/scale_x + 143)
    shelves_map.append((pix_x, pix_y))

path = plan((pix_x_start, pix_y_start, 0), shelves_map[5])


while True:
    # Enter iterative code!
    x_rob, y_rob, theta_rob = HAL.getPose3d().x,HAL.getPose3d().y, HAL.getPose3d().yaw
    
    # Subir plataforma
    # HAL.lift()
    
    # Bajar plataforma
    # HAL.putdown()
    
    # Shows a path on the map
    # GUI.showPath(array) # array is an 2D array 
    
    # Set velocities
    # HAL.setV()
    # HAL.setW()
    # ompl.base._base.SE2StateInternal
    
    # 3 estados: [82, 141], [124.417, 151.287], [62, 137]
    
    # print((round(x_rob, 3), round(y_rob, 3)))
    # print(path)
    # print(theta_rob)
    # time.sleep(2)