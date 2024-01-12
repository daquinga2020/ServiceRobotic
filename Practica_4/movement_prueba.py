from GUI import GUI
from HAL import HAL
from ompl import base as ob
from ompl import geometric as og
import cv2 as cv
import numpy as np
import time
import math

class PathPlanner:
    def __init__(self, Warehouse_Map):
        self.Warehouse_Map = Warehouse_Map
        self.dimensions_warehouse_map = [0, 0, Warehouse_Map.w_map, Warehouse_Map.w_map]
        # Espacio de Estados SE2, un subconjunto de R^2
        self.space = ob.SE2StateSpace()
        self.configure_space()
        
        # Instancia de información espacial para este espacio de estado
        self.si = ob.SpaceInformation(self.space)
        # Establecer la comprobación de validez de estado para este espacio
        self.si.setStateValidityChecker(ob.StateValidityCheckerFn(self.is_state_valid))
        # Instancia de problema
        self.pdef = ob.ProblemDefinition(self.si)
        

    def set_PositionRobot(self, coord_robot):
        self.robot_pose = coord_robot
        
    def set_PositionGoal(self, coord_goal):
        self.goal_pose = coord_goal

    def configure_space(self):
        # Establecer los límites inferior y superior del espacio de estado
        bounds = ob.RealVectorBounds(2)
        bounds.setLow(0, self.dimensions_warehouse_map[0])
        bounds.setLow(1, self.dimensions_warehouse_map[1])
        bounds.setHigh(0, self.dimensions_warehouse_map[2])
        bounds.setHigh(1, self.dimensions_warehouse_map[3])
        self.space.setBounds(bounds)

    def configure_problem_definition(self):
        # Establecer el estado inicial y de meta de nuestro robot
        start = ob.State(self.space)
        start().setX(self.robot_pose[0])
        start().setY(self.robot_pose[1])
        start().setYaw(self.robot_pose[2])
        goal = ob.State(self.space)
        goal().setX(self.goal_pose[0])
        goal().setY(self.goal_pose[1])
        goal().setYaw(self.goal_pose[2])
        
        # Establecer los estados de inicio y meta
        self.pdef.setStartAndGoalStates(start, goal)
        # Optimizador para tratar de encontrar el camino mas corto
        self.pdef.setOptimizationObjective(self.get_path_length_objective())

    def configure_planner(self, planner):
        self.planner = planner
        self.planner.setRange(15)  # Distancia entre estado y estado
        # Establecer el problema que estamos tratando de resolver para el planificador
        self.planner.setProblemDefinition(self.pdef)
        # Realizar los pasos de configuración del planificador
        self.planner.setup()

    def is_state_valid(self, state):
        # Verifica si el estado actual está dentro de un obstáculo
        x = min(int(state.getX()), self.Warehouse_Map.w_map-1)
        y = min(int(state.getY()), self.Warehouse_Map.h_map-1)
        value = self.Warehouse_Map.map_img[y][x]
        return value != 0

    def get_path_length_objective(self):
        return ob.PathLengthOptimizationObjective(self.si)

    def plan(self, timeout=1.0):
        
        self.configure_problem_definition()
        # Crear un planificador para el espacio definido
        self.configure_planner(og.SORRTstar(self.si))
        
        solved = self.planner.solve(timeout)
        if solved:
            sol_path = self.pdef.getSolutionPath()
            path = self.create_numpy_path(sol_path.printAsMatrix())
            return path
        return None

    def create_numpy_path(self, states):
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


class WarehouseMap:
    def __init__(self, real_size, map_img):
        self.map_img = map_img
        self.h_map, self.w_map = map_img.shape
        self.real_w, self.real_h = real_size
        self.scale_x = self.real_w/self.w_map
        self.scale_y = self.real_h/self.h_map
        self.process_map()

    def process_map(self):
        for j in range(self.h_map):
            for i in range(self.w_map):
                if self.map_img[j][i] < 250:
                    self.map_img[j][i] = 0
        kernel = np.ones((3,3), np.uint8)
        self.map_img = cv.erode(self.map_img, kernel, iterations=4)

def convert_gz2pxl(coords_gz, scale):
  
    coords_pxl = []
    
    for c in coords_gz:
        pix_x = round(int(-c[1]/scale[0]) + CENTER_GZ_ON_MAP[0])
        pix_y = round(int(-c[0]/scale[1]) + CENTER_GZ_ON_MAP[1])
        coords_pxl.append((pix_x, pix_y))
    
    return coords_pxl

def convert_pxl2gz(coords_pxl, scale):
  
    coords_gz = []
    
    for c in coords_pxl:
        gz_x = round(-(c[1] - CENTER_GZ_ON_MAP[1])*scale[1], 4)
        gz_y = round(-(c[0] - CENTER_GZ_ON_MAP[0])*scale[0], 4)
        coords_gz.append((gz_x, gz_y))
    
    return coords_gz

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


CENTER_GZ_ON_MAP = (209, 140)

time.sleep(7)

map_img = cv.imread("RoboticsAcademy/exercises/static/exercises/amazon_warehouse_newmanager/resources/images/map.png", cv.IMREAD_GRAYSCALE)
warehouse_w = 20.62
warehouse_h = 13.6
WH_Map = WarehouseMap((warehouse_w, warehouse_h), map_img)
PthPlanner = PathPlanner(WH_Map)


start_pxl_rob = convert_gz2pxl([(HAL.getPose3d().x,HAL.getPose3d().y)], (WH_Map.scale_x, WH_Map.scale_y))

# The shelves coordinates to pixel coordinates
shelves = [(3.728, 0.579), (3.728, -1.242), (3.728, -3.039), (3.728, -4.827), (3.728, -6.781), (3.728, -8.665)]
shelves_map = convert_gz2pxl(shelves, (WH_Map.scale_x, WH_Map.scale_y))

# States Machine
CREATE_PATH = 0
FOLLOW_PATH = 1
HANDLE_SHELF = 2

current_state = CREATE_PATH

shelf_above = False
index_goals = 0
local_goal_reached = False
global_goal_reached = False
index_path = 1
aligned = False

Kp = 0.05

path = [(209, 142), (206, 132), (203, 118), (200, 106), (198, 100), (196, 85), (196, 77), (198, 64)]
path_gz = convert_pxl2gz(path, (20.62/415, 13.6/279))

GUI.showPath(path)

while True:
    # Enter iterative code!
    x_rob, y_rob, theta_rob = HAL.getPose3d().x,HAL.getPose3d().y, HAL.getPose3d().yaw
    
    if local_goal_reached:
        HAL.setV(0)
        HAL.setW(0)
        print("LOCAL GOAL REACHED")
        time.sleep(3)
        local_goal_reached = not local_goal_reached
        index_path += 1
        
    if index_path >= len(path):
        index_path = 1
        global_goal_reached = not global_goal_reached
    
    if not global_goal_reached:
        local_goal_abs = path_gz[index_path]
        
        local_goal_relative = absolute2relative(local_goal_abs[0], local_goal_abs[1], x_rob, y_rob, theta_rob)
        
        # print("\tROB:", round(pxl_rob[0][0], 3), round(pxl_rob[0][1], 3), "GOAL: ",round(local_goal_abs[0], 3), round(local_goal_abs[1], 3))
        print("\tROB:", (round(x_rob, 3), round(y_rob, 3)), "GOAL: ",(round(local_goal_abs[0], 3), round(local_goal_abs[1], 3)))
        distance_goal = math.sqrt(local_goal_relative[0]**2 + local_goal_relative[1]**2)
        
        w = math.atan2(local_goal_relative[1], local_goal_relative[0])*Kp
        print("DIST:", distance_goal, "W:", w)
        
        if distance_goal <= 0.075:
            local_goal_reached = not local_goal_reached
            
        HAL.setV(0.02)
        HAL.setW(w)
    else:
        HAL.setV(0)
        HAL.setW(0)
        print("GLOBAL GOAL REACHED")
        time.sleep(3)
        if aligned:
            aligned = not aligned
            current_state = HANDLE_SHELF
        else:
            w = (np.pi - theta_rob)
            if abs(w) < 0.1:
                w = 0
                aligned = 0
        HAL.setW(w*Kp)
        HAL.setV(0)
        
        
        
        
        
        
        
        
        
        
        
        