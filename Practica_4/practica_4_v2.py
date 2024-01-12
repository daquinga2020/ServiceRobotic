from GUI import GUI
from HAL import HAL
from ompl import base as ob
from ompl import geometric as og
import cv2 as cv
import numpy as np
import time
import math

class PathPlanner:
    def __init__(self, Warehouse_Map, size_w, size_h):
        self.Warehouse_Map = Warehouse_Map
        self.size_rob = size_w, size_h
        self.dimensions_warehouse_map = [0, 0, Warehouse_Map.w_map, Warehouse_Map.w_map]
        # State Space SE2, a subset of R^2
        self.space = ob.SE2StateSpace()
        self.configure_space()
        
        # Instance of spatial information for this state space.
        self.si = ob.SpaceInformation(self.space)
        # Set the state validity check for this space
        self.si.setStateValidityChecker(ob.StateValidityCheckerFn(self.is_state_valid))
        # Problem instance
        self.pdef = ob.ProblemDefinition(self.si)
        

    def set_PositionRobot(self, coord_robot):
        self.robot_pose = coord_robot
        
    def set_PositionGoal(self, coord_goal):
        self.goal_pose = coord_goal

    def configure_space(self):
        # Set lower and upper limits of the state space
        bounds = ob.RealVectorBounds(2)
        bounds.setLow(0, self.dimensions_warehouse_map[0])
        bounds.setLow(1, self.dimensions_warehouse_map[1])
        bounds.setHigh(0, self.dimensions_warehouse_map[2])
        bounds.setHigh(1, self.dimensions_warehouse_map[3])
        self.space.setBounds(bounds)

    def configure_problem_definition(self):
        # Set the initial and target state of our robot
        start = ob.State(self.space)
        start().setX(self.robot_pose[0])
        start().setY(self.robot_pose[1])
        start().setYaw(self.robot_pose[2])
        goal = ob.State(self.space)
        goal().setX(self.goal_pose[0])
        goal().setY(self.goal_pose[1])
        goal().setYaw(self.goal_pose[2])
        
        # Establish start and goal states
        self.pdef.setStartAndGoalStates(start, goal)
        # Optimizer to try to find the shortest route
        self.pdef.setOptimizationObjective(self.get_path_length_objective())

    def configure_planner(self, planner):
        self.planner = planner
        self.planner.setRange(15)  # Distance between state and state
        # Establishing the problem we are trying to solve for the planner
        self.planner.setProblemDefinition(self.pdef)
        # Perform the planner configuration steps
        self.planner.setup()

    def get_circle_coordinates(self, center, radius, image_shape):
        x_center, y_center = center
        circle_coordinates = []

        for x in range(max(0, int(x_center - radius)), min(image_shape[1], int(x_center + radius))):
            for y in range(max(0, int(y_center - radius)), min(image_shape[0], int(y_center + radius))):
                if (x - x_center)**2 + (y - y_center)**2 <= radius**2:
                    circle_coordinates.append((x, y))

        return np.array(circle_coordinates)

    def get_ellipse_coordinates(self, center, x_radio, y_radio):
        coordenadas_elipse = []
        for y in range(center[1] - y_radio, center[1] + y_radio):
            for x in range(center[0] - x_radio, center[0] + x_radio):
                condition = ((x - center[0]) / x_radio) ** 2 + ((y - center[1]) / y_radio) ** 2 <= 1
                if condition:
                    coordenadas_elipse.append((x, y))

        return np.array(coordenadas_elipse)

    def has_black_pixel_in_radius(self, image, center, x_radio, y_radio):
        ellipse_coordinates = self.get_ellipse_coordinates(center, x_radio, y_radio)
        for coord in ellipse_coordinates:
            x_coord, y_coord = coord
            if image[y_coord][x_coord] < 250:
                return True
        return False

    def is_state_valid(self, state):
        x = min(int(state.getX()), self.Warehouse_Map.w_map-1)
        y = min(int(state.getY()), self.Warehouse_Map.h_map-1)
        value = self.Warehouse_Map.map_img[y][x]
        return value > 250 and not self.has_black_pixel_in_radius(self.Warehouse_Map.map_img, (x, y), self.size_rob[0], self.size_rob[1]) # 4

    def get_path_length_objective(self):
        return ob.PathLengthOptimizationObjective(self.si)

    def plan(self, timeout=1.0):
        
        self.configure_problem_definition()
        
        self.configure_planner(og.SORRTstar(self.si))
        
        solved = self.planner.solve(timeout)
        if solved:
            sol_path = self.pdef.getSolutionPath()
            path = self.create_numpy_path(sol_path.printAsMatrix())
            return path
        return None
    
    def reset_planner(self):
        self.planner.clear()

    def create_numpy_path(self, states):
        lines = states.splitlines()
        
        length = len(lines) - 1
        
        array = np.zeros((length, 2))

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
        self.ind_coordinates_obs = 0
        self.coordinates_obs = []
        self.coordinates_obs_values = []
    
    def get_ellipse_coordinates(self, center, x_radio, y_radio):

        coordenadas_elipse = []
        for y in range(center[1] - y_radio, center[1] + y_radio):
            for x in range(center[0] - x_radio, center[0] + x_radio):
                condition = ((x - center[0]) / x_radio) ** 2 + ((y - center[1]) / y_radio) ** 2 <= 1
                if condition:
                    coordenadas_elipse.append((x, y))

        return np.array(coordenadas_elipse)
    
    def save_obs_map(self, center, x_radio, y_radio):
        self.coordinates_obs.append(self.get_ellipse_coordinates(center, x_radio, y_radio))
        values = []
        for c in self.coordinates_obs[self.ind_coordinates_obs]:
            values.append(self.map_img[c[1], c[0]])
            self.map_img[c[1], c[0]] = 255
            
        self.coordinates_obs_values.append(values)
        self.ind_coordinates_obs += 1
        return self.ind_coordinates_obs-1
        
    def update_obs_map(self, ind_obs, coordinates_destination):
        ind = 0
        for c_dest in coordinates_destination:
            self.map_img[c_dest[1], c_dest[0]] = self.coordinates_obs_values[ind_obs][ind]
            ind += 1

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


time.sleep(6)
CENTER_GZ_ON_MAP = (209, 140)

map_img = cv.imread("RoboticsAcademy/exercises/static/exercises/amazon_warehouse_newmanager/resources/images/map.png", cv.IMREAD_GRAYSCALE)
warehouse_w = 20.62
warehouse_h = 13.6
WH_Map = WarehouseMap((warehouse_w, warehouse_h), map_img)


start_pxl_rob = convert_gz2pxl([(HAL.getPose3d().x,HAL.getPose3d().y)], (WH_Map.scale_x, WH_Map.scale_y))

# The shelves coordinates to pixel coordinates
shelves = [(3.828, 0.779), (3.728, -1.242), (3.728, -3.039), (3.728, -4.827), (3.728, -6.781), (3.728, -8.665)]
shelves_map = convert_gz2pxl(shelves, (WH_Map.scale_x, WH_Map.scale_y))
end_point = convert_gz2pxl([(-1.379, -0.155)], (WH_Map.scale_x, WH_Map.scale_y))

# States Machine
CREATE_PATH = 0
FOLLOW_PATH = 1
HANDLE_SHELF = 2

current_state = CREATE_PATH

shelf_above = False
index_goals = 0
local_goal_reached = False
global_goal_reached = False
index_path = 2
aligned = False
ind_obs = 0

Kp = 0.35
Kp_aligned = 0.1

while True:
    # Enter iterative code!
    x_rob, y_rob, theta_rob = HAL.getPose3d().x,HAL.getPose3d().y, HAL.getPose3d().yaw
    
    if current_state == CREATE_PATH:
        print("Creating Path ...")
        if not shelf_above:
            PthPlanner = PathPlanner(WH_Map, 4, 4)
            print("Loading site")
            pxl_rob = convert_gz2pxl([(x_rob, y_rob)], (WH_Map.scale_x, WH_Map.scale_y))
            PthPlanner.set_PositionRobot((pxl_rob[0][0], pxl_rob[0][1], theta_rob))
            PthPlanner.set_PositionGoal((shelves_map[index_goals][0], shelves_map[index_goals][1], np.pi))
            path = PthPlanner.plan(15)
            GUI.showPath(path)
            path = convert_pxl2gz(path, (WH_Map.scale_x, WH_Map.scale_y))
            index_goals += 1
        else:
            PthPlanner = PathPlanner(WH_Map, 17, 40)
            print("Unloading site")
            pxl_rob = convert_gz2pxl([(x_rob, y_rob)], (WH_Map.scale_x, WH_Map.scale_y))
            PthPlanner.set_PositionRobot((pxl_rob[0][0], pxl_rob[0][1], theta_rob))
            PthPlanner.set_PositionGoal((end_point[0][0], end_point[0][1], np.pi))
            path = PthPlanner.plan(15)
            GUI.showPath(path)
            path = convert_pxl2gz(path, (WH_Map.scale_x, WH_Map.scale_y))
            
        time.sleep(2)
        current_state = FOLLOW_PATH
    
    elif current_state == FOLLOW_PATH:
        print("Following path ...")
        
        if local_goal_reached:
            local_goal_reached = not local_goal_reached
            index_path += 1
        
        if index_path >= len(path):
            index_path = 2
            global_goal_reached = not global_goal_reached
            HAL.setW(0)
            HAL.setV(0)
            print("GLOBAL GOAL REACHED")
            time.sleep(2)
        
        if not global_goal_reached:
            local_goal_abs = path[index_path]
            local_goal_relative = absolute2relative(local_goal_abs[0], local_goal_abs[1], x_rob, y_rob, theta_rob)
            print("\tROB:", (round(x_rob, 3), round(y_rob, 3)), "GOAL: ",(round(local_goal_abs[0], 3), round(local_goal_abs[1], 3)))
            distance_goal = math.sqrt(local_goal_relative[0]**2 + local_goal_relative[1]**2)
            
            w = math.atan2(local_goal_relative[1], local_goal_relative[0])*Kp
            print("DIST:", distance_goal, "W:", w)
            
            if distance_goal <= 0.075:
                local_goal_reached = not local_goal_reached
                
            HAL.setV(0.06)
            HAL.setW(w)
        else:
            print("W:", w)
            if not shelf_above:
                if aligned:
                    HAL.setW(0)
                    aligned = not aligned
                    global_goal_reached = not global_goal_reached
                    current_state = HANDLE_SHELF
                else:
                    w = (np.pi - theta_rob)*Kp_aligned
                    if abs(w) < 0.025:
                        w = 0
                        aligned = not aligned

                HAL.setW(w)
                HAL.setV(0)
            else:
                current_state = HANDLE_SHELF
    
    elif current_state == HANDLE_SHELF:
        print("Handling shelf ...")
        
        if shelf_above:
            print("Putting down shelf ...")
            HAL.putdown()
            time.sleep(60*5) # Simulates the completion of the program for a shelf, comment to continue with the rest.
        else:
            print("Lifting shelf ...")
            HAL.lift()
            ind_obs = WH_Map.save_obs_map(shelves_map[index_goals-1], 17, 40) # Clean obs in map
        
        time.sleep(5)
        shelf_above = not shelf_above
        current_state = CREATE_PATH
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    