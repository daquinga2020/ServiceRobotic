from ompl import base as ob
from ompl import geometric as og
import math
from math import sqrt
import numpy as np
import matplotlib.pyplot as plt

# Especificar condición de estado válida
def isStateValid(state):
    x = state.getX()
    y = state.getY()
    
    # Verifica si el estado actual está dentro de un obstáculo circular con distancia euclidea
    if sqrt(pow(x - obstacle[0], 2) + pow(y - obstacle[1], 2)) - obstacle[2] <= 0 or sqrt(pow(x - obstacle2[0], 2) + pow(y - obstacle2[1], 2)) - obstacle2[2] <= 0:
        return False # El estado está dentro del obstáculo, por lo tanto, no es válido
    return True # El estado es válido si no está dentro del obstáculo

def getPathLengthObjective(si):
    return ob.PathLengthOptimizationObjective(si)
 
def plan():
    # Construir el espacio de estados del robot en el que estamos planificando.
    # Estamos en [0,1]x[0,1], un subconjunto de R^2.
    # Espacio de Estados SE2.
    # Posicion en este espacio de estados: (x, y, theta) en un plano 2D
    space = ob.SE2StateSpace()

    # Establecer los límites inferior y superior del espacio de estado
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0, dimensions[0])
    bounds.setLow(1, dimensions[1])
    bounds.setHigh(0, dimensions[2])
    bounds.setHigh(1, dimensions[3])
    space.setBounds(bounds)

    # Construir una instancia de información espacial para este espacio de estado
    si = ob.SpaceInformation(space)
    # Establecer la comprobación de validez de estado para este espacio
    si.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))
  
    # Establecer el estado inicial y de meta de nuestro robot
    start = ob.State(space)
    start().setX(0)
    start().setY(0)
    start().setYaw(math.pi / 4)
    goal = ob.State(space)
    goal().setX(20)
    goal().setY(20)
    goal().setYaw(math.pi / 4)

    # Crear una instancia de problema
    pdef = ob.ProblemDefinition(si)

    # Establecer los estados de inicio y meta
    pdef.setStartAndGoalStates(start, goal)
    
    # Create the optimization objective specified by our command-line argument.
    # This helper function is simply a switch statement.
    # Tratar de encontrar el camino mas corto
    pdef.setOptimizationObjective(getPathLengthObjective(si))

    # Crear un planificador para el espacio definido
    planner = og.RRTstar(si)
    planner.setRange(1)
    # Establecer el problema que estamos tratando de resolver para el planificador
    planner.setProblemDefinition(pdef)

    # Realizar los pasos de configuración del planificador
    planner.setup()

    # Resolver el problema e imprimir la solución si existe
    solved = planner.solve(10.0) # Se establece el limite de tiempo en seg para la búsqueda del plan
    if solved:
        print(pdef.getSolutionPath())
        solution_path = pdef.getSolutionPath()
        # interpolated_path = solution_path.interpolate(30)
        # print(interpolated_path)
        plot_path(solution_path, dimensions)

def create_numpy_path(states):
    # Dividir la cadena en líneas
    lines = states.splitlines()
    
    # Obtener la longitud de la matriz
    length = len(lines) - 1
    
    # Crear una matriz NumPy de ceros con forma (length, 2)
    array = np.zeros((length, 2))

    # Recorrer las líneas y llenar la matriz con las coordenadas
    for i in range(length):
        array[i][0] = float(lines[i].split(" ")[0])
        array[i][1] = float(lines[i].split(" ")[1])
    return array

def plot_path(solution_path, dimensions):
    # Obtener la representación de la solución como matriz
    matrix = solution_path.printAsMatrix()
    
    # Crear una matriz de NumPy a partir de la representación de la solución
    path = create_numpy_path(matrix)
    x, y = path.T
    
    # Configurar el gráfico
    ax = plt.gca()
    # Dibujar la trayectoria en rojo con líneas discontinuas
    ax.plot(x, y, 'r--')
    
    # Marcar los puntos de la trayectoria con círculos verdes
    ax.plot(x, y, 'go') 
    
    # Establecer los límites del gráfico según las dimensiones proporcionadas
    ax.axis(xmin=dimensions[0], xmax=dimensions[2], ymin=dimensions[1], ymax=dimensions[3])
    
    # Agregar un círculo que representa el obstáculo al gráfico
    ax.add_patch(plt.Circle((obstacle[0], obstacle[1]), radius=obstacle[2]))
    # Agregar un círculo que representa el obstáculo al gráfico
    ax.add_patch(plt.Circle((obstacle2[0], obstacle2[1]), radius=obstacle2[2]))

    plt.show()

if __name__ == "__main__":
  dimensions = [0, 0, 20, 20]
  obstacle = [5.5, 5.5, 1]   # [x, y, radius]
  obstacle2 = [3.5, 3.5, 2]
  plan()