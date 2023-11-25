import math

def clean_data_laser(data_laser):
    
    values = data_laser.values # tuple
    clean_data =  []
    
    for i in range(181):
        if not math.isnan(values[i]) and not math.isinf(values[i]):
            clean_data.append((values[i], i))
    
    return clean_data # [(value, angle), ...]

'''import numpy as np
import matplotlib.pyplot as plt

# Genera 50 puntos con coordenadas x e y aleatorias
points_x = np.random(0, 10, 50)
points_y = np.random.uniform(0, 0, 50)

# Ordena los puntos por coordenada x
sorted_indices = np.argsort(points_x)
points_x = points_x[sorted_indices]
points_y = points_y[sorted_indices]

# Calcula el ángulo formado por los puntos
angle_rad = np.arctan2(np.diff(points_y), np.diff(points_x))
angle_deg = np.degrees(angle_rad)

# Calcula el ángulo total formado por el conjunto de puntos
total_angle_rad = np.sum(angle_rad)
total_angle_deg = np.degrees(total_angle_rad)

# Imprime el ángulo total
print(f"Ángulo total formado por el conjunto de puntos: {total_angle_deg} grados")

# Grafica los puntos y muestra el ángulo
plt.scatter(points_x[:-1], angle_deg)
plt.xlabel('Coordenada x')
plt.ylabel('Ángulo (grados)')
plt.title('Ángulo formado por puntos dispersos')
plt.show()'''

'''import numpy as np
import matplotlib.pyplot as plt

# Genera 50 puntos aleatorios en el plano bidimensional
points = np.random.rand(50, 2)

# Calcula el ángulo formado por los puntos
vector_a = points[1] - points[0]
vector_b = points[2] - points[0]

# Usa la fórmula del producto punto para obtener el ángulo en radianes
dot_product = np.dot(vector_a, vector_b)
magnitude_a = np.linalg.norm(vector_a)
magnitude_b = np.linalg.norm(vector_b)
angle_radians = np.arccos(dot_product / (magnitude_a * magnitude_b))

# Convierte el ángulo a grados
angle_degrees = np.degrees(angle_radians)

# Grafica los puntos en 2D
plt.scatter(points[:, 0], points[:, 1], marker='o')

# Muestra el ángulo en el título
plt.title(f'Ángulo entre vectores: {angle_degrees:.2f} grados')

# Muestra el gráfico
plt.show()'''

'''import numpy as np
import matplotlib.pyplot as plt

# Genera 50 puntos aleatorios en el plano bidimensional siguiendo una regresión lineal ascendente
x = np.random.rand(50)
y = 2 * x + 1 + 0.1 * np.random.randn(50)  # Ecuación de una línea con ruido

# Calcula el ángulo formado por los puntos
vector_a = np.array([x[1] - x[0], y[1] - y[0]])
vector_b = np.array([x[2] - x[0], y[2] - y[0]])

# Usa la fórmula del producto punto para obtener el ángulo en radianes
dot_product = np.dot(vector_a, vector_b)
magnitude_a = np.linalg.norm(vector_a)
magnitude_b = np.linalg.norm(vector_b)
angle_radians = np.arccos(dot_product / (magnitude_a * magnitude_b))

# Convierte el ángulo a grados
angle_degrees = np.degrees(angle_radians)

# Grafica los puntos y la regresión lineal
plt.scatter(x, y, marker='o', label='Puntos')
plt.plot(x, 2 * x + 1, color='red', label='Regresión Lineal')

# Muestra el ángulo en el título
plt.title(f'Ángulo entre la regresión lineal y los puntos: {angle_degrees:.2f} grados')

# Muestra la leyenda y el gráfico
plt.legend()
plt.show()'''

import numpy as np
import matplotlib.pyplot as plt

# Datos proporcionados
# data = np.array([[2, 1], [2.5, 0], [1, 0], [5, 2], [-1, -1], [-3, -3]])
data = np.array([[1.732, 1], [3.464, 2.5], [5.196, 3], [6.928, 4], [8.660, 5], [13.856, 8]])

# Extrae las coordenadas x e y de los datos
x = data[:, 0]
y = data[:, 1]

# Encuentra la recta que mejor se ajusta utilizando el método de mínimos cuadrados
A = np.vstack([y, np.ones(len(y))]).T
m, c = np.linalg.lstsq(A, x, rcond=None)[0]

# Calcula el ángulo que forma la recta con el eje Y
angle_radians = np.arctan(m)
angle_degrees = np.degrees(angle_radians)

# Grafica los puntos y la recta de ajuste
plt.scatter(x, y, marker='o', label='Puntos')
plt.plot(x, m*x + c, color='red', label='Recta de ajuste')

# Muestra el ángulo en el título
plt.title(f'Ángulo con el eje Y: {angle_degrees:.2f} grados')

# Muestra la leyenda y el gráfico
plt.legend()
plt.show()





