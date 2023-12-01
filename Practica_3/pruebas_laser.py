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
coordenadas = [
    (2.0152158737182617, 91.0),
    (1.9961011409759521, 92.0),
    (1.9639514684677124, 93.0),
    (1.9774430990219116, 94.0),
    (2.0169520378112793, 95.0),
    (1.9460786581039429, 96.00000050000001),
    (1.945556879043579, 97.0),
    (1.9494003057479858, 98.0),
    (1.986284613609314, 99.0),
    (1.9826714992523193, 100.0),
    (1.9473129510879517, 101.0),
    (1.9371850490570068, 102.0),
    (1.91070556640625, 103.0),
    (1.9285085201263428, 104.0),
    (1.960860013961792, 105.00000000000001),
    (1.9387348890304565, 106.0),
    (1.94031822681427, 107.0),
    (1.951085090637207, 108.0),
    (1.9915918111801147, 109.0),
    (1.9982149600982666, 110.0),
    (2.007309675216675, 111.0),
    (2.024573802947998, 112.0),
    (2.021632432937622, 113.0),
    (2.069480379623413, 114.00000000000001),
    (2.079458475112915, 115.0),
    (2.070833921432495, 116.00000099000001),
    (2.1125237941741943, 117.0),
    (2.1395022869116107, 118.00000000000001),
    (2.1834733486175537, 119.0),
    (2.2338733673095703, 120),
    (2.237081527709961, 121.0),
    (2.2832775115966797, 122.0),
    (2.323514461517334, 123.00000000000001),
    (2.36433744430542, 124.0),
    (2.42838978767395, 125.00000000000001),
    (2.437138795852661, 126.0),
    (2.4647462368011475, 127.00000000000001),
    (2.5219998359680176, 128.0),
    (2.5842244625091553, 129.0),
    (2.578693389892578, 130.0),
    (2.618546724319458, 131.0),
    (2.668168544769287, 132.0),
    (2.744506597518921, 133.0),
    (2.791062355041504, 134.0),
    (2.858689785003662, 135.0),
    (2.880930185317993, 136.0),
    (3.013571262359619, 137.0),
    (3.0504674911499023, 138.0),
    (3.147648334503174, 139.0),
    (3.179889678955078, 140.0),
    (3.2545273363985596, 141.0),
    (3.343215227127075, 142.0),
    (3.386260986328125, 143.0),
    (3.5021743774414062, 144.0),
    (3.6287624835968018, 145.0),
    (3.7615578174591064, 146.0),
    (3.876377820968628, 147.0),
    (3.994255304336548, 148.0),
    (4.025326728820801, 149.0)
]

data = np.array(coordenadas)

# Extrae las coordenadas x e y de los datos
x = data[:, 0]
y = data[:, 1]
print(x)
# Encuentra la recta que mejor se ajusta utilizando el método de mínimos cuadrados
A = np.vstack([y, np.ones(len(y))]).T
m, c = np.linalg.lstsq(A, x, rcond=None)[0]

# Calcula el ángulo que forma la recta con el eje Y
angle_radians = np.arctan(m)
angle_degrees = np.degrees(angle_radians)

# Grafica los puntos y la recta de ajuste
plt.scatter(y, x, marker='o', label='Puntos')
plt.plot(y, m*y + c, color='red', label='Recta de ajuste')

# Muestra el ángulo en el título
plt.title(f'Ángulo con el eje Xautilu: {angle_degrees:.2f} grados')

# Muestra la leyenda y el gráfico
plt.legend()
plt.show()





