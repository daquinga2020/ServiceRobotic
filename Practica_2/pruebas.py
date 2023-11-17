'''import matplotlib.pyplot as plt
import numpy as np

def esta_dentro_del_radio(x, y, m, n, radio):
    distancia = np.sqrt((x - m)**2 + (y - n)**2)
    return distancia <= radio

# Coordenadas del centro
x_centro = 24.67
y_centro = -33.96

# Punto a comprobar
m_punto = 23.36
n_punto = -35.22


# Radio
radio = 1.5

# Comprobar si el punto está dentro del radio
esta_dentro = esta_dentro_del_radio(x_centro, y_centro, m_punto, n_punto, radio)

# Dibujar el círculo y el punto en el gráfico
theta = np.linspace(0, 2*np.pi, 100)
x_circulo = x_centro + radio * np.cos(theta)
y_circulo = y_centro + radio * np.sin(theta)

plt.figure(figsize=(8, 8))
plt.plot(x_circulo, y_circulo, label='Círculo de radio 1.25')
plt.scatter([x_centro, m_punto], [y_centro, n_punto], color=['red', 'blue'], label='Centro y Punto')

# Conectar el centro y el punto con una línea
plt.plot([x_centro, m_punto], [y_centro, n_punto], linestyle='--', color='gray')

# Marcar si el punto está dentro o fuera del círculo
if esta_dentro:
    plt.text(m_punto, n_punto + 0.1, 'Está dentro', color='blue')
else:
    plt.text(m_punto, n_punto + 0.1, 'Está fuera', color='red')

plt.title('Círculo y Punto')
plt.xlabel('Coordenada X')
plt.ylabel('Coordenada Y')
plt.legend()
plt.grid(True)
plt.show()
'''

import matplotlib.pyplot as plt
import numpy as np

def esta_dentro_del_radio(x, y, m, n, radio):
    distancia = np.sqrt((x - m)**2 + (y - n)**2)
    return distancia <= radio

# Puntos y radios
puntos = [(38.39, -30.09), (39.58, -33.06), (24.38, -33.98), (35.45, -34.88), (41.0, -38.99), (32.04, -39.87)]
radios = [2.5] * len(puntos)

# Dibujar los puntos y círculos en el gráfico
plt.figure(figsize=(10, 10))

for punto, radio in zip(puntos, radios):
    x, y = punto
    theta = np.linspace(0, 2*np.pi, 100)
    x_circulo = x + radio * np.cos(theta)
    y_circulo = y + radio * np.sin(theta)

    plt.plot(x_circulo, y_circulo, label=f'Radio = {radio}')
    plt.scatter(x, y, label=f'Centro ({x}, {y})')

plt.title('Posicion de las personas con su espacio restringido')
plt.xlabel('Coordenada X')
plt.ylabel('Coordenada Y')
plt.legend()
plt.grid(True)
plt.show()
