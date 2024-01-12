import cv2
import time
import numpy as np

def click_event(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f'Coordenadas del píxel: ({x}, {y})')
        pixel_value = map_img[y, x]
        print(f'Valor del píxel: {pixel_value}')
        

# Función para obtener las coordenadas de la elipse
def obtener_coordenadas_elipse(centro, radio_mayor, radio_menor, num_puntos=100):
    theta = np.linspace(0, 2*np.pi, num_puntos)
    x = centro[0] + radio_mayor * np.cos(theta)
    y = centro[1] + radio_menor * np.sin(theta)
    return np.column_stack((x.astype(int), y.astype(int)))

def obtener_coordenadas_elipse2(centro, radio_mayor, radio_menor):
    # Función para verificar si un punto está dentro de la elipse
    def punto_dentro_elipse(x, y):
        return ((x - centro[0]) / radio_mayor) ** 2 + ((y - centro[1]) / radio_menor) ** 2 <= 1

    coordenadas_elipse = []
    for y in range(centro[1] - radio_menor, centro[1] + radio_menor):
        for x in range(centro[0] - radio_mayor, centro[0] + radio_mayor):
            if punto_dentro_elipse(x, y):
                coordenadas_elipse.append((x, y))

    return np.array(coordenadas_elipse)

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


CENTER_GZ_ON_MAP = (209, 140)
# Size of warehouse 1
warehouse_w = 20.62
warehouse_h = 13.6

map_img = cv2.imread("map.png", cv2.IMREAD_GRAYSCALE)
h, w = map_img.shape
WH_Map = WarehouseMap((warehouse_w, warehouse_h), map_img)
scale = (warehouse_w/w, warehouse_h/h)

estanteria_centro_real = (3.828, 0.779)
estanteria_centro_img = (round(int(-estanteria_centro_real[1]/scale[0]) + CENTER_GZ_ON_MAP[0]), round(int(-estanteria_centro_real[0]/scale[1]) + CENTER_GZ_ON_MAP[1]))
ind = WH_Map.save_obs_map(estanteria_centro_img, 17, 40)
cv2.imshow('MAP', map_img)
cv2.waitKey(0)

end_point_real = (-1.379, -0.155)
end_point_img = (round(int(-end_point_real[1]/scale[0]) + CENTER_GZ_ON_MAP[0]), round(int(-end_point_real[0]/scale[1]) + CENTER_GZ_ON_MAP[1]))
coords_end_point = WH_Map.get_ellipse_coordinates(end_point_img, 17, 40)
WH_Map.update_obs_map(ind, coords_end_point)
coords_end_point = WH_Map.get_ellipse_coordinates(end_point_img, 5, 5)
for c_est in coords_end_point:
    map_img[c_est[1], c_est[0]] = 0
# # (194, 62) (212, 168)
# # print(estanteria_centro_img, end_point_img)

# r_mayor = 40
# r_menor = 17

# # Obtener las coordenadas de la elipse
# coordenadas_estanteria = obtener_coordenadas_elipse2(estanteria_centro_img, r_menor, r_mayor)
# coordenadas_end_point = obtener_coordenadas_elipse2(end_point_img, r_menor, r_mayor)

# for c_est, c_endpnt in zip(coordenadas_estanteria, coordenadas_end_point):
#     pre_save = map_img[c_endpnt[1], c_endpnt[0]]
#     map_img[c_endpnt[1], c_endpnt[0]] = map_img[c_est[1], c_est[0]]
    # map_img[c_est[1], c_est[0]] = pre_save

# Todo lo que sea menor de 250 es un obstaculo
# for j in range(h):
#     for i in range(w):
#         if map_img[j][i] < 250:
#             map_img[j][i] = 0

# kernel = np.ones((3,3), np.uint8)
# map_img = cv2.erode(map_img, kernel, iterations=6)
# 14 pixeles largo de la estanteria. Robot 5
# Crea la ventana y establece la función de devolución de llamada del clic del mouse
cv2.namedWindow('Imagen')
cv2.setMouseCallback('Imagen', click_event)

while True:
    cv2.imshow('Imagen', map_img)

    # Presiona 'esc' para salir
    if cv2.waitKey(1) == 27:
        break

cv2.destroyAllWindows()


# cv.imshow('MAP', map_img)
# cv.waitKey(0)

# 1, 3, 5, 6
# Coordenadas del píxel: (197, 64)
# Valor del píxel: 254
# Coordenadas del píxel: (270, 64)
# Valor del píxel: 254
# Coordenadas del píxel: (345, 66)
# Valor del píxel: 254
# Coordenadas del píxel: (382, 65)
# Valor del píxel: 254
# 
# Centro del mundo en pixeles(208, 146)

# PX: (43, 151) GZ: (-0.491, 8.201)
# PX: (122, 187) GZ: (-2.23, 4.221)
# PX: (352, 192) GZ: (-2.42, -7.18)
# PX: (7, 10) GZ: (-6.55, 9.978)