
'''from __future__ import print_function
import cv2 as cv
import argparse

def detectAndDisplay(frame):
    frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    frame_gray = cv.equalizeHist(frame_gray)
    
    #-- Detect faces
    faces = face_cascade.detectMultiScale(frame_gray)
    for (x,y,w,h) in faces:
        center = (x + w//2, y + h//2)
        frame = cv.ellipse(frame, center, (w//2, h//2), 0, 0, 360, (255, 0, 255), 4)
        
    cv.imshow('Capture - Face detection', frame)

# Argumentos que se introducen por la entrada del terminal con --face_cascade, --eyes_cascade y --camera-
parser = argparse.ArgumentParser(description='Code for Cascade Classifier tutorial.')
parser.add_argument('--face_cascade', help='Path to face cascade.', default='haarcascade_frontalface_alt.xml')
parser.add_argument('--eyes_cascade', help='Path to eyes cascade.', default='haarcascade_eye_tree_eyeglasses.xml')
parser.add_argument('--camera', help='Camera divide number.', type=int, default=0)
args = parser.parse_args()

face_cascade_name = args.face_cascade
eyes_cascade_name = args.eyes_cascade

face_cascade = cv.CascadeClassifier()
eyes_cascade = cv.CascadeClassifier()

#-- 1. Load the cascades
if not face_cascade.load(cv.samples.findFile(face_cascade_name)):
    print('--(!)Error loading face cascade')
    exit(0)
if not eyes_cascade.load(cv.samples.findFile(eyes_cascade_name)):
    print('--(!)Error loading eyes cascade')
    exit(0)

camera_device = args.camera
#-- 2. Read the video stream
cap = cv.VideoCapture(camera_device)

if not cap.isOpened:
    print('--(!)Error opening video capture')
    exit(0)
    
while True:
    ret, frame = cap.read()
    if frame is None:
        print('--(!) No captured frame -- Break!')
        break

    detectAndDisplay(frame)

    if cv.waitKey(10) == 27:
        break
    
'''
import numpy as np
import cv2
import time

def rotate_img(img, angle):
    # Rotar imagen cada 45º hasta encontrar cara
    (h, w) = img.shape[:2]
    center = (w / 2, h / 2)
    scale = 1

    M = cv2.getRotationMatrix2D(center, angle, scale)
    rotated = cv2.warpAffine(img, M, (w, h))
    
    return rotated

def detect_body(img_gray):
    
    counter = 0
    (h, w) = img_gray.shape[:2]
    
    for j in range(h):
        for i in range(w):
            if img_gray[j][i] == 0:
                counter += 1
                if counter >= 20:
                    return True
            
    return False

def detect_face(img_gray, detector, a):
    angle = 0
    a[0] = 1
    results = []
    while angle <= 360:

        img_gray_rotated = rotate_img(img_gray, angle)
        results = detector.detectMultiScale(img_gray_rotated)

        if len(results) != 0:
            for (x,y,w,h) in results:
                cv2.rectangle(img_gray_rotated, (x,y), (x+w,y+h), 255, 2)
                cv2.imshow('Face',img_gray_rotated)
                cv2.waitKey(0)
            return True

        angle += 45
    
    return False

#---loading the Haar Cascade detector using CascadeClassifier---
face_detector=cv2.CascadeClassifier('haarcascade_frontalface_alt.xml')
#---Loading the image from local -----

start = time.time()
img = cv2.imread('Imgs_pruebas/cara_4.png')
h, w = img.shape[:2]

center_x, center_y = w // 2, h//2
mid_h, mid_w = int(h // 3.5), int(w // 3.5)

start_y = center_y - mid_h
end_y = center_y + mid_h
start_x = center_x - mid_w
end_x = center_x + mid_w

recorted_img = img[start_y:end_y, start_x:end_x]
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


if detect_body(img_gray):
    print("CUERPO DETECTADO")

angle = 0

# import cv2
# import numpy as np

# # Leer la imagen con OpenCV
# imagen_opencv = cv2.imread('Imgs_pruebas/cara_1.png')

# # Obtener la altura y el ancho de la imagen
# alto, ancho = imagen_opencv.shape[:2]

# # Definir las coordenadas del área central que quieres conservar
# centro_y, centro_x = alto // 2, ancho // 2
# mitad_alto, mitad_ancho = alto // 4, ancho // 4  # Por ejemplo, si quieres quedarte con la mitad de la altura y el ancho

# # Calcular las coordenadas del rectángulo para recortar
# inicio_y = centro_y - mitad_alto
# fin_y = centro_y + mitad_alto
# inicio_x = centro_x - mitad_ancho
# fin_x = centro_x + mitad_ancho

# # Recortar la imagen
# imagen_recortada = imagen_opencv[inicio_y:fin_y, inicio_x:fin_x]

# # Mostrar la imagen original y la imagen recortada
# cv2.imshow('Imagen Original', imagen_opencv)
# cv2.imshow('Imagen Recortada', imagen_recortada)
# cv2.waitKey(0)
# cv2.destroyAllWindows()



# while angle <= 360:
#     # angle = input("Angulo de rotacion: ")
#     # angle = int(angle)
#     # scale = 1

#     # M = cv2.getRotationMatrix2D(center, angle, scale)
#     # img_rotated = cv2.warpAffine(img, M, (w, h))

#     img_gray_rotated = rotate_img(img_gray, angle)

#     # img_gray = cv2.cvtColor(img_rotated, cv2.COLOR_BGR2GRAY)
#     # img_gray = cv2.equalizeHist(img_gray)

#     # cv2.imshow('img',img_gray_rotated)
#     # cv2.waitKey(1000)

#     results = face_detector.detectMultiScale(img_gray_rotated)
#     print(results)

#     for (x,y,w,h) in results:
#         cv2.rectangle(img_gray_rotated, (x,y), (x+w,y+h), (0,255,0), 2)
#         cv2.imshow('img',img_gray_rotated)
#         print(round(time.time()-start, 4))
#         cv2.waitKey(0)
    
#     cv2.destroyAllWindows()
#     angle += 45
b = [0]
cv2.imshow('img',img_gray)
cv2.waitKey(0)
if detect_face(img_gray, face_detector, b):
    print("Detecto Cara")

# resultsContiene # las coordenadas de los cuadros delimitadores de la imagen.
# detectMultiScale # Este método sólo funciona con imágenes en escala de grises.
# cv2.rectangle # Este método OpenCV nos permite dibujar rectángulos después de pasar las coordenadas.
# scaleFactor = 1.3 # Parámetro de ajuste fino de 1 a 2.
