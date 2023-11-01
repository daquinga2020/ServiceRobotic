
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

#---loading the Haar Cascade detector using CascadeClassifier---
face_detector=cv2.CascadeClassifier('haarcascade_frontalface_alt.xml')
#---Loading the image from local -----

img = cv2.imread('Team-India-unveils-new-jersey.png')
img = cv2.resize(img, (1400, 800))

img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
img_gray = cv2.equalizeHist(img_gray)
results = face_detector.detectMultiScale(img_gray)
print(results)

for (x,y,w,h) in results:
    cv2.rectangle(img, (x,y), (x+w,y+h), (0,255,0), 2)
    cv2.imshow('img',img)
    cv2.waitKey(0)

cv2.destroyAllWindows()


# resultsContiene # las coordenadas de los cuadros delimitadores de la imagen.
# detectMultiScale # Este método sólo funciona con imágenes en escala de grises.
# cv2.rectangle # Este método OpenCV nos permite dibujar rectángulos después de pasar las coordenadas.
# scaleFactor = 1.3 # Parámetro de ajuste fino de 1 a 2.

