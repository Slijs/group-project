import cv2
import numpy as np
import os
import datetime
import time
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.UART as UART
import serial
import signal
import sys


UART.setup("UART2")
ser = serial.Serial(port = "/dev/ttyO2", baudrate=9600)
ser.close()
ser.open()

def signal_handler(signal, frame):
    print('\nClosing serial and exiting.\n')
    ser.close()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
#signal.pause()

currentDT = datetime.datetime.now()
print (str(currentDT))

GPIO.setup("P9_14", GPIO.OUT)
 
# Create a VideoCapture object and read from input file
# If the input is the camera, pass 0 instead of the video file name
#camera = cv2.VideoCapture(0)
#camera.set(cv2.cv.CV_CAP_PROP_FOURCC, cv2.cv.CV_FOURCC('M', 'J', 'P', 'G') );
#camera.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 640); 
#camera.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 480);
 
# Check if camera opened successfully
#if (camera.isOpened()== False): 
#    print("Error opening video stream")

# Captures a single image from the camera and returns it in PIL format
def get_image():
    # read is the easiest way to get a full image out of a VideoCapture object.
    #retval, im = camera.read()
    #return im

    currentDT = datetime.datetime.now()
    print (str(currentDT))
    os.system('/home/debian/workingGetImageNoEdge')
    currentDT = datetime.datetime.now()
    print (str(currentDT))
    bigImage = cv2.imread('capture.png')
    return cv2.resize(bigImage,(960, 640), interpolation = cv2.INTER_AREA)

# print the centre of the object, relative to the centre of the image
def print_angle(image_width, object_x, object_w):
    # get the centre of the object detected
    object_centre = (object_x + (object_w/2))

    # get the offset relative to centre of image
    image_offset = object_centre - (image_width/2)
    print(image_offset)

def detect_faces(face_cascade, upper_body_cascade, colored_img, scaleFactor = 1.22):    
    grey_copy = cv2.cvtColor(colored_img, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(grey_copy, scaleFactor=scaleFactor, minNeighbors=5)
    #upper_body = upper_body_cascade.detectMultiScale(grey_copy, scaleFactor=1.4, minNeighbors=5)
    #lower_body = lower_body_cascade.detectMultiScale(grey_copy, scaleFactor=scaleFactor, minNeighbors=5)
    print(len(faces))
    #print(len(upper_body))
    #print(len(lower_body))

    # if a face detected, blink LED
    if(len(faces) > 0):
        print("turn on LED")
	GPIO.output("P9_14", GPIO.HIGH)
        time.sleep(2)
	print("turn off LED")
        GPIO.output("P9_14", GPIO.LOW)
    else:
        print("No faces detected.\n")

    for (x, y, w, h) in faces:
        #cv2.rectangle(grey_copy, (x, y), (x+w, y+h), (0, 255, 0), 2)
        height, width = grey_copy.shape[:2]
        print_angle(width, x, w)

    return len(faces)

lbp_face_cascade = cv2.CascadeClassifier('data/lbpcascade_frontalface.xml')
haar_upperbody_cascade = cv2.CascadeClassifier('data/haarcascade_upperbody.xml')
#haar_lowerbody_cascade = cv2.CascadeClassifier('data/haarcascade_lowerbody.xml')


#ser.write('1')

while True:
    if ser.isOpen():
        print "Serial is open! Waiting for signal to check for images."
        command = ser.read()
        print("Got serial signal {}".format(command))
        if command == '1':
            print("Getting an image to check for a face.\n")
            test1 = get_image()
            response = detect_faces(lbp_face_cascade, haar_upperbody_cascade, test1)
            if response > 0:
                ser.write('1')
                print("Responded by serial: 1")
            else:
                ser.write('0')
                print("Responded by serial: 0")

#cv2.imshow('Test Image', detect_faces(lbp_face_cascade, test1))
#cv2.waitKey(0)
#print camera.get(cv2.cv.CAP_PROP_FOURCC)



## Read until video is completed
#while(camera.isOpened()):
#  # Capture frame-by-frame
#  ret, frame = camera.read()
#  if ret == True:
# 
#    # Display the resulting frame
#    cv2.imshow('Frame',frame)
# 
#    # Press Q on keyboard to  exit
#    if cv2.waitKey(25) & 0xFF == ord('q'):
#      break
# 
#  # Break the loop
#  else: 
#    break
 
# When everything done, release the video camerature object
#camera.release()
 
# Closes all the frames
#cv2.destroyAllWindows()

currentDT = datetime.datetime.now()
print (str(currentDT))
