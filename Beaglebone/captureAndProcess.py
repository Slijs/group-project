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

# setup the UART
UART.setup("UART2")
ser = serial.Serial(port = "/dev/ttyO2", baudrate=9600)
ser.close() # close first incase it was left open by some other process
ser.open()  # open to use

# listen for an OS signal to terminate, so that we can close UART properly
def signal_handler(signal, frame):
    print('\nClosing serial and exiting.\n')
    ser.close()
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

# GPIO output for LED
GPIO.setup("P9_14", GPIO.OUT)

# Captures a single image from the camera and returns it in PIL format
def get_image():
    os.system('/home/debian/boneCVLowRes') # capture a frame from webcam
    bigImage = cv2.imread('capture.png')
    # resize the image before returning
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
    # upper body disabled due to taking too long
    #upper_body = upper_body_cascade.detectMultiScale(grey_copy, scaleFactor=1.4, minNeighbors=5)
    # if a face detected, blink LED
    if(len(faces) > 0):
        print("turn on LED")
	GPIO.output("P9_14", GPIO.HIGH)
        time.sleep(2)
	print("turn off LED")
        GPIO.output("P9_14", GPIO.LOW)
    else:
        print("No faces detected.")
    for (x, y, w, h) in faces:
        # get the height and width of the image
	height, width = grey_copy.shape[:2]
        # print the offset of the face detected from the centre of the image
	print_angle(width, x, w)
    return len(faces)

##########################################################################
#                            MAIN START                                  #
##########################################################################

# Load the face and upper body cascades
lbp_face_cascade = cv2.CascadeClassifier('data/lbpcascade_frontalface.xml')

# Loop continuously, waiting for '1' to be received over UART. If it gets
# it, we will check for a face. If we find a face, we will return 1, else 0
while True:
    if ser.isOpen():
        print "Serial is open! Waiting for signal to check for images."
        command = ser.read() # Get command from UART, block until one received
        print("Got serial signal {}".format(command))
        if command == '1':
            print("Getting an image to check for a face.\n")
            test1 = get_image() # get the image from external program, and load it
	    # Do the face detection, and record how many faces detected
            response = detect_faces(lbp_face_cascade, haar_upperbody_cascade, test1)
            if response > 0:
                ser.write('1')
                print("Responded by serial: 1")
            else:
                ser.write('0')
                print("Responded by serial: 0")
