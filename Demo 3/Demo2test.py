#Include imports
import smbus
import time
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import busio
import board
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
from cv2 import aruco
import numpy
import math
import array

# Initialise I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)
#set up display for later use
bus = smbus.SMBus(1)
#Setup LCD screen
lcd_columns = 16
lcd_rows = 2
address = 0x04
picture = 0
#create global varibales for data transfer
global angle
global marker_distance_cm
global counter
counter = 0
angle = 0
marker_distance_cm = 0
global state
state = 0
#angle = 90

#print(value)
# Initialise the LCD class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
#def printLCD():
#    print(angle)

def initCamera():
    # initialize the camera and grab a reference to the raw camera capture
    global camera
    camera = PiCamera()
    global prev_marker_id
    global marker_id
    prev_marker_id = -1
    marker_id = -3
    
#    lcd.message = ("Angle From Camera:\n  %d degrees" % (angle))
def cv_Demo2():
    rawCapture = PiRGBArray(camera)
    camera.iso = 100
    # Wait for the automatic gain control to settle
    time.sleep(2)
    global angle
    global marker_distance_cm
    global prev_marker_id
    global marker_id
    correction = 80
    correction2 = 3200
    counter = 0
    # Now fix the values
    camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = 'off'
    g = camera.awb_gains
    camera.awb_mode = 'off'
    camera.awb_gains = g
    # allow the camera to warmup
    time.sleep(0.1)
    camera.framerate = 32 #32
    aruco_dictionary = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()
    timer = 0
    xwidth = 54*(math.pi/180)
    ywidth = 41*(math.pi/180)
    marker_flag = 1
    # start continuous picture taking
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
         marker_found = False
          
         # convert to grayscale
         image = frame.array
         cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
         cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
         corners, ids, rejectedImgPoints = aruco.detectMarkers(image, aruco_dictionary, parameters=parameters)
         if type(ids) == numpy.ndarray:
            print("Marker found")
            marker_found = True
            output_array = numpy.ndarray(len(ids))
            index = 0
            for i in ids:
                for j in i:
                    #Determine angle of marcker form the center of the camera
                    xcenter = int((corners[0][0][0][0]+corners[0][0][2][0])/2)
                    ycenter = int((corners[0][0][0][1]+corners[0][0][2][1])/2)
                    yheight = abs(int(corners[0][0][0][1]-corners[0][0][2][1]))
                    marker_distance_ft = 490/yheight
                    #print(yheight)
#                    print("Marker Distance in ft: ",end = "")
#                    print(marker_distance_ft)
#                    print("Rounding: ", end = "")
#                    print(round(marker_distance_ft))
                    
                    
#                    print(marker_distance_cm)
#                    marker_distance_cm = [int(ele) for ele in str(marker_distance_cm) if ele.isdigit()]
#                    print("Marker Distacne in m: ", end="")
#                    print(marker_distance_m)
                    imageMiddleX = image.shape[1]/2
                    xdist = (xcenter - image.shape[1]/2)
                    ydist = (ycenter - image.shape[0]/2)

                    xangle = (xdist/image.shape[1]) * xwidth
                    yangle = (ydist/image.shape[0]) * ywidth
                
                    # Calculate the angle from the z-axis to the center point
                    # First calculate distance (in pixels to screen) on z-axis
                    a1 = xdist/math.tan(xangle)
                    a2 = ydist/math.tan(yangle)
                    a = (a1 + a2)/2
                    distance = math.sqrt(pow(xdist,2)+pow(ydist,2))
                    #Calculate final angle for each Aruco
                    angle = math.atan2(distance,a)*(180/math.pi)
#                    print("Correction: ", end = "")
                    marker_distance_ft = marker_distance_ft * 0.96 - (-2.5244*pow(angle * math.pi/180,2) + 0.027)
#                    print(marker_distance_ft)
                    marker_distance_cm = marker_distance_ft*30.48*100
                    marker_distance_cm = int(marker_distance_cm)
                    if(xcenter > imageMiddleX):
                        angle *= -1
                        if(angle <-3.8 and angle >-4.3):
                            angle += 4
                    else:
                        angle += 4
                    if(angle > 0):
                        angle -= 2
                    if(counter == 0):
                        counter += 1
                        angle += 3
                    display_angle = str(angle)
                    #print("Angle from camera:",angle,"degrees")
                    lcd.message = ("Beacon Detected \nAngle: %s" % (display_angle))
                    marker_flag=1
                    output_array[index] = j
#                    print("Marker Indexes:", end=" ")
#                    print(j)
                    marker_id = j
                    index+=1
                    #print()
                    break
         rawCapture.truncate(0)
         if marker_found == False:
             #print("No markers found")
             if marker_flag == 1:
                 lcd.clear()
                 marker_flag = 0
             lcd.message = ("Beacon Not\nDetected")
         else:
             if marker_id != prev_marker_id and abs(marker_distance_ft) < 4.5:
                lcd.clear()
                lcd.message = ("Sending\nangle...")
                prev_marker_id = marker_id
                angle = int((angle + correction)* 100)
                value = [int(ele) for ele in str(angle) if ele.isdigit()]
                time.sleep(0.5)
                writeSetpoint(value)
                time.sleep(3.3) #2.8
                print("angle sent")
                print(readPosition())
                lcd.clear()
                lcd.message = ("Sending\ndistance...")
                time.sleep(1)
                value = [int(ele) for ele in str(marker_distance_cm - correction2) if ele.isdigit()] #2450
                writeSetpoint(value)
                time.sleep((marker_distance_ft + 1) / 2 + 1)
                correction = 25
                correction2 = 5400
                lcd.clear()
                lcd.message = ("Finished\nSend")
                time.sleep(1.5)
                lcd.clear()
                lcd.message = ("Waiting...")
                time.sleep(1.5)
        
             
#Writes value to the wire so the aruino can get the value
def writeSetpoint(value):
    bus.write_i2c_block_data(address, 0, value)
    return 1

#Reads the value on the wire
def readPosition():
    pos = bus.read_i2c_block_data(address,0,1)
    return pos
#Prints desired and actual value to the LCD screen 
def printLCD():
    rec = readPosition()
    str1 = ''.join(chr(e) for e in rec)
    print(rec)
    #print(str1)
    lcd.message = ("Desired: %s\n Actual: %s" % (angle, str1[0:4]))
    
def checkFlag(arduinoState):
    state_flag = state
    while state_flag!=arduinoState:
        state_flag = readPosition()
#        print(state_flag)
        time.sleep(0.5)

initCamera()

time.sleep(0.1)

if(state == 0):
    cv_Demo2()
#    print(angle)
#    print(marker_distance_cm)
    state = 1
if(state == 1):
#    time.sleep(3)
    if counter != 0:
        angle = int((angle + 50)* 100)
    else:
        angle = int((angle + 45)*100)
    value = [int(ele) for ele in str(angle) if ele.isdigit()]
    writeSetpoint(value)
    time.sleep(3.3) #2.8
    print("angle sent")
    print(readPosition())
    time.sleep(1)
    value = [int(ele) for ele in str(marker_distance_cm - 2348) if ele.isdigit()]
    writeSetpoint(value)
#    print(readPosition())
#    time.sleep(2)
#    print(readPosition())
#    print("finished")



