import smbus
import time
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import busio
import board
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
from cv2 import aruco
import numpy as np
import math
import array

bus = smbus.SMBus(1)

lcd_columns = 16
lcd_rows = 2

picture = 0

# Initialise I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)

# Initialise the LCD class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

# This is the address we setup in the Arduino Program
address = 0x04

angle = 12.56
value = [int(ele) for ele in str(angle) if ele.isdigit()]


#Writes value to the wire so the aruino can get the value
def writeSetpoint(value):
    bus.write_i2c_block_data(address, 0, value)
    return 1

#Reads the value on the wire
def readPosition():
    pos = bus.read_i2c_block_data(address,0,5)
    return pos

#Prints desired and actual value to the LCD screen 
def printLCD():
    rec = readPosition()
    str1 = ''.join(chr(e) for e in rec)
    print(rec)
    print(str1)
    lcd.message = ("Desired: %s\n Actual: %s" % (angle, str1[0:4]))
    
print(value)
writeSetpoint(value)
printLCD()

def checkType(a_list):
    for element in a_list:
        if isinstance(element, int):
            print("It's an Integer")
        if isinstance(element, str):
            print("It's an string")
        if isinstance(element, float):
            print("It's an floating number")

checkType(value)
