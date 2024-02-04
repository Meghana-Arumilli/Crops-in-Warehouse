# Raspberry Pi Adjustable Air Quality Detector Controlled via GUI
#
# Raspberry Pi 3B+
# 
# By Kutluhan Aktar
#
# Learn how to develop a GUI, named Air Quality Module, to control a mini pan-tilt kit and get information from an MQ-135 Air Quality Sensor.
# Also, you can change the background light (RGB) via the GUI.
# 
# Get more information on the project page:
# https://theamplituhedron.com/projects/Raspberry-Pi-Adjustable-Air-Quality-Detector-Controlled-via-GUI/



from time import sleep
from subprocess import call 
import RPi.GPIO as GPIO
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn


# Create the SPI bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

# Create the cs (chip select)
cs = digitalio.DigitalInOut(board.D5)

# Create the mcp object
mcp = MCP.MCP3008(spi, cs)

# Create analog inputs connected to the input pins on the MCP3008.
channel_0 = AnalogIn(mcp, MCP.P0)

# Define RGB pins settings and PWM frequencies
GPIO.setmode(GPIO.BCM)

def _range(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
    
while True:
    
    # Test your module, then define the value range - in this case between 0 and 60000.
    sensorValue = _range(channel_0.value, 0, 60000, 0, 1023)
    print(sensorValue)
    sleep(0.5)

       
    
