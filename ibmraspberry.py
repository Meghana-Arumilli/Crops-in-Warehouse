import RPi.GPIO as GPIO
import time
import sys
import ibmiotf.application
import ibmiotf.device
from time import sleep
from subprocess import call 
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import requests
import Adafruit_DHT
DHT_SENSOR = Adafruit_DHT.DHT22
DHT_PIN = 4
#Provide your IBM Watson Device Credentials
organization = "czwo3b"
deviceType = "iotproject"
deviceId = "1234"
authMethod = "token"
authToken = "hF&TJkkREj07BySWcj"

# Initialize GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.cleanup()
channel=25
# read data using pin 
GPIO.setup(23, GPIO.IN) #PIR
GPIO.setup(24,GPIO.OUT)#buzzer
GPIO.setup(channel,GPIO.IN)#flame
GPIO.setup(16,GPIO.OUT)#fan
GPIO.setup(27,GPIO.OUT)#Light
T=0
H=0
flame=0
# Create the SPI bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

# Create the cs (chip select)
cs = digitalio.DigitalInOut(board.D5)

# Create the mcp object
mcp = MCP.MCP3008(spi, cs)

# Create analog inputs connected to the input pins on the MCP3008.
channel_0 = AnalogIn(mcp, MCP.P0)
def _range(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
def callback(channel):
    print("channel",channel)
    global flame
    flame=1
    print("flame detected")
    #r= requests.get('https://www.fast2sms.com/dev/bulkV2?authorization=CZajTYBFxqW9thEIuMd3QKlceHgikzSyRo0JL7b6UPVn4fpvsDJd6pvkVMt71eGDcnKIRBqQHzgNo5L9&message=Alert!!Fire Detected in Warehousee&language=english&route=q&numbers=7993397777')
    #print(r)
GPIO.add_event_detect(channel, GPIO.BOTH, bouncetime=300)  # let us know when the pin goes HIGH or LOW
GPIO.add_event_callback(channel, callback)  # assign function to GPIO PIN, Run function on change


def myCommandCallback(cmd):
        print("Command received: %s" % cmd.data)
       
        
        if cmd.command == "setInterval":
                if 'interval' not in cmd.data:
                        print("Error - command is missing required information: 'interval'")
                else:
                        interval = cmd.data['interval']
        elif cmd.command == "print":
                if 'message' not in cmd.data:
                        print("Error - command is missing required information: 'message'")
                else:
                        print(cmd.data['message'])

try:
	deviceOptions = {"org": organization, "type": deviceType, "id": deviceId, "auth-method": authMethod, "auth-token": authToken}
	deviceCli = ibmiotf.device.Client(deviceOptions)
	#..............................................
	
except Exception as e:
	print("Caught exception connecting device: %s" % str(e))
	sys.exit()

# Connect and send a datapoint "hello" with value "world" into the cloud as an event of type "greeting" 10 times
deviceCli.connect()

while True:
    H, T = Adafruit_DHT.read_retry(DHT_SENSOR, DHT_PIN)
    sensorValue = _range(channel_0.value, 0, 60000, 0, 1023)#gas sensor
    if H is not None and T is not None:
        print("Temp={0:0.1f}*C  Humidity={1:0.1f}%".format(T, H))
    else:
        print("Failed to retrieve data from humidity sensor")
    if T>30:
        GPIO.output(16,True)
        GPIO.output(27,False)#light
        print("Fan ON")
    else:
        GPIO.output(27,True)
        GPIO.output(16,False)#fan
        print('Light ON')
        
    sleep(0.5) #Wait 5 seconds and read again
    if GPIO.input(23):#pir
        M='detected'
        GPIO.output(24, True)#buzzer
        print("Motion Detected...")
        time.sleep(1)
    else:
            M='Not Detected'
            GPIO.output(24, False)
    if (T>40 and sensorValue>150):
        print("Spoil Alert!!!")
        #q=requests.get('https://www.fast2sms.com/dev/bulkV2?authorization=CZajTYBFxqW9thEIuMd3QKlceHgikzSyRo0JL7b6UPVn4fpvsDJd6pvkVMt71eGDcnKIRBqQHzgNo5L9&message=Food Spoil Alert&language=english&route=q&numbers=7993397777')
        #print(q)
 
    data = { 'Temperature' : T, 'Humidity': H ,'Motion':M,'Flame':flame ,'Gas_Sensor':sensorValue}
    print (data)
    def myOnPublishCallback():
        print ("Published Temperature = %s C" % T, "Humidity = %s %%" % H, "to IBM Watson")

    success = deviceCli.publishEvent("iotproject", "json", data, qos=0, on_publish=myOnPublishCallback)
    if not success:
        print("Not connected to IoTF")
    time.sleep(1)
    flame=0
    deviceCli.commandCallback = myCommandCallback

# Disconnect the device and application from the cloud
deviceCli.disconnect()
