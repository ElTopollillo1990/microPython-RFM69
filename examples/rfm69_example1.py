"""
Basic example using rfm69_1.py to listen to RFM sender implemented on Arduino using LowPowerLabs Arduino library 
@languer2023-2024
"""


### LIBRARIES ###
from machine import SPI, Pin
from rfm69_1 import RFM69
import time


### DEFINITIONS ###
# PICOW DEFs
led = Pin("LED", Pin.OUT)

#RFM69 DEFs
 # (frequency register) # 
RF69_315MHZ = 31
RF69_433MHZ = 43
RF69_868MHZ = 86
RF69_915MHZ = 91

 # (rfm "network" IDs) #
network = 0
thisNode = 1

 # (needed for RFM69HW) #
rfm_69HW = True
 # (spi definitions) #
rfm_spi = SPI(0, baudrate=50000, polarity=0, phase=0, firstbit=SPI.MSB, 
              sck=machine.Pin(18), mosi=machine.Pin(19), miso=machine.Pin(16))
rfm_cs = Pin( 17, Pin.OUT, value=True )
 # (reset and interrupt pin definitions - note that interrtup pin is not really used in this implementation) #
rfm_rst = Pin( 20, Pin.OUT, value=False )
rfm_int = Pin( 22, Pin.IN )


### FUNCTIONS ###
# blink onboard LED
def blink_led():
    led.off()
    for cnt in range(4): 
        led.toggle()
        time.sleep(0.2)


### MAIN PROGRAM ###
# blink onboard LED
blink_led()

# initialize RFM Class
radio = RFM69( spi=rfm_spi, freqBand = RF69_915MHZ, nodeID = thisNode, networkID = network,
               isRFM69HW = rfm_69HW, intPin = rfm_int, rstPin = rfm_rst, csPin = rfm_cs, DEBUG = False)
print("RFM class initialized ...")
# enable RFM CRC
radio.enableCrc(True)

# read RFM configuration
print("Reading registers...")
print("==================================================")
print("RFM Version               :", radio.version)
print("Internal Temperature      :", radio.readTemperature(-5))
print("Frequency                 :", radio.getFrequency)
print("Frequency Register (MSB)  :", str(hex(radio.readReg( 0x07 ))))
print("Frequency Register (MID)  :", str(hex(radio.readReg( 0x08 ))))
print("Frequency Register (LSB)  :", str(hex(radio.readReg( 0x09 ))))
print("==================================================")
time.sleep(2.5)
print("Reading All Registers")
results = radio.readAllRegs()
for result in results:
    print(result)
print("==================================================")
time.sleep(5)
print("shutting down")
radio.shutdown()
