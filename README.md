# microPython-RFM69 (unfinished work)
RFM69 Driver for MicroPython
## rfm69.py - Limited library for RFM69 Packet Radio module (SPI interface)
based on LowPowerLabs Arduino implementation (http://lowpowerlab.com/)
* port of receiver library working
* port of transmitter library ***not complete***

2023-2024 - @languer
https://github.com/ElTopollillo1990/microPython-RFM69/

#### Credits:
* https://github.com/LowPowerLab/RFM69
* https://github.com/etrombly/RFM69
* https://github.com/mchobby/esp8266-upy/tree/master/rfm69

#### Basic Usage
Currently this only works by receiving packets from a sender unit that uses LowPower Lab's Arduino code.

*Main Imports*
```python
from rfm69_1_registers import *
from micropython import const
from time import sleep, sleep_us, sleep_ms, ticks_ms, ticks_diff
```
*Define RFM Class*
```python
class RFM69:
    def __init__(self, spi=None, freqBand=None, nodeID=None, networkID=None, isRFM69HW=False,
                 intPin=None, rstPin=None, csPin=None, DEBUG=False):
```
The class needs to be initialized with the proper values (see example 1 or 2) for more information.

*Properties (most useful ones - other properties are defined in library and examples provide additional usage information)*
>- reset -> resets RFM module 
>- shutdown -> shutsdown RFM module
>- sleep -> sleeps RFM module
>- readTemperature -> returns RFM internal temperature
>- getFrequency -> returns RFM frequency
>- SenderID -> returns source node from where packet is being sent (i.e. sender)
>- TargetID -> returns target node to where packet is being sent (i.e. recipient)
>- PacketLen -> returns length of received packet
>- RSSI -> returns received signal strength from last received packet
>- receiveDone -> indicates if a packet has been received and returns the results of the packet <

