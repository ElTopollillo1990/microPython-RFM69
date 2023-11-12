"""
rfm69.py - Limited library for RFM69 Packet Radio module (SPI interface)
based on LowPowerLabs Arduino implementation (http://lowpowerlab.com/)

2023 - @languer

credits:
    https://github.com/LowPowerLab/RFM69
    https://github.com/etrombly/RFM69
    https://github.com/mchobby/esp8266-upy/tree/master/rfm69
"""

from rfm69_1_registers import *
from micropython import const
from time import sleep, sleep_us, sleep_ms, ticks_ms, ticks_diff

class RFM69:
    def __init__(self, spi=None, freqBand=None, nodeID=None, networkID=None, isRFM69HW=False,
                 intPin=None, rstPin=None, csPin=None, DEBUG=False):
        self.resetPin = rstPin
        self.csPin = csPin
        self.spiBus = spi
        self.intPin = intPin
        self.freqBand = freqBand
        self.address = nodeID
        self.networkID = networkID
        self.isRFM69HW = isRFM69HW
        self.intLock = False
        self.mode = ""
        self.promiscuousMode = False
        self.DATASENT = False
        self.DATALEN = 0
        self.SENDERID = 0
        self.TARGETID = 0
        self.PAYLOADLEN = 0
        self.ACK_REQUESTED = 0
        self.ACK_RECEIVED = 0
        self.RSSI = 0
        self.DATA = []
        self.sendSleepTime = 0.05
        self.DEBUG = DEBUG

        frfMSB = {RF69_315MHZ: RF_FRFMSB_315, RF69_433MHZ: RF_FRFMSB_433,
                  RF69_868MHZ: RF_FRFMSB_868, RF69_915MHZ: RF_FRFMSB_915}
        frfMID = {RF69_315MHZ: RF_FRFMID_315, RF69_433MHZ: RF_FRFMID_433,
                  RF69_868MHZ: RF_FRFMID_868, RF69_915MHZ: RF_FRFMID_915}
        frfLSB = {RF69_315MHZ: RF_FRFLSB_315, RF69_433MHZ: RF_FRFLSB_433,
                  RF69_868MHZ: RF_FRFLSB_868, RF69_915MHZ: RF_FRFLSB_915}

        self.CONFIG = {
        0x01: [REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY],
        #no shaping
        0x02: [REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00],
        #default:55 KBPS
        0x03: [REG_BITRATEMSB, RF_BITRATEMSB_55555],
        0x04: [REG_BITRATELSB, RF_BITRATELSB_55555],
        #default:5khz, (FDEV + BitRate/2 <= 500Khz)
        0x05: [REG_FDEVMSB, RF_FDEVMSB_50000],
        0x06: [REG_FDEVLSB, RF_FDEVLSB_50000],
        0x07: [REG_FRFMSB, frfMSB[freqBand]],
        0x08: [REG_FRFMID, frfMID[freqBand]],
        0x09: [REG_FRFLSB, frfLSB[freqBand]],
        #0x11: [REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111],
        #over current protection (default is 95mA)
        #0x13: [REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95],
        # RXBW defaults are { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5} (RxBw: 10.4khz)
        0x19: [REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2],
        #for BR-19200: //* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_3 },
        #DIO0 is the only IRQ we're using
        0x25: [REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01],
        #must be set to dBm = (-Sensitivity / 2) - default is 0xE4=228 so -114dBm
        0x26: [REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF], #DIO5 ClkOut disable for power saving
        0x28: [REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN], #writing to this bit ensures that the FIFO & status flags are reset
        0x29: [REG_RSSITHRESH, 220],
        #/* 0x2d */ { REG_PREAMBLELSB, RF_PREAMBLESIZE_LSB_VALUE } // default 3 preamble bytes 0xAAAAAA
        0x2e: [REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0],
        #attempt to make this compatible with sync1 byte of RFM12B lib
        0x2f: [REG_SYNCVALUE1, 0x2D],
		#NETWORK ID
        0x30: [REG_SYNCVALUE2, networkID],
		0x37: [REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF |
            RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF],
        #in variable length mode: the max frame size, not used in TX
        0x38: [REG_PAYLOADLENGTH, 66],
		#* 0x39 */ { REG_NODEADRS, nodeID }, //turned off because we're not using address filtering
        #TX on FIFO not empty
		0x3C: [REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE],
		#RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
        0x3d: [REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF],
        #for BR-19200: //* 0x3d */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_NONE | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, //RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
		#* 0x6F */ { REG_TESTDAGC, RF_DAGC_CONTINUOUS }, // run DAGC continuously in RX mode
        # run DAGC continuously in RX mode, recommended default for AfcLowBetaOn=0
        0x6F: [REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0],
        0x00: [255, 0]
        }

        self.reset()
        self.verifySync()
        self.writeConfig()

    @property
    def version(self):
        return self.readReg( REG_VERSION )

    @property
    def SenderID(self):
        return self.SENDERID
        
    @property
    def TargetID(self):
        return self.TARGETID

    @property
    def PacketLen(self):
        return self.PAYLOADLEN


    def reset(self):
        # Reset the module, then check it's working.
        self.csPin.high()
        self.resetPin.low()
        sleep_us(100)
        self.resetPin.high()
        sleep_us(100)
        self.resetPin.low()
        sleep_ms(5)

    def verifySync(self):
        # Setup RFM SYNC values
        while self.readReg(REG_SYNCVALUE1) != 0xAA:
            self.writeReg(REG_SYNCVALUE1, 0xAA)
        while self.readReg(REG_SYNCVALUE1) != 0x55:
            self.writeReg(REG_SYNCVALUE1, 0x55)

    def writeConfig(self):
        # Write RFM config
        for value in self.CONFIG.values():
            self.writeReg(value[0], value[1])
        # Basic RFM setup
        self.encrypt(0)
        self.setHighPower(self.isRFM69HW)
        # Wait for ModeReady
        while (self.readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00:
            pass
 
    def shutdown(self):
        self.setHighPower(False)
        self.sleep()
        
    def sleep(self):
        self.setMode(RF69_MODE_SLEEP)
        
    def readTemperature(self, calFactor):
        self.setMode(RF69_MODE_STANDBY)
        self.writeReg(REG_TEMP1, RF_TEMP1_MEAS_START)
        while self.readReg(REG_TEMP1) & RF_TEMP1_MEAS_RUNNING:
            pass
        # COURSE_TEMP_COEF puts reading in the ballpark, user can add additional correction
        #'complement'corrects the slope, rising temp = rising val
        return (int(self.readReg(REG_TEMP2))) + COURSE_TEMP_COEF + calFactor

    def rcCalibration(self):
        self.writeReg(REG_OSC1, RF_OSC1_RCCAL_START)
        while self.readReg(REG_OSC1) & RF_OSC1_RCCAL_DONE == 0x00:
            pass

    def setMode(self, newMode):
        if newMode == self.mode:
            return
        if newMode == RF69_MODE_TX:
            self.writeReg(REG_OPMODE, (self.readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_TRANSMITTER)
            if self.isRFM69HW:
                self.setHighPowerRegs(True)
        elif newMode == RF69_MODE_RX:
            self.writeReg(REG_OPMODE, (self.readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER)
            if self.isRFM69HW:
                self.setHighPowerRegs(False)
        elif newMode == RF69_MODE_SYNTH:
            self.writeReg(REG_OPMODE, (self.readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SYNTHESIZER)
        elif newMode == RF69_MODE_STANDBY:
            self.writeReg(REG_OPMODE, (self.readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY)
        elif newMode == RF69_MODE_SLEEP:
            self.writeReg(REG_OPMODE, (self.readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SLEEP)
        else:
            return
        # RFM ia using packet mode so this check is not needed;
        # but waiting for mode ready is necessary when going from sleep because the FIFO may not be immediately available from previous mode
        while self.mode == RF69_MODE_SLEEP and self.readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY == 0x00:
            pass
        self.mode = newMode;

    def promiscuous(self, onOff):
        self.promiscuousMode = onOff

    def setAddress(self, addr):
        self.address = addr
        self.writeReg(REG_NODEADRS, self.address)

    def setNetwork(self, networkID):
        self.networkID = networkID
        self.writeReg(REG_SYNCVALUE2, networkID)
        
    def enableCrc(self, value):
        reg = self.readReg((REG_PACKETCONFIG1) & 0b11101111)
        self.writeReg(REG_PACKETCONFIG1, reg | (value << 4)) 

    def encrypt(self, key):
        self.setMode(RF69_MODE_STANDBY)
        if key != 0 and len(key) == 16:
            self.writeReg([REG_AESKEY1 | 0x80] + [int(ord(i)) for i in list(key)])
            self.writeReg(REG_PACKETCONFIG2,(self.readReg(REG_PACKETCONFIG2) & 0xFE) | RF_PACKET2_AES_ON)
        else:
            self.writeReg(REG_PACKETCONFIG2,(self.readReg(REG_PACKETCONFIG2) & 0xFE) | RF_PACKET2_AES_OFF)

    def setFrequency(self, freqHz):
        step = 61.03515625
        freq = int(round(freqHz / step))
        self.writeReg(REG_FRFMSB, freq >> 16)
        self.writeReg(REG_FRFMID, freq >> 8)
        self.writeReg(REG_FRFLSB, freq)

    @property
    def getFrequency(self):
        step = 61.03515625
        freq = (self.readReg(REG_FRFMSB) << 16) + (self.readReg(REG_FRFMID) << 8) + self.readReg(REG_FRFLSB)
        return int(round(freq * step))

    def setHighPower(self, onOff):
        if onOff:
            self.writeReg(REG_OCP, RF_OCP_OFF)
            #enable P1 & P2 amplifier stages
            self.writeReg(REG_PALEVEL, (self.readReg(REG_PALEVEL) & 0x1F) | RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON)
        else:
            self.writeReg(REG_OCP, RF_OCP_ON)
            #enable P0 only
            self.writeReg(REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | powerLevel)

    def setHighPowerRegs(self, onOff):
        if onOff:
            self.writeReg(REG_TESTPA1, 0x5D)
            self.writeReg(REG_TESTPA2, 0x7C)
        else:
            self.writeReg(REG_TESTPA1, 0x55)
            self.writeReg(REG_TESTPA2, 0x70)

    def setPowerLevel(self, powerLevel):
        if powerLevel > 31:
            powerLevel = 31
        self.powerLevel = powerLevel
        self.writeReg(REG_PALEVEL, (self.readReg(REG_PALEVEL) & 0xE0) | self.powerLevel)

    def readRSSI(self):
        return (-self.readReg(REG_RSSIVALUE) / 2.0)

    def clearFifo( self ):
        self.setMode( RF69_MODE_STANDBY )
        self.setMode( RF69_MODE_RX )

    def receiveBegin(self):
        if (self.DEBUG): print("start of RX") #DEBUG
        self.DATALEN = 0
        self.SENDERID = 0
        self.TARGETID = 0
        self.PAYLOADLEN = 0
        self.ACK_REQUESTED = 0
        self.ACK_RECEIVED = 0
        self.RSSI = 0
        if (self.readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY):
            # avoid RX deadlocks
            self.writeReg(REG_PACKETCONFIG2, (self.readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART)
        #set DIO0 to "PAYLOADREADY" in receive mode
        self.writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01)
        self.clearFifo()

    def receiveDone(self):
#         if (self.DEBUG): print("in rx") #DEBUG
        if self.readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_TIMEOUT:
            if (self.DEBUG): print("IRQ timeout") #DEBUG
            self.writeReg(REG_PACKETCONFIG2, (self.readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART)

        elif self.mode == RF69_MODE_RX and self.readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY:
            if (self.DEBUG): print("Payload detected") #DEBUG
            self.setMode(RF69_MODE_STANDBY)
            fifo_length = self.readReg(REG_FIFO)
            if (self.DEBUG): print("Payload length = ", fifo_length) #DEBUG
            packet = self.burstReadReg(REG_FIFO, fifo_length)
            self.RSSI = self.readRSSI()
            self.PAYLOADLEN = len(packet)
            self.SENDERID = packet[1]
            self.TARGETID = packet[0]
            return packet
        elif self.mode == RF69_MODE_RX:
        # already in RX no payload yet
#             if (self.DEBUG): print("rx loop again") #DEBUG
            return None
        self.receiveBegin()
        return None

    def readAllRegs(self):
        results = []
        for address in range(1, 0x50):
            results.append([str(hex(address)), str(bin(self.readReg(address)))])
        return results
 

	# Read/Write Functions
    def readReg(self, register):  # was 'spi_read'
        # Read U8 register from module
        data = bytearray(2)
        data[0] = register & ~0x80
        data[1] = 0
        resp = bytearray(2)
        self.csPin.low()
        self.spiBus.write_readinto(data, resp )
        self.csPin.high()
        return resp[1]
    
    def burstReadReg(self, register, length):
        data = bytearray(length+1)
        data[0] = register & ~0x80
        for i in range(1,length+1):
            data[i] = 0
        # We get the length again as the first character of the buffer
        buf = bytearray(length+1)
        self.csPin.low()
        self.spiBus.write_readinto(data, buf )
        self.csPin.high()
        return buf[1:]

    def writeReg(self, register, value): # was 'spi_write'
        # Write U8 value into a module register
        if (type(value) is bytearray) or (type(value) is bytes):
            data = bytearray(1)
            data[0] = register | 0x80
            self.csPin.low()
            self.spiBus.write(data )
            self.spiBus.write(value)
            self.csPin.high()
        else:
            data = bytearray(2)
            data[0] = register | 0x80
            data[1] = value
            self.csPin.low()
            self.spiBus.write(data )
            self.csPin.high()

