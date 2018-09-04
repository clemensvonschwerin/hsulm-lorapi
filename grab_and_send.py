import wiringpi as wp
import os
import sys
from threading import Timer
import LoRaWAN
from LoRaWAN.MHDR import MHDR

# From: https:#github.com/dragino/rpi-lora-tranceiver/blob/master/dragino_lora_app_w1/main.c
# SX1272 - Raspberry connections
CHANNEL = 0
sx1272 = True

ssPin = 6
dio0 = 7
RST = 0

# Set spreading factor to SF7
sf = 7

# Set center frequency uint32_t
freq = 868100000  # in Mhz! (868.1)

#############################################
#############################################

REG_FIFO = 0x00
REG_OPMODE = 0x01
REG_FIFO_ADDR_PTR = 0x0D
REG_FIFO_TX_BASE_AD = 0x0E
REG_FIFO_RX_BASE_AD = 0x0F
REG_RX_NB_BYTES = 0x13
REG_FIFO_RX_CURRENT_ADDR = 0x10
REG_IRQ_FLAGS = 0x12
REG_DIO_MAPPING_1 = 0x40
REG_DIO_MAPPING_2 = 0x41
REG_MODEM_CONFIG = 0x1D
REG_MODEM_CONFIG2 = 0x1E
REG_MODEM_CONFIG3 = 0x26
REG_SYMB_TIMEOUT_LSB = 0x1F
REG_PKT_SNR_VALUE = 0x19
REG_PAYLOAD_LENGTH = 0x22
REG_IRQ_FLAGS_MASK = 0x11
REG_MAX_PAYLOAD_LENGTH = 0x23
REG_HOP_PERIOD = 0x24
REG_SYNC_WORD = 0x39
REG_VERSION = 0x42

PAYLOAD_LENGTH = 0x40

# LOW NOISE AMPLIFIER
REG_LNA = 0x0C
LNA_MAX_GAIN = 0x23
LNA_OFF_GAIN = 0x00
LNA_LOW_GAIN = 0x20

RegDioMapping1 = 0x40 # common
RegDioMapping2 = 0x41 # common

RegPaConfig = 0x09 # common
RegPaRamp = 0x0A # common
RegPaDac = 0x5A # common

SX72_MC2_FSK = 0x00
SX72_MC2_SF7 = 0x70
SX72_MC2_SF8 = 0x80
SX72_MC2_SF9 = 0x90
SX72_MC2_SF10 = 0xA0
SX72_MC2_SF11 = 0xB0
SX72_MC2_SF12 = 0xC0

SX72_MC1_LOW_DATA_RATE_OPTIMIZE = 0x01 # mandated for SF11 and SF12

# sx1276 RegModemConfig1
SX1276_MC1_BW_125 = 0x70
SX1276_MC1_BW_250 = 0x80
SX1276_MC1_BW_500 = 0x90
SX1276_MC1_CR_4_5 = 0x02
SX1276_MC1_CR_4_6 = 0x04
SX1276_MC1_CR_4_7 = 0x06
SX1276_MC1_CR_4_8 = 0x08

SX1276_MC1_IMPLICIT_HEADER_MODE_ON = 0x01

# sx1276 RegModemConfig2
SX1276_MC2_RX_PAYLOAD_CRCON = 0x04

# sx1276 RegModemConfig3
SX1276_MC3_LOW_DATA_RATE_OPTIMIZE = 0x08
SX1276_MC3_AGCAUTO = 0x04

# preamble for lora networks(nibbles swapped)
LORA_MAC_PREAMBLE = 0x34

RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG1 = 0x0A
RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2 = 0x70 # LoRa/GPS HEAD is based on the SX1276 transceiver

# FRF
REG_FRF_MSB = 0x06
REG_FRF_MID = 0x07
REG_FRF_LSB = 0x08

FRF_MSB = 0xD9 # 868.1 Mhz
FRF_MID = 0x06
FRF_LSB = 0x66

# ----------------------------------------
# Constants for radio registers
OPMODE_LORA = 0x80
OPMODE_MASK = 0x07
OPMODE_SLEEP = 0x00
OPMODE_STANDBY = 0x01
OPMODE_FSTX = 0x02
OPMODE_TX = 0x03
OPMODE_FSRX = 0x04
OPMODE_RX = 0x05
OPMODE_RX_SINGLE = 0x06
OPMODE_CAD = 0x07

# ----------------------------------------
# Bits masking the corresponding IRQs from the radio
IRQ_LORA_RXTOUT_MASK = 0x80
IRQ_LORA_RXDONE_MASK = 0x40
IRQ_LORA_CRCERR_MASK = 0x20
IRQ_LORA_HEADER_MASK = 0x10
IRQ_LORA_TXDONE_MASK = 0x08
IRQ_LORA_CDDONE_MASK = 0x04
IRQ_LORA_FHSSCH_MASK = 0x02
IRQ_LORA_CDDETD_MASK = 0x01

# DIO function mappings D0D1D2D3
MAP_DIO0_LORA_RXDONE = 0x00  # 00------
MAP_DIO0_LORA_TXDONE = 0x40  # 01------
MAP_DIO1_LORA_RXTOUT = 0x00  # --00----
MAP_DIO1_LORA_NOP = 0x30  # --11----
MAP_DIO2_LORA_NOP = 0xC0  # ----11--

# C / WiringPi Constants
LOW = 0
HIGH = 1
OUTPUT = 1
INPUT = 0

# The address of the node which is 10 by default
node_number = 10
msg = bytearray([node_number, 0])

# TTN init information
devaddr = [0x26, 0x01, 0x16, 0x08]
nwskey = [0x74, 0x09, 0x86, 0x05, 0x2A, 0x14, 0x07, 0xCD, 0xCB, 0xA7, 0x06, 0x1A, 0xEA, 0x6C, 0x0D, 0xA0]
appskey = [0x81, 0xB4, 0x19, 0x40, 0x57, 0x4C, 0xE6, 0xC2, 0xA1, 0x2B, 0x28, 0xDD, 0x3E, 0x55, 0x5F, 0x93]


def die(error):
    sys.stderr.write(error)
    sys.exit(1)


def selectreceiver():
    wp.digitalWrite(ssPin, LOW)


def unselectreceiver():
    wp.digitalWrite(ssPin, HIGH)


def readReg(addr):
    selectreceiver()
    spibuf = bytes([addr & 0x7F, 0x00])
    retlen, retdata = wp.wiringPiSPIDataRW(CHANNEL, spibuf)
    unselectreceiver()
    return int(retdata[1])


def writeReg(addr, value):
    spibuf = bytes([addr | 0x80, value])
    selectreceiver()
    wp.wiringPiSPIDataRW(CHANNEL, spibuf)
    unselectreceiver()


def opmode(mode):
    writeReg(REG_OPMODE, (readReg(REG_OPMODE) & (~OPMODE_MASK & 255)) | mode)


def opmodeLora():
    u = OPMODE_LORA
    if not sx1272:
        u |= 0x8 # TBD: sx1276 high freq
    writeReg(REG_OPMODE, u)


def SetupLoRa():
    global sx1272
    wp.digitalWrite(RST, HIGH)
    wp.delay(100)
    wp.digitalWrite(RST, LOW)
    wp.delay(100)

    version = readReg(REG_VERSION)

    if version == 0x22:
        # sx1272
        sx1272 = True
    else:
        # sx1276?
        wp.digitalWrite(RST, LOW)
        wp.delay(100)
        wp.digitalWrite(RST, HIGH)
        wp.delay(100)
        version = readReg(REG_VERSION)
        if version == 0x12:
            # sx1276
            sx1272 = False
        else:
            sys.stderr.write("Error: Unrecognized transceiver.\n")
            sys.exit(1)

    opmode(OPMODE_SLEEP)

    # set frequency
    frf = (freq << 19) // 32000000
    writeReg(REG_FRF_MSB, (frf >> 16) & 255)  # get uint8 part
    writeReg(REG_FRF_MID, (frf >> 8) & 255)   # get uint8 part
    writeReg(REG_FRF_LSB, (frf >> 0) & 255)   # get uint8 part

    writeReg(REG_SYNC_WORD, 0x34); # LoRaWAN public sync word

    if (sx1272):
        if (sf == 11 or sf == 12):
            writeReg(REG_MODEM_CONFIG, 0x0B)
        else:
            writeReg(REG_MODEM_CONFIG, 0x0A)
        writeReg(REG_MODEM_CONFIG2, (sf << 4) | 0x04)
    else:
        if (sf == 11 or sf == 12):
            writeReg(REG_MODEM_CONFIG3, 0x0C)
        else:
            writeReg(REG_MODEM_CONFIG3, 0x04)
        writeReg(REG_MODEM_CONFIG, 0x72)
        writeReg(REG_MODEM_CONFIG2, (sf << 4) | 0x04)

    if (sf == 10 or sf == 11 or sf == 12):
        writeReg(REG_SYMB_TIMEOUT_LSB, 0x05)
    else:
        writeReg(REG_SYMB_TIMEOUT_LSB, 0x08)
    writeReg(REG_MAX_PAYLOAD_LENGTH, 0x80)
    writeReg(REG_PAYLOAD_LENGTH, PAYLOAD_LENGTH)
    writeReg(REG_HOP_PERIOD, 0xFF)
    writeReg(REG_FIFO_ADDR_PTR, readReg(REG_FIFO_RX_BASE_AD))

    writeReg(REG_LNA, LNA_MAX_GAIN)


def receive(payload):
    global receivedbytes;
    # clear rxDone
    writeReg(REG_IRQ_FLAGS, 0x40)

    irqflags = readReg(REG_IRQ_FLAGS)

    # payload crc: 0x20
    if ((irqflags & 0x20) == 0x20):
        sys.stderr.write("Warning: CRC error\n")
        writeReg(REG_IRQ_FLAGS, 0x20)
        return False
    else:
        currentAddr = readReg(REG_FIFO_RX_CURRENT_ADDR)
        receivedCount = readReg(REG_RX_NB_BYTES)
        receivedbytes = receivedCount

        writeReg(REG_FIFO_ADDR_PTR, currentAddr)

        for i in range(receivedCount):
            payload[i] = readReg(REG_FIFO)
        return True


def receivepacket():
    message = bytearray(256)
    if (wp.digitalRead(dio0) == 1):
        if (receive(message)):
            value = readReg(REG_PKT_SNR_VALUE)
            if (value & 0x80): # The SNR sign bit is 1
                # Invert and divide by 4
                value = (((~value & 255) + 1) & 0xFF) >> 2
                SNR = -value
            else:
                # Divide by 4
                SNR = (value & 0xFF) >> 2

            if (sx1272):
                rssicorr = 139
            else:
                rssicorr = 157

        #printf("Packet RSSI: %d, ", readReg(0x1A)-rssicorr);
        #printf("RSSI: %d, ", readReg(0x1B)-rssicorr);
        #printf("SNR: %li, ", SNR);
        #printf("Length: %i", (int)receivedbytes);
        #printf("\n");
        #printf("Payload: %s\n", message);

        return message[:receivedbytes]
    return bytearray()


def configPower(pw):
    if not sx1272:
        # no boost used for now
        if (pw >= 17):
            pw = 15
        elif (pw < 2):
            pw = 2
        # check board type for BOOST pin
        writeReg(RegPaConfig, (0x80 | (pw & 0xf)) & 255)  # get uint8 part
        writeReg(RegPaDac, readReg(RegPaDac) | 0x4)

    else:
        # set PA config (2-17 dBm using PA_BOOST)
        if (pw > 17):
            pw = 17
        elif (pw < 2):
            pw = 2
        writeReg(RegPaConfig, (0x80 | (pw-2)) & 255)  # get uint8 part


def writeBuf(addr, value, len):
    spibuf = bytearray(256)
    spibuf[0] = addr | 0x80
    for i in range(len):
        spibuf[i + 1] = value[i]
    selectreceiver()
    wp.wiringPiSPIDataRW(CHANNEL, bytes(spibuf))
    unselectreceiver()


def txlora(frame, datalen):
    # set the IRQ mapping DIO0 = TxDone DIO1 = NOP DIO2 = NOP
    writeReg(RegDioMapping1, MAP_DIO0_LORA_TXDONE | MAP_DIO1_LORA_NOP | MAP_DIO2_LORA_NOP)
    # clear all radio IRQ flags
    writeReg(REG_IRQ_FLAGS, 0xFF)
    # mask all IRQs but TxDone
    writeReg(REG_IRQ_FLAGS_MASK, (~IRQ_LORA_TXDONE_MASK & 255))  # Ensure that ~ does not result in negative number
    # initialize  the payload size and address pointers
    writeReg(REG_FIFO_TX_BASE_AD, 0x00)
    writeReg(REG_FIFO_ADDR_PTR, 0x00)
    writeReg(REG_PAYLOAD_LENGTH, datalen)
    # download buffer to the radio FIFO
    writeBuf(REG_FIFO, frame, datalen)
    # now we actually start the transmission
    opmode(OPMODE_TX)

    wp.delay(10000)

    reset_to_standby()

    return {}
    #t = Timer(10, reset_to_standby)
    #t.start()
    #return t


def reset_to_standby():
    print("Switching back to standby mode!")
    opmodeLora()
    opmode(OPMODE_STANDBY)


# Send a message every 3 seconds
def sigalarm_handler(signal):
    global msg, node_number
    msg[0] = node_number
    msg[1] += 1

    txlora(msg, len(msg))
    signal.alarm(3)


if __name__ == "__main__":
    wp.wiringPiSetup()
    wp.pinMode(ssPin, wp.GPIO.OUTPUT)
    wp.pinMode(dio0, wp.GPIO.INPUT)
    wp.pinMode(RST, wp.GPIO.OUTPUT)

    wp.wiringPiSPISetup(CHANNEL, 500000)

    SetupLoRa()

    # Configure as sender
    opmodeLora()
    # enter standby mode (required for FIFO loading))
    opmode(OPMODE_STANDBY)

    writeReg(RegPaRamp, (readReg(RegPaRamp) & 0xF0) | 0x08)  # set PA ramp-up time 50 uSec

    configPower(23)

    print("Send packets at SF " + str(sf) + " on " + str(freq/1000000) + "Mhz\n")

    # Connect to TTN using LoRaWAN
    lorawan = LoRaWAN.new(nwskey, appskey)

    lorawan.create(MHDR.UNCONF_DATA_UP, {'devaddr': devaddr, 'fcnt': 1, 'data': list(map(ord, 'Python rules!'))})

    msg = bytes(lorawan.to_raw())
    while 1:
        resettimer = txlora(msg, len(msg))
        wp.delay(10000)

    # while(1):
    #     print("Scanning for people...")
    #     peopleout = os.popen("howmanypeoplearearound -a wlan0 --number --allmacaddresses").read()
    #     numpeople = int(peopleout[peopleout.find('\n')+1:])
    #     msg = bytes([node_number, numpeople])
    #     print("Found " + str(numpeople) + " people. Transmitting message!")
    #     resettimer = txlora(msg, len(msg))
