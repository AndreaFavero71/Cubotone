#!/usr/bin/env python
# -*- coding: utf-8 -*-


# from https://github.com/Bogdanel/Raspberry-Pi-Python-3-TM1637-Clock
# resume table with possible letters 

"""
Display	Bin	Hex	Dec	HexDigit Index
0	0b00111111	0x3F	63	0
1	0b00000110	0x06	6	1
2	0b01011011	0x5B	91	2
3	0b01001111	0x4F	79	3
4	0b01100110	0x66	102	4
5	0b01101101	0x6D	109	5
6	0b01111101	0x7D	125	6
7	0b00000111	0x07	7	7
8	0b01111111	0x7F	127	8
9	0b01101111	0x6F	111	9
A	0b01110111	0x77	119	10
b	0b01111100	0x7C	124	11
C	0b00111001	0x39	57	12
d	0b01011110	0x5E	94	13
E	0b01111001	0x79	121	14
F	0b01110001	0x71	113	15
G	0b00111101	0x3D	61	16
H	0b01110110	0x76	118	17
I	0b00000110	0x06	6	18
J	0b00011110	0x1E	30	19
K	0b01110110	0x76	118	20
L	0b00111000	0x38	56	21
M	0b01010101	0x55	85	22
n	0b01010100	0x54	84	23
O	0b00111111	0x3F	63	24
P	0b01110011	0x73	115	25
q	0b01100111	0x67	103	26
r	0b01010000	0x50	80	27
S	0b01101101	0x6D	109	28
t	0b01111000	0x78	120	29
U	0b00111110	0x3E	62	30
v	0b00011100	0x1C	28	31
W	0b00101010	0x2A	42	32
X	0b01110110	0x76	118	33
y	0b01101110	0x6E	110	34
Z	0b01011011	0x5B	91	35
blank	0b00000000	0x00	0	36
-	0b01000000	0x40	64	37
*	0b01100011	0x63	99	38
"""


import math
import RPi.GPIO as GPIO
import threading
from time import sleep, localtime


GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

HexDigits = [0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 
            0x6F, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71, 0x3D, 0x76, 
            0x06, 0x1E, 0x76, 0x38, 0x55, 0x54, 0x3F, 0x73, 0x67, 
            0x50, 0x6D, 0x78, 0x3E, 0x1C, 0x2A, 0x76, 0x6E, 0x5B,
            0x00, 0x40, 0x63, 0xFF]

ADDR_AUTO = 0x40
ADDR_FIXED = 0x44
STARTADDR = 0xC0
# DEBUG = False


class TM1637:
    __doublePoint = False
    __Clkpin = 0
    __Datapin = 0
    __brightness = 1.0  # default to max brightness
    __currentData = [0, 0, 0, 0]

    def __init__(self, CLK, DIO, brightness):
        self.__Clkpin = CLK
        self.__Datapin = DIO
        self.__brightness = brightness
        GPIO.setwarnings(False)
        GPIO.setup(self.__Clkpin, GPIO.OUT)
        GPIO.setup(self.__Datapin, GPIO.OUT)

    def cleanup(self):
        """Stop updating clock, turn off display, and cleanup GPIO"""
        self.StopClock()
        self.Clear()
        GPIO.cleanup()

    def Clear(self):
        b = self.__brightness
        point = self.__doublePoint
        self.__brightness = 0
        self.__doublePoint = False
        data = [0x7F, 0x7F, 0x7F, 0x7F]
        self.Show(data)
        # Restore previous settings:
        self.__brightness = b
        self.__doublePoint = point

    def ShowInt(self, i):
        s = str(i)
        self.Clear()
        for i in range(0, len(s)):
            self.Show1(i, int(s[i]))

    def Show(self, data):
        for i in range(0, 4):
            self.__currentData[i] = data[i]

        self.start()
        self.writeByte(ADDR_AUTO)
        self.br()
        self.writeByte(STARTADDR)
        for i in range(0, 4):
            self.writeByte(self.coding(data[i]))
        self.br()
        self.writeByte(0x88 + int(self.__brightness))
        self.stop()

    def Show1(self, DigitNumber, data):
        """show one Digit (number 0...3)"""
        if(DigitNumber < 0 or DigitNumber > 3):
            return  # error

        self.__currentData[DigitNumber] = data

        self.start()
        self.writeByte(ADDR_FIXED)
        self.br()
        self.writeByte(STARTADDR | DigitNumber)
        self.writeByte(self.coding(data))
        self.br()
        self.writeByte(0x88 + int(self.__brightness))
        self.stop()
    
    # Scrolls any integer n (can be more than 4 digits) from right to left display.
    def ShowScroll(self, n):
        n_str = str(n)
        k = len(n_str)

        for i in range(0, k + 4):
            if (i < k):
                self.Show([int(n_str[i-3]) if i-3 >= 0 else None, int(n_str[i-2]) if i-2 >= 0 else None, int(n_str[i-1]) if i-1 >= 0 else None, int(n_str[i]) if i >= 0 else None])
            elif (i >= k):
                self.Show([int(n_str[i-3]) if (i-3 < k and i-3 >= 0) else None, int(n_str[i-2]) if (i-2 < k and i-2 >= 0) else None, int(n_str[i-1]) if (i-1 < k and i-1 >= 0) else None, None])
            sleep(1)

    def SetBrightness(self, percent):
        """Accepts percent brightness from 0 - 1"""
        max_brightness = 7.0
        brightness = math.ceil(max_brightness * percent)
        if (brightness < 0):
            brightness = 0
        if(self.__brightness != brightness):
            self.__brightness = brightness
            self.Show(self.__currentData)

    def ShowDoublepoint(self, on):
        """Show or hide double point divider"""
        if(self.__doublePoint != on):
            self.__doublePoint = on
            self.Show(self.__currentData)

    def writeByte(self, data):
        for i in range(0, 8):
            GPIO.output(self.__Clkpin, GPIO.LOW)
            if(data & 0x01):
                GPIO.output(self.__Datapin, GPIO.HIGH)
            else:
                GPIO.output(self.__Datapin, GPIO.LOW)
            data = data >> 1
            GPIO.output(self.__Clkpin, GPIO.HIGH)

        # wait for ACK
        GPIO.output(self.__Clkpin, GPIO.LOW)
        GPIO.output(self.__Datapin, GPIO.HIGH)
        GPIO.output(self.__Clkpin, GPIO.HIGH)
        GPIO.setup(self.__Datapin, GPIO.IN)

        while(GPIO.input(self.__Datapin)):
            sleep(0.001)
            if(GPIO.input(self.__Datapin)):
                GPIO.setup(self.__Datapin, GPIO.OUT)
                GPIO.output(self.__Datapin, GPIO.LOW)
                GPIO.setup(self.__Datapin, GPIO.IN)
        GPIO.setup(self.__Datapin, GPIO.OUT)

    def start(self):
        """send start signal to TM1637"""
        GPIO.output(self.__Clkpin, GPIO.HIGH)
        GPIO.output(self.__Datapin, GPIO.HIGH)
        GPIO.output(self.__Datapin, GPIO.LOW)
        GPIO.output(self.__Clkpin, GPIO.LOW)

    def stop(self):
        GPIO.output(self.__Clkpin, GPIO.LOW)
        GPIO.output(self.__Datapin, GPIO.LOW)
        GPIO.output(self.__Clkpin, GPIO.HIGH)
        GPIO.output(self.__Datapin, GPIO.HIGH)

    def br(self):
        """terse break"""
        self.stop()
        self.start()

    def coding(self, data):
        if(self.__doublePoint):
            pointData = 0x80
        else:
            pointData = 0

        if(data == 0x7F or data is None):
            data = 0
        else:
            data = HexDigits[data] + pointData
        return data

    def clock(self, military_time):
        """Clock script modified from:
            https://github.com/johnlr/raspberrypi-tm1637"""
        self.ShowDoublepoint(True)
        while (not self.__stop_event.is_set()):
            t = localtime()
            hour = t.tm_hour
            if not military_time:
                hour = 12 if (t.tm_hour % 12) == 0 else t.tm_hour % 12
            d0 = hour // 10 if hour // 10 else 36
            d1 = hour % 10
            d2 = t.tm_min // 10
            d3 = t.tm_min % 10
            digits = [d0, d1, d2, d3]
            self.Show(digits)
            # # Optional visual feedback of running alarm:
            # print digits
            # for i in tqdm(range(60 - t.tm_sec)):
            for i in range(60 - t.tm_sec):
                if (not self.__stop_event.is_set()):
                    sleep(1)

    def StartClock(self, military_time=True):
        # Stop event based on: http://stackoverflow.com/a/6524542/3219667
        self.__stop_event = threading.Event()
        self.__clock_thread = threading.Thread(
            target=self.clock, args=(military_time,))
        self.__clock_thread.start()


    def StopClock(self):
        try:
            print ('Attempting to stop live clock')
            self.__stop_event.set()
        except:
            print ('No clock to close')


    def Cam(self):
        cam=[12, 10, 22, 36]
        self.Show(cam)
    
    
    def Cal(self):
        cal=[12, 10, 21, 36]
        self.Show(cal)
    
    
    def Cube(self):
        cube=[12, 30, 8, 14]
        self.Show(cube)


    def Read(self):
        read=[27, 14, 10, 13]
        self.Show(read)
  

    def Done(self):
        done=[13, 0, 23, 14]
        self.Show(done)


    def Error(self):
        error=[14, 27, 27, 36]
        self.Show(error)        


    def Press(self):
        press=[25, 27, 14, 28]
        self.Show(press)  

        
        
if __name__ == "__main__":
    """Confirm the display operation"""
    display = TM1637(CLK=26, DIO=19, brightness=1.0)
    
    for i in range(3):
        display.Read()
        sleep(1)
        display.Cube()
        sleep(1)
    display.Clear()
    sleep(2)
    
    for i in range(3):
        display.Cube()
        sleep(1)
        display.Done()
        sleep(1)
    display.Clear()
    sleep(2)

    
          

#     display.Clear()
#     digits = [1, 2, 3, 4]
#     display.Show(digits)
#     print ("1234  - Working? (Press Key)")
#     scrap = input()
# 
#     print ("Updating one digit at a time:")
#     display.Clear()
#     display.Show1(1, 3)
#     sleep(0.5)
#     display.Show1(2, 2)
#     sleep(0.5)
#     display.Show1(3, 1)
#     sleep(0.5)
#     display.Show1(0, 4)
#     print ("4321  - (Press Key)")
#     scrap = input()
# 
#     print ("Add double point\n")
#     display.ShowDoublepoint(True)
#     sleep(0.2)
#     print ("Full Brightness")
#     display.SetBrightness(1)
#     sleep(0.5)
#     print ("30% Brightness")
#     display.SetBrightness(0.3)
#     sleep(1)
#     print ("Brightness Off")
#     display.SetBrightness(0)
#     sleep(1)
#     display.Clear()

    # See clock.py for how to use the clock functions!
