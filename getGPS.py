import time
import serial
import string
import pynmea2
from pynmea2 import nmea
#--------------------------------------------------------

#----------Khai bao serial---------------------
#-----Serial GPS data-----
serAMA = serial.Serial(
    port='/dev/ttyAMA0',
    baudrate = 9600,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS,
    timeout = 1,
    )
#-----Serial truyen GPRS-----
"""serUSB = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate = 9600,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS,
    timeout = 1,
    )
"""

print('Start')
while True:
    #-----Kiem tra Serial port-----
    #if(serAMA.isOpen() == False):
    #    print('Unable to open serial device. Try to open...\n')
    #    serAMA.open()
        
    #--------doc du lieu GPS------------
    rawdata = serAMA.readline()
    for line in rawdata.splitlines():
        if line.startswith(b'$GPRMC'): #Chi lay truong $GPGGA
            strline = line.decode('utf-8')
            gprmc = pynmea2.parse(strline)
            Lat = gprmc.latitude
            Lng = gprmc.longitude
            DateTime = gprmc.datetime
            Speed = gprmc.spd_over_grnd
            LatDir = gprmc.lat_dir
            LngDir = gprmc.lon_dir
            print('********************')
            print('Thoi gian: ' + str(DateTime))
            print('Toa do: ' + str(Lat) + ' ' + str(LatDir) + ', ' + str(Lng) + ' ' + str(LngDir))
            print('Toc do: ' + str(Speed))
    #time.sleep(1)   
    

