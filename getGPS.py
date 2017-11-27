import os
import sys
import Adafruit_DHT as dht
import paho.mqtt.client as mqtt
import json
#import piggpio
#import sleep
import subprocess
import time
import datetime
import serial
import string
import pynmea2
import sqlite3
from datetime import datetime, date
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

#-----Khai bao host server-----
THINGSBOARD_HOST = '35.198.249.118'
ACCESS_TOKEN = 'RASP_GPS_1234'

#-----Khoi tao database tam thoi-----
try:
    database = sqlite3.connect('GPS.db') #Tao/Mo file ten GPS voiws SQLITE3 DB
    cursor = database.cursor()
    cursor.execute('''CREATE TABLE IF NOT EXISTS gps(id INTEGER PRIMARY KEY, timestamp INTEGER, latitude REAL, longitude REAL, speed REAL)''')
    database.commit()
except Exception as e:
    database.rollback()
    raise e
finally:
    database.close()

#----------Dinh nghia ham chuyen doi UNIX-----
def unix_convert(year,month,day,hour,minute,second):
    day = datetime(year,month,day,hour,minute,second)
    unix = day.strftime('%s')
    return unix

#----- Dinh nghia Ham gui du lieu-----
def send_mqtt(lat,lng,speed):
    device_data = {'latitude': lat,'longitude': lng,'speed': speed}
    client.publish('v1/devices/me/telemetry', json.dumps(device_data), 1)

#----------Dinh nghia ham cap nhat database----------
def insert_db(timestamp, latitude, longitude, speed):
    db = sqlite3.connect('GPS.db')
    cursor = db.cursor()
    cursor.execute('''INSERT INTO gps(timestamp, latitude, longitude, speed) VALUES(?,?,?,?)''', (timestamp, latitude, longitude, speed))
    db.commit()
    db.close()

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
            strcheck = line.split(b',')
            if strcheck[9] != b'': 
                gprmc = pynmea2.parse(strline)
                Lat = gprmc.latitude
                Lng = gprmc.longitude
                CurrDate = gprmc.datestamp
                CurrTime = gprmc.timestamp
                unix = unix_convert(CurrDate.year,CurrDate.month,CurrDate.day,CurrTime.hour,CurrTime.minute,CurrTime.second)
                Speed = gprmc.spd_over_grnd
                LatDir = gprmc.lat_dir
                LngDir = gprmc.lon_dir
                print('********************')
                print('Thoi gian: ' + str(CurrDate) + ' ' + str(CurrTime) + ' ' + str(unix))
                print('Toa do: ' + str(Lat) + ' ' + str(LatDir) + ', ' + str(Lng) + ' ' + str(LngDir))
                print('Toc do: ' + str(Speed))
            else:
                print('Module dang khoi dong hoac khong co tin hieu GPS')
    

