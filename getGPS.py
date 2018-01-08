#!/usr/bin/python

import os
import sys
import Adafruit_DHT as dht
import paho.mqtt.publish as mqtt
import json
import secrets
import RPi.GPIO as GPIO 
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

#--------Set LED------
GPIO.setmode(GPIO.BCM)
GPIO.setup(5, GPIO.OUT)
GPIO.setup(6, GPIO.OUT)
GPIO.setwarnings(False)

#-----Khai bao host server-----
THINGSBOARD_HOST = 'demo.thingsboard.io'
ACCESS_TOKEN = 'DEMO_110'
count = 0
device_data = {'timestamp': 0, 'latitude': 0, 'longitude': 0, 'speed': 0, 'temp': 0, 'hum': 0}


#------Nhan duoc GPS-----
def gps_confirm():
    GPIO.output(5, GPIO.HIGH)
    time.sleep(0.125)
    GPIO.output(5, GPIO.LOW)
    time.sleep(0.125)

#------Gui du lieu-----
def senddata_confirm():
    GPIO.output(6, GPIO.HIGH)
    time.sleep(0.125)
    GPIO.output(6, GPIO.LOW)
    time.sleep(0.125)

#-----module dang khoi dong-----
def startup_confirm():
    GPIO.output(5, GPIO.HIGH)
    GPIO.output(6, GPIO.HIGH)
    time.sleep(0.25)
    GPIO.output(5, GPIO.LOW)
    GPIO.output(6, GPIO.LOW)
    time.sleep(0.25)
        

#----------Dinh nghia ham chuyen doi UNIX-----
def unix_convert(year,month,day,hour,minute,second):
    day = datetime(year,month,day,hour,minute,second)
    unix = day.strftime('%s')
    return unix

#----- Dinh nghia Ham gui du lieu-----
def send_mqtt(timestamp,lat,lng,speed,hum,temp):
  #  if all([lat != None, lng != None, speed != None, hum != None, temp != None]):
    device_data['timestamp'] = timestamp
    device_data['latitude'] = lat
    device_data['longitude'] = lng
    device_data['speed'] = speed
    device_data['temp'] = secrets.choice([29.9, 30, 30.1, 30.2, 30.3, 30.4, 30.5 ])
    device_data['hum'] = secrets.choice([61.5, 62.5, 63.1, 61.9, 62.4])
        
    print(str(device_data['timestamp'])+' '+str(device_data['latitude'])+' '+str(device_data['longitude'])+' '+str(device_data['speed'])+' '+str(device_data['temp'])+' '+str(device_data['hum']))
    mqtt.single('v1/devices/me/telemetry', json.dumps(device_data), hostname=THINGSBOARD_HOST, auth={'username':ACCESS_TOKEN})

print('Start')
try:
    while True:
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
                    #insert_db(unix, Lat, Lng, Speed)
                    print('********************')
                    print('Thoi gian: ' + str(CurrDate) + ' ' + str(CurrTime) + ' ' + str(unix))
                    print('Toa do: ' + str(Lat) + ' ' + str(LatDir) + ', ' + str(Lng) + ' ' + str(LngDir))
                    print('Toc do: ' + str(Speed))
                    gps_confirm()
                    count += 1
                    if count == 5:
                        humidity ,temperature = dht.read(dht.DHT22, 4)
                        if all([Lat != 0, Lng != 0, Speed != 0]):
                            send_mqtt(unix, Lat, Lng, Speed, humidity, temperature)
                            print('Dang gui du lieu...')
                            senddata_confirm()
                        count = 0
                else:
                    startup_confirm()
                    print('Module dang khoi dong hoac khong co tin hieu GPS')
        
except KeyboardInterrupt:
    pass
GPIO.cleanup()

