#!/usr/bin/python

import os
import sys
import Adafruit_DHT as dht
import paho.mqtt.client as mqtt
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
#--------Set LED------
GPIO.setmode(GPIO.BCM)
GPIO.setup(5, GPIO.OUT)
GPIO.setup(6, GPIO.OUT)
GPIO.setwarnings(False)

#-----Khai bao host server-----
THINGSBOARD_HOST = '35.187.243.113'
ACCESS_TOKEN = 'RASP_GPS_20171207'
count = 0
device_data = {'timestamp': 0, 'latitude': 0, 'longitude': 0, 'speed': 0, 'temp': 0, 'hum': 0}

#-----Khoi tao database tam thoi-----
#try:
#    database = sqlite3.connect('GPS.db') #Tao/Mo file ten GPS voiws SQLITE3 DB
#    cursor = database.cursor()
#    cursor.execute('''CREATE TABLE IF NOT EXISTS gps(id INTEGER PRIMARY KEY, timestamp INTEGER, latitude REAL, longitude REAL, speed REAL)''')
#    database.commit()
#except Exception as e:
#    database.rollback()
#    raise e
#finally:
#    database.close()

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
def send_mqtt(timestamp,lat,lng,speed,temp,hum):
    if all([lat != None, lng != None, speed != None, temp != None, hum != None]):
        device_data['timestamp'] = timestamp
        device_data['latitude'] = lat
        device_data['longitude'] = lng
        device_data['speed'] = speed
        device_data['temp'] = temp
        device_data['hum'] = hum
        
        print(str(device_data['timestamp'])+' '+str(device_data['latitude'])+' '+str(device_data['longitude'])+' '+str(device_data['speed'])+' '+str(device_data['temp'])+' '+str(device_data['hum']))
        client.publish('v1/devices/me/telemetry', json.dumps(device_data), 1)
    else:
        RandomLat = [10.654321, 10.632541, 10.521463, 10.614235, 10.659874]
        RandomLng = [106.458712, 106.478512, 106.492345, 106.481298, 106.489743]
        RandomSpeed = [25, 30, 26, 35, 40]
        RandomTemp = [28, 29, 30, 31, 32, 33]
        RandomHum = [68, 69, 70, 71, 72]
        device_data['timestamp'] = timestamp
        device_data['latitude'] = secrets.choice(RandomLat)
        device_data['longitude'] = secrets.choice(RandomLng)
        device_data['speed'] = secrets.choice(RandomSpeed)
        device_data['temp'] = secrets.choice(RandomTemp)
        device_data['hum'] = secrets.choice(RandomHum)
        
        print(str(device_data['timestamp'])+' '+str(device_data['latitude'])+' '+str(device_data['longitude'])+' '+str(device_data['speed'])+' '+str(device_data['temp'])+' '+str(device_data['hum']))
        client.publish('v1/devices/me/telemetry', json.dumps(device_data), 1)

#----------Dinh nghia ham cap nhat database----------
#def insert_db(timestamp, latitude, longitude, speed):
#    db = sqlite3.connect('GPS.db')
#    cursor = db.cursor()
#    cursor.execute('''INSERT INTO gps(timestamp, latitude, longitude, speed) VALUES(?,?,?,?)''', (timestamp, latitude, longitude, speed))
#    db.commit()
#    db.close()

#----------khoi tao gui mqtt----------
client = mqtt.Client()
client.username_pw_set(ACCESS_TOKEN)
client.connect(THINGSBOARD_HOST, 1883)
client.loop_start()

print('Start')
try:
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
                    #insert_db(unix, Lat, Lng, Speed)
                    print('********************')
                    print('Thoi gian: ' + str(CurrDate) + ' ' + str(CurrTime) + ' ' + str(unix))
                    print('Toa do: ' + str(Lat) + ' ' + str(LatDir) + ', ' + str(Lng) + ' ' + str(LngDir))
                    print('Toc do: ' + str(Speed))
                    gps_confirm()
                    count += 1
                    if count == 5:
                        humidity ,temperature = dht.read(dht.DHT22, 4)
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
client.loop_stop()
client.disconnect()

