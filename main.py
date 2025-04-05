from multiprocessing import Process, Value
from threading import *
from bleak import BleakClient, uuids
from gpiozero import LED
from suntime import Sun
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder, Quality
from datetime import datetime
import cv2
import pytz
import os
import subprocess
import pyrebase
import asyncio
import time
import board
import neopixel_spi as neopixel

firebaseConfig = {'apiKey': " AIzaSyB4lIjFLY2AY9tbKafF8Ss_Zz0Jhf9UYVU",
'authDomain': "helmet-e424f.firebaseapp.com",
'databaseURL': "https://helmet-e424f.firebaseio.com",
'projectId': "helmet-e424f",
'storageBucket': "helmet-e424f.appspot.com",
'messagingSenderId': "534090770089",
'appId': "1:534090770089:web:f27c86341b3daa7f306bb8",
'measurementId': "G-D5XJE1GWLS"}

firebase = pyrebase.initialize_app(firebaseConfig)
storage = firebase.storage()

cam = Picamera2()
config = cam.create_video_configuration(main={"size": (1280, 720), "format": "RGB888"})
cam.configure(config)
cam.start()

face_cascade = cv2.CascadeClassifier("/home/pi/haarcascade_frontalface_default.xml")

frame_count = 0
start_time = time.time()

encoder = H264Encoder()

num_pixels = 60
spi = board.SPI()
pixels = neopixel.NeoPixel_SPI(spi, num_pixels)

laser = LED(24)
laser.on()
# Replace with the MAC address of your Raspberry Pi Pico W
pico_address = "28:CD:C1:0B:6D:EA"

SERVICE_UUID = uuids.normalize_uuid_16(0x1848)
CHARACTERISTIC_UUID = uuids.normalize_uuid_16(0x2A6E) 

# Belleville, Ontario, Canada coordinates
latitude = 44.1668
longitude = -77.3835

# Set timezone to Eastern Time (since Belleville is in this timezone)
eastern = pytz.timezone('Canada/Eastern')

# Get the Sun object for the location
sun = Sun(latitude, longitude)

flag = Value('b', False)

def recvid():
        while True:
                if flag.value:
                        cam.start_recording(encoder, "/home/pi/INCIDENT.h264")
                        time.sleep(5)
                        cam.stop_recording()
                        
                        flag.value = False

async def connect_and_communicate(address):
    print(f"Connecting to {address}...")
    
    async with BleakClient(address) as client:
        print(f"Connected: {client.is_connected}")

        while True:
            response = await client.read_gatt_char(CHARACTERISTIC_UUID)
            message = response.decode('utf-8')
            print(f"Received: {message}")

            if message == "Left Pressed":
                print("left")
                for _ in range(5):
                    for i in range(23):
                        pixels[i] = ((255, 0, 0))
                        pixels.show()
                        time.sleep(.05)
                    pixels.fill((0, 0, 0))
            elif message == "Right Pressed":
                print("right")
                for _ in range(5):
                    for i in range(46, 22, -1):
                        pixels[i] = ((255, 0, 0))
                        pixels.show()
                        time.sleep(.05)
                    pixels.fill((0, 0, 0))
            await asyncio.sleep(.5)

# Function to run BLE in a separate thread
def run_ble():
    asyncio.run(connect_and_communicate(pico_address))

def is_dark_outside():
    while True:
        # Get the current time in the local timezone (Eastern)
        current_time = datetime.now(eastern)

        # Get today's sunset time
        sunset_time = sun.get_sunset_time(current_time)

        # Check if the current time is past the sunset time
        if current_time < sunset_time:
            laser.on()
        else:
            laser.off()
        
        # Sleep for 1 minute before checking again
        time.sleep(60)

rec = Process(target=recvid)
rec.start()

ble = Thread(target=run_ble)
ble.start()

dark = Thread(target=is_dark_outside)
dark.start()

try:
        while True:
                frame = cam.capture_array()
                
                faces = face_cascade.detectMultiScale(frame, scaleFactor=1.3, minNeighbors=5)
                if len(faces) > 0:
                        print("Face detected")
                        flag.value = True
                frame_count+=1
                elasped_time = time.time() - start_time
                fps = frame_count / elasped_time
                print(f"FPS: {fps:.2f}")

except KeyboardInterrupt:
        print("dis is not working")

finally:
        rec.terminate()
        
        frame = cam.capture_array()
        cv2.imwrite("/home/pi/INCIDENT.jpg", frame)
        storage.child("INCIDENT.jpg").put("/home/pi/INCIDENT.jpg")

        subprocess.run(["ffmpeg", "-y", "-i", "/home/pi/INCIDENT.h264", "-c:v", "copy", "/home/pi/INCIDENTLow.mp4"])
        subprocess.run(['ffmpeg', '-y', '-i', '/home/pi/INCIDENTLow.mp4', '-b:v', '2000k', '-bufsize', '500k', '/home/pi/INCIDENT.mp4'])
        storage.child("INCIDENT.mp4").put("/home/pi/INCIDENT.mp4")
        
        print("done saving")

