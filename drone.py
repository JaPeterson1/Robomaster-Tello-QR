from robomaster import robot
from robomaster import flight
from robomaster import camera
import time
import numpy as np
import pyzbar.pyzbar as pyzbar
import cv2 as cv
import threading
from robomaster import media


tl_drone = robot.Drone()
tl_drone.initialize()
tl_flight = tl_drone.flight
tl_camera = tl_drone.camera

currentCodeCenter = None

print(f"Finished inializing drone with SDK version {tl_drone.get_sdk_version()}; battery is at {tl_drone.battery.get_battery()}%, temp is at {tl_drone.get_temp()['temph']}")

def move(x:int, y:int, z:int):
    tl_flight.go(x,y,z).wait_for_completed()

def initVideo():
    tl_camera.start_video_stream(display=True)
    tl_camera.set_fps("high")
    tl_camera.set_resolution("high")
    tl_camera.set_bitrate(6)

def checkQR():
    returnedCodes = []
    clahe = cv.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    img = tl_camera.read_video_frame(strategy="newest")
    grayImg = clahe.apply(cv.cvtColor(img, cv.COLOR_BGR2GRAY))
    decoded = pyzbar.decode(grayImg)
    for code in decoded:
        polygon = np.array([list(code.polygon[0]), list(code.polygon[1]), list(code.polygon[2]), list(code.polygon[3])], np.int32).reshape((-1, 1, 2))
        cv.polylines(img, [polygon], True, (0, 255,0),2)
        center = (int((code.polygon[0].x+code.polygon[1].x+code.polygon[2].x+code.polygon[3].x)/4),int((code.polygon[0].y+code.polygon[1].y+code.polygon[2].y+code.polygon[3].y)/4))
        returnedCodes.append((code.data.decode('utf-8'), center))
    return returnedCodes

def videoThread():
    global currentCodeCenter
    initVideo()
    time.sleep(1)
    clahe = cv.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    while True:
        try:
            img = tl_camera.read_video_frame(strategy="newest")
            grayImg = clahe.apply(cv.cvtColor(img, cv.COLOR_BGR2GRAY))
            decoded = pyzbar.decode(grayImg)
            if(len(decoded)==0):
                currentCodeCenter = None
            for code in decoded:
                print(code.data.decode('utf-8'))
                polygon = np.array([list(code.polygon[0]), list(code.polygon[1]), list(code.polygon[2]), list(code.polygon[3])], np.int32).reshape((-1, 1, 2))
                center = (int((code.polygon[0].x+code.polygon[1].x+code.polygon[2].x+code.polygon[3].x)/4),int((code.polygon[0].y+code.polygon[1].y+code.polygon[2].y+code.polygon[3].y)/4))
                centerZeroed = (center[0]-img.shape[1]/2, center[1]-img.shape[0]/2)
                currentCodeCenter=centerZeroed
                cv.circle(img, center, 5, (0, 0, 255), 3)
                cv.polylines(img, [polygon], True, (0,255,0), 2)
            cv.imshow("img", img)
            cv.waitKey(33)
        except Exception as e:
            print(e)

def setHover(on:bool):
    global hover
    hover=on

def cooling(on:bool):
    if(on):
        tl_flight.motor_on().wait_for_completed()
    else:
        tl_flight.motor_off().wait_for_completed()

def coolingThread():
    coolingRunning=False
    while True:
        try:
            droneTemp = tl_drone.get_temp()['temph']
            if(droneTemp>80 and not coolingRunning):
                print(f"Drone temp reached {droneTemp}; turrning on cooling")
                coolingRunning=True
                cooling(True)
            elif droneTemp<=80 and coolingRunning:
                print(f"Drone cooling reached {droneTemp}, turning off cooling")
                coolingRunning=False
                cooling(False)
            time.sleep(5)
        except Exception as e:
            print(e)

def takeoff():
    tl_flight.takeoff().wait_for_completed()

def land():
    tl_flight.land().wait_for_completed()

def getCurrentCodeCenter():
    return currentCodeCenter

def close_connection():
    tl_drone.close()