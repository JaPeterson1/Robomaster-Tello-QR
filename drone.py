from robomaster import robot
from robomaster import flight
from robomaster import camera
import time
import numpy as np
import pyzbar.pyzbar as pyzbar
import cv2 as cv
import threading
from robomaster import media
from simple_pid import PID

tl_drone = robot.Drone()
tl_drone.initialize()
tl_flight = tl_drone.flight
tl_camera = tl_drone.camera
currentCodeCenter = (0,0)
hover = False
hoverPIDX = PID(0, 0, 0, setpoint=0)
hoverPIDY = PID(0, 0, 0, setpoint=0)

print(f"Finished inializing drone with SDK version {tl_drone.get_sdk_version()}; battery is at {tl_drone.battery.get_battery()}%")

def move(x:int, y:int, z:int):
    tl_flight.go(x,y,z).wait_for_completed()

def initVideo():
    tl_camera.start_video_stream(display=False)
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
    initVideo()
    clahe = cv.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    img = tl_camera.read_video_frame(strategy="newest")
    while True:
        grayImg = clahe.apply(cv.cvtColor(img, cv.COLOR_BGR2GRAY))
        try:
            decoded = pyzbar.decode(grayImg)
            for code in decoded:
                print(code.data.decode('utf-8'))
                polygon = np.array([list(code.polygon[0]), list(code.polygon[1]), list(code.polygon[2]), list(code.polygon[3])], np.int32).reshape((-1, 1, 2))
                center = (int((code.polygon[0].x+code.polygon[1].x+code.polygon[2].x+code.polygon[3].x)/4),int((code.polygon[0].y+code.polygon[1].y+code.polygon[2].y+code.polygon[3].y)/4))
                print(center)
                if(hover):
                    currentCodeCenter = center
                    #tl_flight.rc(hoverPIDX(center[0]), hoverPIDY(center[1]))
                    print(f"RC X:{hoverPIDX(center[0]-img.shape[1]/2)}, Y{hoverPIDY(center[1]-img.shape[0]/2)}")
                cv.circle(img, center, 5, (0, 0, 255), 3)
                cv.polylines(img, [polygon], True, (0,255,0), 2)
            cv.imshow("img", img)
            cv.waitKey(33)
        except Exception as e:
            print(e)
        img = tl_camera.read_video_frame(strategy="newest")

def setHover(on:bool):
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
                coolingRunning=True
                cooling(True)
            elif droneTemp<=80 and coolingRunning:
                coolingRunning=False
                cooling(False)
            time.sleep(1)
        except Exception as e:
            print(e)

def close_connection():
    tl_drone.close()