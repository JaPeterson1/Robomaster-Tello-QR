import drone
import threading
import time
import cv2 as cv

videoThread = threading.Thread(target=drone.videoThread)
videoThread.start()
time.sleep(1)
drone.takeoff()
time.sleep(50)
drone.land()
drone.close_connection()