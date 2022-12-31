import drone
import threading
import time

videoThread = threading.Thread(target=drone.videoThread)
#drone.cooling(True)
videoThread.start()
coolingThread = threading.Thread(target=drone.coolingThread)
coolingThread.start()
drone.setHover(True)
time.sleep(50)
drone.close_connection()