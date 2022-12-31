import drone
import threading
import time

t = threading.Thread(target=drone.videoThread)
t.start()
drone.setHover(True)
time.sleep(50)
drone.close_connection()