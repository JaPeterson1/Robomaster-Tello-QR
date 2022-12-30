import drone
import threading
import time

t = threading.Thread(target=drone.videoThread)
t.start()
time.sleep(50)
drone.close_connection()