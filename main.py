# from new_drone import *
import cv2
import numpy as np
from djitellopy import tello
import new_drone
# from e_drone.drone import *
# from e_drone.protocol import *
import time

try:
    mydrone = new_drone.initializeTello()
    mydrone.takeoff()
    time.sleep(3)
    print('start')
    new_drone.match_center(mydrone)
    print('finish1')
    time.sleep(5)
    new_drone.match_center(mydrone)
    print('finish2')
    time.sleep(5)
    new_drone.match_center(mydrone)
    print('finish3')
except KeyboardInterrupt:
    mydrone.land()
    print('stop')
except Exception as e:
    mydrone.land()
    print(e)