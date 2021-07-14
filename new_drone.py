import cv2
import numpy as np
from djitellopy import Tello
import time


# sendControlPosition(self, positionX, positionY, positionZ, velocity, heading, rotationalVelocity):

def initializeTello():
    # METHOD: CONNECT TO TELLO

    myDrone = Tello()  # Create a Tello object
    myDrone.connect()  # Connect PC to Tello ()
    # Set all velocity and speed of drone to 0
    myDrone.for_back_velocity = 0
    myDrone.left_right_velocity = 0
    myDrone.up_down_velocity = 0
    myDrone.yaw_velocity = 0
    myDrone.speed = 0

    print(myDrone.get_battery())  # Print out percentage of battery
    myDrone.streamoff()  # Turn off video stream of drone
    myDrone.streamon()  # Turn on video stream of drone
    print("battery: ", myDrone.get_battery())
    return myDrone


def capture_img(myDrone, w=240, h=240):
    # METHOD: GET THE IMAGE FROM TELLO

    # Read frame from Tello
    myFrame = myDrone.get_frame_read()
    myFrame = myFrame.frame

    img = cv2.resize(myFrame, (w, h))  # Resize video to width = w and height = h
    return img


def move_to_center(drone):
    h = -1
    lower_blue = np.array([200, 30, 50])
    upper_blue = np.array([220, 100, 100])
    while h < 2:
        print('move to center')
        img = cv2.imread(capture_img(drone))
        img = cv2.GaussianBlur(img, (9, 9), 3)

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        _, contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        cnt = contours[0]
        M = cv2.moments(cnt)

        cx = int(M['m10'] / (M['m00'] + 0.000000000000001))
        cy = int(M['m01'] / (M['m00'] + 0.000000000000001))
        h = len(hierarchy[0])
        print(h)

        if h >= 2:
            break

        while not check_y(drone):
            if cy < 143:
                drone.move_left(10)
            elif cy > 157:
                drone.move_right(10)
            else:
                print('y ok y : ', cy)
            time.sleep(2)

        drone.sendControlWhile(0, 0, 0, 0, 1000)

        while not check_x(drone):
            if cx < 113:
                drone.move_up(10)
            elif cx > 127:
                drone.move_down(10)
            else:
                print('x ok x : ', cx)
            time.sleep(2)


def find_centroid(drone):  # centroid = 240x240 in (480x480) // need to recheck
    lower_blue = np.array([200, 30, 50])
    upper_blue = np.array([220, 100, 100])
    while True:
        img = cv2.imread(capture_img(drone))
        img = cv2.GaussianBlur(img, (9, 9), 3)

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        _, contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        print(len(hierarchy[0]))

        try:
            if len(hierarchy[0]) <= 1:
                drone.move_back(10)
                move_to_center(drone)
            else:
                cnt = contours[0]
                img = cv2.drawContours(img, contours, 0, (255, 255, 0), 3)
                M = cv2.moments(cnt)
                cx = int(M['m10'] / (M['m00'] + 0.000000000000001))
                cy = int(M['m01'] / (M['m00'] + 0.000000000000001))
                print(cx, cy)
                # cv2.imshow('mask', mask)
                # cv2.waitKey(0)
                return cx, cy
        except ValueError:
            print("go back")
            # cv2.imshow('mask', mask)
            # cv2.waitKey(0)
            drone.move_back(10)
            time.sleep(2)


def match_center(drone):
    while not check_y(drone):
        cy = find_centroid(drone)[1]
        if cy < 143:
            drone.move_down(10)
        elif cy > 157:
            drone.move_up(10)
        else:
            print('y ok y : ', cy)
        time.sleep(2)

    drone.sendControlWhile(0, 0, 0, 0, 1000)

    while not check_x(drone):
        cx = find_centroid(drone)[0]
        if cx < 113:
            drone.sendControlPosition(0, 0.1, 0, 0.5, 0, 0)
        elif cx > 127:
            drone.sendControlPosition(0, -0.1, 0, 0.5, 0, 0)
        else:
            print('x ok x : ', cx)
        time.sleep(2)

    drone.sendControlWhile(0, 0, 0, 0, 1000)

    pass_obstacle(drone)


def check_x(drone):
    lower_blue = np.array([200, 30, 50])
    upper_blue = np.array([220, 100, 100])

    img = cv2.imread(capture_img(drone))
    img = cv2.GaussianBlur(img, (9, 9), 3)

    # img = cv2.resize(img, dsize=(240,240))
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    _, contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    cnt = contours[0]
    M = cv2.moments(cnt)
    cx = int(M['m10'] / (M['m00'] + 0.000000000000001))
    cy = int(M['m01'] / (M['m00'] + 0.000000000000001))
    print('check_x : ', cx)

    if abs(cx - 120) <= 7:
        print('x true')
        return True
    else:
        return False


def check_y(drone):
    lower_blue = np.array([200, 30, 50])
    upper_blue = np.array([220, 100, 100])

    img = cv2.imread(capture_img(drone))
    img = cv2.GaussianBlur(img, (9, 9), 3)

    # img = cv2.resize(img, dsize=(240,240))
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    _, contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    cnt = contours[0]
    M = cv2.moments(cnt)
    cx = int(M['m10'] / (M['m00'] + 0.000000000000001))
    cy = int(M['m01'] / (M['m00'] + 0.000000000000001))
    print('check_y : ', cy)
    if abs(cy - 150) <= 7:
        print('y true')
        return True
    else:
        return False


def find_redpoint(drone):
    img = cv2.imread(capture_img(drone))
    upper_red = np.array([360, 100, 100])
    lower_red = np.array([340, 0, 0])
    upper_red2 = np.array([40, 100, 100])
    lower_red2 = np.array([0, 0, 0])
    img = cv2.GaussianBlur(img, (9, 9), 2.5)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_red, upper_red)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask + mask2

    point_red = np.nonzero(mask)
    num_point_red = np.size(point_red)
    print('red : ', num_point_red)
    # cv2.imshow('red', mask)
    # cv2.waitKey(0)
    return num_point_red


def find_purplepoint(drone):
    img = cv2.imread(capture_img(drone))
    upper_purple = np.array([135, 100, 100])
    lower_purple = np.array([120, 0, 0])
    img = cv2.GaussianBlur(img, (9, 9), 2.5)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_purple, upper_purple)
    point_purple = np.nonzero(mask)
    num_point_purple = np.size(point_purple)
    # cv2.imshow('purple', mask)
    print('purple : ', num_point_purple)
    return num_point_purple


def pass_obstacle(drone):
    while True:
        if find_purplepoint(drone) > 1000:
            print('detect purple point')
            drone.land()

            return 0

        if find_redpoint(drone) < 1000:
            drone.move_forward(10)

            print('not find red(purple) point')
            time.sleep(1)

        else:
            drone.rotate_counter_clockwise(-90)
            print('find red point')
            time.sleep(2)

            print('h')
            drone.move_forward(10)
            time.sleep(2)
            return 0