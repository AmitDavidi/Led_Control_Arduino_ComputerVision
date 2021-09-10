import cv2
import numpy as np
import pyfirmata
from time import *

# set up camera
frameWidth = 640
frameHeight = 480
camera = cv2.VideoCapture(0)  # get the camera
camera.set(3, frameWidth)  # 3 is the id for width
camera.set(4, frameHeight)  # 4 is the id for height
camera.set(10, 150)  # set brightness 150

# set up the board
board = pyfirmata.Arduino('COM7')


def blink(led_num):
    board.digital[led_num].write(1)
    sleep(0.5)
    board.digital[led_num].write(0)
    sleep(0.5)


# will support detect_red - get the mask - and return the edges of the contours
# in a given frame
def get_contours(originalFrame, mask, draw_limit):
    epsilon = 0.1
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # extreme outer countours
    len1 = len(contours)
    i = 0
    return_values = []
    while i < len1:
        area = cv2.contourArea(contours[i])  # get area of specific contour
        if (area > draw_limit):  # draw it only if its bigger than specific value
            arcLength = cv2.arcLength(contours[i], True)
            approx = cv2.approxPolyDP(contours[i], epsilon * arcLength, True)  # get the edges to return
            x, y, w, h = cv2.boundingRect(approx)  # get the edges x y w h
            # cv2.rectangle(originalFrame,(x,y),(x+w,y+h),(0,255,0),2) #draw a rectangle
            return_values.append((x + w) // 2)
        i += 1
    return return_values


# will detect the location of red led color and return the x value
# in a given frame
def detect_color(frame, lower, upper, draw_limit):
    frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # convert frame to HSV - Hue Sat Val
    # cv2.imshow("HSV frame", frameHSV)
    mask = cv2.inRange(frameHSV, lower, upper)  # upper - lower - pre determined
    # cv2.imshow("HSV frame", mask)

    return get_contours(frame, mask, draw_limit)  # return the x value of the red thing - list

    # final_frame = cv2.bitwise_and(frame,frame,mask=mask)
    # cv2.imshow("HSV frame", final_frame)


# blinks the leds and get their position
def get_initial_position(lower, upper):
    positions = {}
    isuccess, iframe = camera.read()
    for i in [8, 9]:
        board.digital[i].write(1)
        sleep(1)
        isuccess, iframe = camera.read()
        positions[i] = detect_color(iframe, lower, upper, 50)[0]
        board.digital[i].write(0)
    print(positions)
    return positions


key = 0
###track bars
# cv2.namedWindow("Track_Bars")
# cv2.resizeWindow("Track_Bars", 640, 240)
# cv2.createTrackbar("Hue_min", "Track_Bars", 0, 179, lambda x: None)
# cv2.createTrackbar("Hue_max", "Track_Bars", 179, 179, lambda x: None)
# cv2.createTrackbar("sat_min", "Track_Bars", 0, 255, lambda x: None)
# cv2.createTrackbar("sat_max", "Track_Bars", 255, 255, lambda x: None)
# cv2.createTrackbar("val_min", "Track_Bars", 0, 255, lambda x: None)
# cv2.createTrackbar("val_max", "Track_Bars", 255, 255, lambda x: None)
lower_red = np.array([0, 0, 255])
upper_red = np.array([179, 32, 255])
lower_blue = np.array([90, 118, 22])
upper_blue = np.array([128, 249, 255])

ledPositions = get_initial_position(lower_red, upper_red)

keys = list(ledPositions.keys())
keys_len = len(keys)

while key != ord('q'):
    success, frame = camera.read()  # read the img from capture - cap
    blink(8)
    blink(9)
    # track bars
    # h_min = cv2.getTrackbarPos("Hue_min", "Track_Bars")
    # h_max = cv2.getTrackbarPos("Hue_max", "Track_Bars")
    # sat_min = cv2.getTrackbarPos("sat_min", "Track_Bars")
    # sat_max = cv2.getTrackbarPos("sat_max", "Track_Bars")
    # val_min = cv2.getTrackbarPos("val_min", "Track_Bars")
    # val_max = cv2.getTrackbarPos("val_max", "Track_Bars")
    # lower_red = np.array([h_min,sat_min,val_min])
    # upper_red = np.array([h_max, sat_max, val_max])currentBlue_pos =

    currentBlue_pos = detect_color(frame, lower_blue, upper_blue, 500)

    if (currentBlue_pos != []):
        # print(currentBlue_pos[0])
        i = 0  # iterate over the keys - evaluate current position of blue with the positions of the leds
        while i < keys_len:
            if (currentBlue_pos[0] >= ledPositions[keys[i]] - 30 and currentBlue_pos[0] <= ledPositions[keys[i]] + 30):
                board.digital[keys[i]].write(
                    1)  # if the blue is near the led with key - keys[i] (i'th elem in keys list) - turn it on
            else:
                board.digital[keys[i]].write(0)  # else - just shut it off
            i += 1
    else:
        board.digital[8].write(0)
        board.digital[9].write(0)

    cv2.imshow("Webcam", frame)

    key = cv2.waitKey(1)

board.digital[8].write(0)
board.digital[9].write(0)
