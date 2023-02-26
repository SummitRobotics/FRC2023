import argparse
import copy
import cv2
from networktables import NetworkTables
import numpy as np
import threading

def yellowCountours(image):
    frame_threshold = cv2.inRange(image, (20, 0, 120), (40, 255, 255))
    #thresh2 = cv2.inRange(image, (145, 0, 150), (180, 150, 255))
    #frame_threshold = cv2.bitwise_or(frame_threshold, thresh2)
            
        #Erode Color Threshold Mask to remove noise
    frame_threshold_eroded = cv2.erode(frame_threshold, None, iterations=3)

        #Dialate Color Threshold Mask after Eroding to Beef Up detected areas for analysis
    frame_threshold_dilated = cv2.dilate(frame_threshold_eroded, None, iterations=3)

        #Find contours in eroded and dialated color threshold mask
    cnts = cv2.findContours(frame_threshold_dilated.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    return gc(cnts)

def purpleCountours(image):
    frame_threshold = cv2.inRange(image, (115, 0, 0), (165, 150, 125))
    #thresh2 = cv2.inRange(image, (170, 200, 0), (180, 255, 255))
    #frame_threshold = cv2.bitwise_or(frame_threshold, thresh2)
            
        #Erode Color Threshold Mask to remove noise
    frame_threshold_eroded = cv2.erode(frame_threshold, None, iterations=3)

        #Dialate Color Threshold Mask after Eroding to Beef Up detected areas for analysis
    frame_threshold_dilated = cv2.dilate(frame_threshold_eroded, None, iterations=3)

        #Find contours in eroded and dialated color threshold mask
    cnts = cv2.findContours(frame_threshold_dilated.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    return gc(cnts)

def makeNum(xCoord, yCoord, purple):
    mainNum = 0
    if xCoord < 0:
        xCoord *= -1
        mainNum += 100000000
    mainNum += xCoord*1000000
    if yCoord < 0:
        yCoord *= -1
        mainNum += 1000
    mainNum += yCoord*100
    if purple: 
        mainNum += 2000000000
    else: 
        mainNum += 1000000000
    mainNum += 2
    return mainNum
def gc(cnts):
    # if the length the contours tuple returned by cv2.findContours
    # is '2' then we are using either OpenCV v2.4, v4-beta, or
    # v4-official
    if len(cnts) == 2:
        cnts = cnts[0]

    # if the length of the contours tuple is '3' then we are using
    # either OpenCV v3, v4-pre, or v4-alpha
    elif len(cnts) == 3:
        cnts = cnts[1]

    # otherwise OpenCV has changed their cv2.findContours return
    # signature yet again and I have no idea WTH is going on
    else:
        raise Exception(("Contours tuple must have length 2 or 3, "
            "otherwise OpenCV changed their cv2.findContours retucv.circle(image, (center[0], center[1]), 5, (0, 0, 255), 2) "
            "in that case"))

    # return the actual contours array
    return cnts

def initNetworkTables():
    cond = threading.Condition()
    notified = [False]

    # Used to connect to network tables
    def connectionListener(connected, info):
            print(info, '; Connected=%s' % connected)
            with cond:
                notified[0] = True
                cond.notify()

    NetworkTables.initialize(server='10.54.68.2')
    NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

    with cond:
        print("Waiting")
        if not notified[0]:
            cond.wait()

def get_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--width", help='cap width', type=int, default=960)
    parser.add_argument("--height", help='cap height', type=int, default=540)

    parser.add_argument("--families", type=str, default='tag16h5')
    parser.add_argument("--nthreads", type=int, default=1)
    parser.add_argument("--quad_decimate", type=float, default=2.0)
    parser.add_argument("--quad_sigma", type=float, default=0.0)
    parser.add_argument("--refine_edges", type=int, default=1)
    parser.add_argument("--decode_sharpening", type=float, default=0.25)
    parser.add_argument("--debug", type=int, default=0)

    args = parser.parse_args()

    return args

def main():

    # initNetworkTables()
    # fieldElementNetworkTable = NetworkTables.getTable('fieldElements')

    args = get_args()

    cap_device = args.device
    cap_width = args.width
    cap_height = args.height

    cap = cv2.VideoCapture(cap_device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cap_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cap_height)

    while True:

        ret, image = cap.read()
        if not ret:
            break
        debug_image = copy.deepcopy(image)

        ballNum = 0
        largestContour = np.array([[]])
        largestContourMeasure = np.array([[]])
        llpython = [1,1,1,1,1,1,1,1]
        frame_HSV = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
        print(frame_HSV.shape) 
        frame_threshold = cv2.inRange(image, (0, 0, 200), (10, 255, 255))
        for i in range(2):
            if i == 1:
                cnts = yellowCountours(frame_HSV)
                purple = False
            else:
                cnts = purpleCountours(frame_HSV)
                purple = True
                ballNum = 4
            if len(cnts) > 0:
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and
                      # centroid
                for c in cnts:
                    yNeg = False
                    xNeg = False
                    ((x, y), radius) = cv2.minEnclosingCircle(c)
                    xCoord = (x-160)*.3725
                    yCoord = (y-120)*.3808
                    xCoord = round(xCoord, 1)
                    yCoord = round(yCoord, 1)
                    mask1 = np.zeros_like(frame_threshold)
                    contourMask = np.zeros_like(frame_threshold)
                    contourMask = cv2.drawContours(contourMask, c, -1, (255),1)
                    cv2.fillPoly(contourMask, pts =[c], color=(255))
                              # draw the circle and centroid on the frame,
                              # then update the list of tracked points
                    x,y,w,h = cv2.boundingRect(c)
                    if radius > 20:
                        mainNum = makeNum(xCoord, yCoord, purple)
                        if (ballNum < 4) and (i == 0):
                            llpython[ballNum] = mainNum
                            ballNum +=1
                        elif (8 > ballNum >= 4) and (i == 1):
                            llpython[ballNum] = mainNum
                            ballNum +=1
                        cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,0),2)
                        if cv2.countNonZero(contourMask) > cv2.countNonZero(largestContourMeasure):
                            largestContourMeasure = contourMask
                            largestContour = c
                            
        # cv2.imshow('Quorb Sight', image)
        # fieldElementNetworkTable.putNumberArray(llpython)

        # key = cv2.waitKey(1)
        # if key == 27:  # ESC
            # break
    # cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
