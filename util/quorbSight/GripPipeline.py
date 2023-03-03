import argparse
import cv2
import numpy as np
from enum import Enum
from math import *
import threading
from networktables import NetworkTables

# An OpenCV pipeline generated by GRIP and improved by Owen et al.
# Recognizes cones and quorbs and publishes the results to NetworkTables

BlurType = Enum('BlurType', 'Box_Blur Gaussian_Blur Median_Filter Bilateral_Filter')
# Enums don't work. Quorb is now 0, Cone is now 1
# ElementType = Enum('Quorb', 'Cone')

# Resize parameters
RESIZE_WIDTH = 320.0
RESIZE_HEIGHT = 240.0
RESIZE_INTERPOLATION = cv2.INTER_CUBIC

# Blur parameters
BLUR_TYPE = BlurType.Box_Blur
BLUR_RADIUS = 5.405405405405399

# HSL threshold parameters
QUORB_HSL_HUE = [118.16546762589928, 150.96774193548387]
QUORB_HSL_SAT = [52.74280575539568, 255.0]
QUORB_HSL_LUM = [0.0, 255.0]
CONE_HSL_HUE = [0, 22]
CONE_HSL_SAT = [73, 255]
CONE_HSL_LUM = [70, 255]

# Find contours parameter
FIND_ONLY_EXTERNAL = False

# Filter contours parameters
FILTER_MIN_AREA = 200.0
FILTER_MIN_PERIMETER = 0.0
FILTER_MIN_WIDTH = 0.0
FILTER_MAX_WIDTH = 1000.0
FILTER_MIN_HEIGHT = 0.0
FILTER_MAX_HEIGHT = 1000.0
FILTER_SOLIDITY = [80.93525179856115, 100.0]
FILTER_MAX_VERTICES = 1000000.0
FILTER_MIN_VERTICES = 0.0
CONE_FILTER_MIN_RATIO = 0.5
CONE_FILTER_MAX_RATIO = 1.5
QUORB_FILTER_MIN_RATIO = 0.1
QUORB_FILTER_MAX_RATIO = 5.0

def get_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--width", help='cap width', type=int, default=960)
    parser.add_argument("--height", help='cap height', type=int, default=540)

    parser.add_argument("--nthreads", type=int, default=1)
    parser.add_argument("--quad_decimate", type=float, default=2.0)
    parser.add_argument("--quad_sigma", type=float, default=0.0)
    parser.add_argument("--refine_edges", type=int, default=1)
    parser.add_argument("--decode_sharpening", type=float, default=0.25)
    parser.add_argument("--debug", type=int, default=0)

    args = parser.parse_args()

    return args

def midpoint(x1, y1, x2, y2):
    # Returns the midpoint between two points
    x = (x1 + x2) / 2
    y = (y1 + y2) / 2
    return (x, y)

def distance(x1, y1, x2, y2):
    # Returns the length of a line
    return sqrt((x2 - x1)**2 + (y2 - y1)**2)

def resize_image(input):
    """Scales an image to an exact size.
    Args:
        input: A numpy.ndarray.
    Returns:
        A numpy.ndarray of the new size.
    """
    return cv2.resize(input, ((int)(RESIZE_WIDTH), (int)(RESIZE_HEIGHT)), 0, 0, RESIZE_INTERPOLATION)

def blur(src):
    """Softens an image using one of several filters.
    Args:
        src: The source mat (numpy.ndarray).
        type: The blurType to perform represented as an int.
        radius: The radius for the blur as a float.
    Returns:
        A numpy.ndarray that has been blurred.
    """
    if(type is BlurType.Box_Blur):
        ksize = int(2 * round(BLUR_RADIUS) + 1)
        return cv2.blur(src, (ksize, ksize))
    elif(type is BlurType.Gaussian_Blur):
        ksize = int(6 * round(BLUR_RADIUS) + 1)
        return cv2.GaussianBlur(src, (ksize, ksize), round(BLUR_RADIUS))
    elif(type is BlurType.Median_Filter):
        ksize = int(2 * round(BLUR_RADIUS) + 1)
        return cv2.medianBlur(src, ksize)
    else:
        return cv2.bilateralFilter(src, -1, round(BLUR_RADIUS), round(BLUR_RADIUS))

def hsl_threshold(input, type):
    """Segment an image based on hue, saturation, and luminance ranges.
    Args:
        input: A BGR numpy.ndarray.
    Returns:
        A black and white numpy.ndarray.
    """
    out = cv2.cvtColor(input, cv2.COLOR_BGR2HLS)
    if type == 0:
        return cv2.inRange(out, (QUORB_HSL_HUE[0], QUORB_HSL_LUM[0], QUORB_HSL_SAT[0]), 
            (QUORB_HSL_HUE[1], QUORB_HSL_LUM[1], QUORB_HSL_SAT[1]))
    else:
        return cv2.inRange(out, (CONE_HSL_HUE[0], CONE_HSL_LUM[0], CONE_HSL_SAT[0]), 
            (CONE_HSL_HUE[1], CONE_HSL_LUM[1], CONE_HSL_SAT[1]))

def find_contours(input):
    """Sets the values of pixels in a binary image to their distance to the nearest black pixel.
    Args:
        input: A numpy.ndarray.
    Return:
        A list of numpy.ndarray where each one represents a contour.
    """
    if(FIND_ONLY_EXTERNAL):
        mode = cv2.RETR_EXTERNAL
    else:
        mode = cv2.RETR_LIST
    method = cv2.CHAIN_APPROX_SIMPLE
    contours = cv2.findContours(input, mode=mode, method=method)
    return contours

def filter_contours(input_contours, type):
    """Filters out contours that do not meet certain criteria.
    Args:
        input_contours: Contours as a list of numpy.ndarray.
    Returns:
        Contours as a list of numpy.ndarray.
    """
    output = []
    for contour in input_contours:
        _,_,w,h = cv2.boundingRect(contour)
        if (w < FILTER_MIN_WIDTH or w > FILTER_MAX_WIDTH):
            continue
        if (h < FILTER_MIN_HEIGHT or h > FILTER_MAX_HEIGHT):
            continue
        area = cv2.contourArea(contour)
        if (area < FILTER_MIN_AREA):
            continue
        if (cv2.arcLength(contour, True) < FILTER_MIN_PERIMETER):
            continue
        hull = cv2.convexHull(contour)
        solid = 100 * area / cv2.contourArea(hull)
        if (solid < FILTER_SOLIDITY[0] or solid > FILTER_SOLIDITY[1]):
            continue
        if (len(contour) < FILTER_MIN_VERTICES or len(contour) > FILTER_MAX_VERTICES):
            continue
        ratio = (float)(w) / h
        if type == 0:
            if (ratio < QUORB_FILTER_MIN_RATIO or ratio > QUORB_FILTER_MAX_RATIO):
                continue
        else:
            if (ratio < CONE_FILTER_MIN_RATIO or ratio > CONE_FILTER_MAX_RATIO):
                continue
        output.append(contour)
    return output

def convex_hulls(input_contours):
    """Computes the convex hulls of contours.
    Args:
        input_contours: A list of numpy.ndarray that each represent a contour.
    Returns:
        A list of numpy.ndarray that each represent a contour.
    """
    output = []
    for contour in input_contours:
        output.append(cv2.convexHull(contour))
    return output

def process(source0):
    # Runs the shared part of both pipelines
    return blur(resize_image(source0))

def find_quorbs(image):
    return convex_hulls(filter_contours(find_contours(hsl_threshold(image, 0))[0], 0))

def find_cones(image):
    return convex_hulls(filter_contours(find_contours(hsl_threshold(image, 1))[0], 1))

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
    # NetworkTables.initialize(server='localhost')
    NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

    with cond:
        print("Waiting...")
        if not notified[0]:
            cond.wait()

def main():
    args = get_args()
    cap_device = args.device
    cap_width = args.width
    cap_height = args.height

    cap = cv2.VideoCapture(cap_device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cap_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cap_height)

    initNetworkTables()

    # Refer to these table names when reading from Java code
    table = NetworkTables.getTable('gameElements')
    hasTargetEntry = table.getEntry('hasTarget')
    typeEntry = table.getEntry('elementType')
    xEntry = table.getEntry('x')
    yEntry = table.getEntry('y')
    sizeEntry = table.getEntry('size')
    orientationEntry = table.getEntry('orientation')

    hasTargetEntry.setDefaultBoolean(False)
    typeEntry.setDefaultString("none")
    xEntry.setDefaultDouble(0)
    yEntry.setDefaultDouble(0)
    sizeEntry.setDefaultDouble(0)
    orientationEntry.setDefaultDouble(0)

    while True:
        ret, image = cap.read()
        if not ret:
            break
        image = process(image)
        hulls = []
        quorbs = find_quorbs(image)
        cones = find_cones(image)
        if len(quorbs) > 0:
            for quorb in find_quorbs(image):
                hulls.append((quorb, 0))

        if len(cones) > 0:
            for cone in find_cones(image):
                hulls.append((cone, 1))
        
        bestHullType = "none"
        bestHullX = 500
        bestHullY = 500
        bestHullSize = 0
        bestHullOrientation = 1000
        
        for hull in hulls:
            
            # Output of minAreaRect needs to be converted to integer coordinates of corners
            box = np.intp(cv2.boxPoints(cv2.minAreaRect(hull[0])))

            # Find midpoints and side lengths
            pt1 = midpoint(box[0][0], box[0][1], box[1][0], box[1][1])
            pt2 = midpoint(box[1][0], box[1][1], box[2][0], box[2][1])
            pt3 = midpoint(box[2][0], box[2][1], box[3][0], box[3][1])
            pt4 = midpoint(box[3][0], box[3][1], box[0][0], box[0][1])
            center = midpoint(pt1[0], pt1[1], pt3[0], pt3[1])
            len1 = distance(box[0][0], box[0][1], box[1][0], box[1][1])
            len2 = distance(box[1][0], box[1][1], box[2][0], box[2][1])
            len3 = distance(box[2][0], box[2][1], box[3][0], box[3][1])
            len4 = distance(box[3][0], box[3][1], box[0][0], box[0][1])

            # cv2.circle(image, (int(pt1[0]), int(pt1[1])), 5, (0, 255, 0), -1)
            # cv2.circle(image, (int(pt2[0]), int(pt2[1])), 5, (0, 255, 0), -1)
            # cv2.circle(image, (int(pt3[0]), int(pt3[1])), 5, (0, 255, 0), -1)
            # cv2.circle(image, (int(pt4[0]), int(pt4[1])), 5, (0, 255, 0), -1)
            # cv2.circle(image, (int(center[0]), int(center[1])), 5, (0, 255, 0), -1)

            # cv2.drawContours(image, [hull[0]], 0, (0, 255, 0), 2)
            # cv2.drawContours(image, [box], 0, (0, 255, 0), 2)

            orientation = 0

            # Figure out which two sides are longest and calculate orientation
            if len1 == max(len1, len2, len3, len4):
                cv2.line(image, (int(pt1[0]), int(pt1[1])), (int(pt3[0]), int(pt3[1])), (0, 255, 0), 2)
                orientation = atan((pt3[1] - pt1[1]) / (pt3[0] - pt1[0]))
            else:
                cv2.line(image, (int(pt2[0]), int(pt2[1])), (int(pt4[0]), int(pt4[1])), (0, 255, 0), 2)
                orientation = atan((pt4[1] - pt2[1]) / (pt4[0] - pt2[0]))

            if cv2.getContourArea(hull[0]) > bestHullSize:
                bestHullType = "quorb" if hull[1] == 0 else "cone"
                bestHullX = center[0]
                bestHullY = center[1]
                bestHullSize = cv2.getContourArea(hull[0])
                bestHullOrientation = orientation

        if not bestHullType == "none":
            hasTargetEntry.setBoolean(True)
            typeEntry.setString(bestHullType)
            xEntry.setDouble(bestHullX - 160)
            yEntry.setDouble(120 - bestHullY)
            sizeEntry.setDouble(bestHullSize)
            orientationEntry.setDouble(bestHullOrientation)
        else:
            hasTargetEntry.setBoolean(False)
            typeEntry.setString("none")
            xEntry.setDouble(500)
            yEntry.setDouble(500)
            sizeEntry.setDouble(0)
            orientationEntry.setDouble(1000)

        # cv2.imshow('Quorb Sight', image)
        # key = cv2.waitKey(1)
        # if key == 27:  # ESC
            # break

    # cv2.destroyAllWindows()

if __name__ == '__main__':
    main()