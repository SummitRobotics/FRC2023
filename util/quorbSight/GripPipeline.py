import argparse
import cv2
import numpy
import math
from enum import Enum

# An OpenCV pipeline generated by GRIP and improved by Owen et al.

BlurType = Enum('BlurType', 'Box_Blur Gaussian_Blur Median_Filter Bilateral_Filter')

# Resize parameters
RESIZE_WIDTH = 320.0
RESIZE_HEIGHT = 240
RESIZE_INTERPOLATION = cv2.INTER_CUBIC

# Blur parameters
BLUR_TYPE = BlurType.Box_Blur
BLUR_RADIUS = 5.405405405405399

# HSL threshold parameters
HSL_HUE = [118.16546762589928, 150.96774193548387]
HSL_SAT = [52.74280575539568, 255.0]
HSL_LUM = [0.0, 255.0]

# Find contours parameter
FIND_ONLY_EXTERNAL = False

# Filter contours parameters
FILTER_MIN_AREA = 50.0
FILTER_MIN_PERIMETER = 0.0
FILTER_MIN_WIDTH = 0.0
FILTER_MAX_WIDTH = 1000.0
FILTER_MIN_HEIGHT = 0.0
FILTER_MAX_HEIGHT = 1000.0
FILTER_SOLIDITY = [80.93525179856115, 100.0]
FILTER_MAX_VERTICES = 1000000.0
FILTER_MIN_VERTICES = 0.0
FILTER_MIN_RATIO = 0.5
FILTER_MAX_RATIO = 1.5

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
        print(process(image))

def process(source0):
    # Runs the pipeline and sets all outputs to new values.
    # Index 0 from find_contours is the actual list of contours
    
    results = convex_hulls(filter_contours(find_contours(hsl_threshold(blur(resize_image(source0))))[0]))
    return len(results)

def resize_image(input):
    """Scales and image to an exact size.
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

def hsl_threshold(input):
    """Segment an image based on hue, saturation, and luminance ranges.
    Args:
        input: A BGR numpy.ndarray.
    Returns:
        A black and white numpy.ndarray.
    """
    out = cv2.cvtColor(input, cv2.COLOR_BGR2HLS)
    return cv2.inRange(out, (HSL_HUE[0], HSL_LUM[0], HSL_SAT[0]),  (HSL_HUE[1], HSL_LUM[1], HSL_SAT[1]))

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

def filter_contours(input_contours):
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
        if (ratio < FILTER_MIN_RATIO or ratio > FILTER_MAX_RATIO):
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
    
if __name__ == '__main__':
    main()
