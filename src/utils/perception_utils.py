import cv2
import numpy as np

def dilate(img, dilation_size=1):
    dilation_shape = cv2.MORPH_RECT
    element = cv2.getStructuringElement(dilation_shape, (2 * dilation_size + 1, 2 * dilation_size + 1),
                                       (dilation_size, dilation_size))
    img = cv2.dilate(img, element)
    return img

def erode(img, erosion_size=1):
    erosion_shape = cv2.MORPH_RECT
    element = cv2.getStructuringElement(erosion_shape, (2 * erosion_size + 1, 2 * erosion_size + 1),
                                       (erosion_size, erosion_size))
    img = cv2.erode(img, element)
    return img

def get_uppermost_contour(img):
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) == 0:
        print('no contours')
        return None, None
    lowest_y = img.shape[0]
    selected_cx = 0
    selected_cy = 0
    idx = 0
    for idx, contour in enumerate(contours):
        M = cv2.moments(contour)
        if M["m00"] > 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            if cY < lowest_y and cv2.contourArea(contour) > 10:
                lowest_y = cY
                selected_cx = cX
                selected_cy = cY
                sidx = idx
    return selected_cx, selected_cy

def get_contours(img):
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours

def detect_door_circle(img):
    contours = get_contours(img)
    plain_img = np.ones_like(img, dtype=np.uint8) * 255
    cv2.drawContours(plain_img, contours, -1, (0, 0, 0), -1)
    detected_circles = cv2.HoughCircles(plain_img, cv2.HOUGH_GRADIENT, 1, 60, param1 = 70, param2 = 20, minRadius = 50, maxRadius = 70)
    if detected_circles is not None:
        detected_circles = detected_circles.astype(np.uint64)
        return detected_circles[0]
    else:
        return None
