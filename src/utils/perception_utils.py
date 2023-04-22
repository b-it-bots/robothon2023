import cv2

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
