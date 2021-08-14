import cv2 as cv


# TODO: use Machine Learning


def _calc_max_contour_area(contours):

    if not contours:
        raise ValueError

    '''
    Contours is a Python list of all the contours in the image. 
    Each individual contour is a Numpy array of (x,y) coordinates of boundary points of the object.
    '''
    contours_area = [cv.contourArea(contour) for contour in contours]
    index = contours_area.index(max(contours_area))  # 只返回第一个最大值的 index

    return contours[index], contours_area[index]
