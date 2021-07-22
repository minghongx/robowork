from itertools import groupby
from operator import itemgetter
import builtins

from cv2 import *


def row_of_pixels_method(frame, ordinate: int) -> int:
    """
    Calculate pixel difference between image centre line and line centre by only using a row of pixels of the image,
    because the line to be followed must be in the lower part of the image
    :param frame: an OpenCV-format image
    :param ordinate: ordinate of the row in the coordinate system of the frame
    :return: pixel difference between image centre line and line centre (minus means the line centre is on the left of the image centre)
    """

    if not 0 <= ordinate <= frame.shape[0]:
        raise ValueError('Coordinate sys of the frame does not have this ordinate')

    otsu_thresh, otsu_binary_img = threshold(cvtColor(frame, COLOR_BGR2GRAY), 0, 255, THRESH_BINARY_INV + THRESH_OTSU)

    opened_otsu_binary_img = morphologyEx(otsu_binary_img, MORPH_OPEN, (9,9))  # remove tiny white regions

    # Assume 一行像素中最长的白色部分是被巡线. 计算它中点像素的横坐标
    a_row_of_pixels = opened_otsu_binary_img[ordinate]
    white_pixel_abscissas = [abscissa for abscissa, intensity in enumerate(a_row_of_pixels) if intensity == 255]
    if not white_pixel_abscissas:
        raise RuntimeError('Maybe the camera does not capture the line')
    consecutive_abscissas = [list(map(itemgetter(1), group)) for key, group in groupby(enumerate(white_pixel_abscissas), lambda _: _[1]-_[0])]  # 通过将数字与其 index 相减作为 key, 把数组中连续递增 1 的部分分成一组
    presumable_line_abscissas = builtins.max(consecutive_abscissas, key=len)  # The longest white is of interest
    presumable_line_centre_abscissa = int((presumable_line_abscissas[0]+presumable_line_abscissas[-1])/2)

    return presumable_line_centre_abscissa - frame.shape[1]/2


# def thirty_millimetre_black_line_method(frame):


"""
Ref
图像的分辨率越高, 或者尺寸越大, 通常会对图像识别造成负面影响. 对图像识别来说, 图像信息能减少就减少.
https://blog.csdn.net/yzy_1996/article/details/85318179
https://stackoverflow.com/questions/2154249/identify-groups-of-continuous-numbers-in-a-list
https://stackoverflow.com/questions/773/how-do-i-use-itertools-groupby
https://stackoverflow.com/questions/48700894/tuple-unpacking-in-lambda-function
https://stackoverflow.com/questions/5893163/what-is-the-purpose-of-the-single-underscore-variable-in-python
https://stackoverflow.com/questions/35064294/how-to-get-the-groups-generated-by-groupby-as-lists
https://www.merriam-webster.com/dictionary/presumable
https://docs.python.org/3/library/builtins.html
https://stackoverflow.com/questions/28050229/access-builtin-functions-by-builtins
https://stackoverflow.com/questions/41049732/python-function-yields-tuple-and-only-one-element-is-wanted
"""
