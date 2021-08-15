import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt


def pick_hsv(event, x, y, flags, param):
    if event == cv.EVENT_LBUTTONDOWN:

        h, s, v = tuple(channel[y,x] for channel in cv.split(frame_hsv))

        print(f'H: {h:3d} S: {s:3d} V: {v:3d}')

        # define the “lower” and “upper” range of a colour
        lower_range = np.array([h-10, s-40, v-40])
        upper_range = np.array([h+10, s+40, v+40])

        mask = cv.inRange(frame_hsv, lower_range, upper_range)
        cv.imshow("Mask", mask)


def pick_grayscale(event, x, y, flags, param):
    if event == cv.EVENT_LBUTTONDOWN:

        intensity = frame_gray[y,x]
        print(f'Intensity :{intensity:3d}')

        otsu_thresh, otsu_mask = cv.threshold(frame_gray, 0, 255, cv.THRESH_BINARY_INV + cv.THRESH_OTSU)
        print(f'Otsu Threshold Value: {otsu_thresh:f}')
        cv.imshow("Otsu Mask", otsu_mask)

        # Histogram
        gray_hist = cv.calcHist([frame_gray], [0], otsu_mask, [256], [0, 256])
        plt.figure()
        plt.title('Grayscale Histogram within Otsu Mask')
        plt.xlabel('Bins')
        plt.ylabel('# of pixels')
        plt.plot(gray_hist)
        plt.xlim([0, 256])
        plt.show()


def photo(event, x, y, flags, param):
    if event == cv.EVENT_LBUTTONDOWN:
        cv.imwrite("captured.jpg", frame, [int(cv.IMWRITE_JPEG_QUALITY), 100])


if __name__ == '__main__':

    cap = cv.VideoCapture(0)

    cv.namedWindow("Pick HSV Values")
    cv.setMouseCallback("Pick HSV Values", pick_hsv, )

    cv.namedWindow("Pick Grayscale Values")
    cv.setMouseCallback("Pick Grayscale Values", pick_grayscale, )

    cv.namedWindow("Take photos")
    cv.setMouseCallback("Take photos", photo, )

    while True:

        isTrue, frame = cap.read()

        # frame_wb = cv.xphoto.createGrayworldWB().balanceWhite(frame)
        frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        cv.imshow("Pick HSV Values", frame_hsv)

        frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        cv.imshow("Pick Grayscale Values", frame_gray)

        cv.imshow("Take photos", frame)

        if cv.waitKey(20) & 0xFF == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows()

'''
Reference
https://python3-cookbook.readthedocs.io/zh_CN/latest/c07/p04_return_multiple_values_from_function.html
推导式是可以从一个数据序列构建另一个新的数据序列的结构体
https://stackoverflow.com/questions/16940293/why-is-there-no-tuple-comprehension-in-python
https://stackoverflow.com/questions/51871134/hsv-opencv-colour-range
'''