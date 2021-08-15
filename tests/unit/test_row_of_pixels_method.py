import cv2 as cv

from puppypi_pro.line_following_methods import row_of_pixels_method

ordinate = 470
capture = cv.VideoCapture(-1)

while True:

    ret, frame = capture.read()
    if not ret:
        continue

    try:
        abscissa = row_of_pixels_method(frame, ordinate) + frame.shape[1]/2
    except RuntimeError:  # Maybe the camera does not capture the line
        abscissa = None

    # Draw a circle on the presumable line centre
    if abscissa is not None:
        line_centre = (int(abscissa), int(ordinate))
        cv.circle(frame, line_centre, 10, (0, 0, 255), thickness=3)

    cv.imshow('Line Centre', frame)

    if cv.waitKey(20) & 0xFF==ord('q'):
        break

capture.release()
cv.destroyAllWindows()
