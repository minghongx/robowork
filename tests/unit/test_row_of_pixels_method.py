import cv2 as cv

from puppypi_pro.line_following_methods import row_of_pixels_method


def select_a_row(event, abscissa, ordinate, flags, param):
    if event == cv.EVENT_LBUTTONDOWN:
        try:
            abscissa_of_line_centre = row_of_pixels_method(frame, ordinate)
        except RuntimeError:  # Maybe the camera does not capture the line
            abscissa_of_line_centre = None

        # Draw a circle on the presumable line centre
        if abscissa_of_line_centre is not None:
            line_centre = (abscissa_of_line_centre, ordinate)
            cv.circle(frame, line_centre, 10, (0, 0, 255), thickness=3)


cap = cv.VideoCapture(-1)
while True:

    ret, frame = cap.read()
    if not ret:
        continue

    cv.namedWindow("Line Centre")
    cv.setMouseCallback("Line Centre", select_a_row, )
    cv.imshow('Line Centre', frame)

    if cv.waitKey(20) & 0xFF==ord('q'):
        break

cap.release()
cv.destroyAllWindows()
