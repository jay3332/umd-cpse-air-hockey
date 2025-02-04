"""Calibrates the corners of the hockey table."""

import cv2
import numpy as np
from math import hypot

from vision import PERSPECTIVE_DST, TABLE_HEIGHT, TABLE_WIDTH

capture = cv2.VideoCapture(0)


def trackbar(name: str, *, default: int, count: int) -> None:
    cv2.createTrackbar(name, 'out', default, count, lambda _: None)


def on_click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        _ = lambda name: cv2.getTrackbarPos(name, 'out')
        x1, y1 = _('x1'), _('y1')
        x2, y2 = _('x2'), _('y2')
        x3, y3 = _('x3'), _('y3')
        x4, y4 = _('x4'), _('y4')

        distances = {
            ('x1', 'y1'): hypot(x1 - x, y1 - y),
            ('x2', 'y2'): hypot(x2 - x, y2 - y),
            ('x3', 'y3'): hypot(x3 - x, y3 - y),
            ('x4', 'y4'): hypot(x4 - x, y4 - y),
        }
        xn, yn = min(distances, key=distances.get)
        cv2.setTrackbarPos(xn, 'out', x)
        cv2.setTrackbarPos(yn, 'out', y)


cv2.namedWindow('out')
cv2.setMouseCallback('out', on_click)
has_trackbars = False

if __name__ == '__main__':
    corners = None
    while True:
        ret, f = capture.read()
        if not ret:
            break

        if not has_trackbars:
            h, w, _ = f.shape
            trackbar('x1', default=0, count=w)
            trackbar('y1', default=0, count=h)
            trackbar('x2', default=w, count=w)
            trackbar('y2', default=0, count=h)
            trackbar('x3', default=w, count=w)
            trackbar('y3', default=h, count=h)
            trackbar('x4', default=0, count=w)
            trackbar('y4', default=h, count=h)
            has_trackbars = True

        # Draw the corners
        _ = lambda name: cv2.getTrackbarPos(name, 'out')
        corners = [
            (_('x1'), _('y1')),
            (_('x2'), _('y2')),
            (_('x3'), _('y3')),
            (_('x4'), _('y4')),
        ]
        for i, (x, y) in enumerate(corners):
            cv2.circle(f, (x, y), 5, (0, 255, 0), -1)
            cv2.putText(f, f'{i + 1}', (x + 10, y + 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Draw lines between corners
        for i in range(4):
            cv2.line(f, corners[i], corners[(i + 1) % 4], (0, 255, 0), 2)

        cv2.imshow('out', f)

        matrix = cv2.getPerspectiveTransform(
            np.float32(corners),
            np.float32(PERSPECTIVE_DST),
        )
        warped = cv2.warpPerspective(f, matrix, (TABLE_WIDTH, TABLE_HEIGHT))
        cv2.imshow('warped', warped)

        if cv2.waitKey(1) & 0xff == ord('q'):
            break

    capture.release()
    cv2.destroyAllWindows()

    print('Corners:', corners)
