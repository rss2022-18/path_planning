import cv2
import numpy as np
import sys


def dilate_image(img, kernel_size=5):
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    return cv2.erode(img, kernel, iterations=3)


if __name__ == '__main__':
    path = './stata_basement.png'
    if len(sys.argv) > 2:
        path = sys.argv[1]
        if len(sys.argv) > 3:
            kernel_size = int(sys.argv[2])
    img = cv2.imread('./stata_basement.png', 0)
    img = dilate_image(img)
    cv2.imwrite(path[:-4] + '_eroded.png', img)
