import cv2
import numpy as np

if __name__ == "__main__":
    basepath = "/home/myk/workspace_szh/mid/"
    bin_thresold = 40

    img1 = "{0}input1.jpg".format(basepath)
    img2 = "{0}input2.jpg".format(basepath)
    to1 = cv2.imread(img1)
    gray1 = cv2.cvtColor(to1, cv2.COLOR_BGR2GRAY)
    gray1 = cv2.blur(gray1, (3, 3))
    to2 = cv2.imread(img2)
    gray2 = cv2.cvtColor(to2, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.blur(gray2, (3, 3))
    cv2.imwrite(basepath+"gray1.jpg", gray1)
    cv2.imwrite(basepath+"gray2.jpg", gray2)
    graydiff = cv2.absdiff(gray1, gray2)
    cv2.imwrite(basepath+"graydiff.jpg", graydiff)
    retval, graydiff_thresh = cv2.threshold(graydiff, bin_thresold, 255, cv2.THRESH_BINARY);
    cv2.imwrite(basepath+"graydiff_thresh.jpg", graydiff_thresh)
    erodeimg=cv2.erode(graydiff_thresh,(3,3),iterations=1)
    cv2.imwrite(basepath+"erodeimg.jpg", erodeimg)
    dilateimg = cv2.dilate(erodeimg, (18, 18),iterations=3)
    cv2.imwrite(basepath+"diateimg.jpg", dilateimg)

    print(np.sum(dilateimg) // 255)