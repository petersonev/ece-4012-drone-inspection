# Code inspired by: https://gist.github.com/Munawwar/0efcacfb43827ba3a6bac3356315c419 and
# http://www.bogotobogo.com/python/OpenCV_Python/python_opencv3_Image_Gradient_Sobel_Laplacian_Derivatives_Edge_Detection.php and
# http://www.codepasta.com/site/vision/segmentation/

# Imports
import cv2
import numpy as np
import math


def getSobel(channel):
    sobelx = cv2.Sobel(channel, cv2.CV_16S, 1, 0, borderType=cv2.BORDER_REPLICATE)
    sobely = cv2.Sobel(channel, cv2.CV_16S, 0, 1, borderType=cv2.BORDER_REPLICATE)
    sobel = np.hypot(sobelx, sobely)

    return sobel


def getLapcian(img):
    return cv2.Laplacian(img, cv2.CV_64F)


def findSignificantContours(img, sobel_8u):
    image, contours, heirarchy = cv2.findContours(sobel_8u, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Find level 1 contours
    level1 = []
    for i, tupl in enumerate(heirarchy[0]):
        # Each array is in format (Next, Prev, First child, Parent)
        # Filter the ones without parent
        if tupl[3] == -1:
            tupl = np.insert(tupl, 0, [i])
            level1.append(tupl)

    # From among them, find the contours with large surface area.
    significant = []
    tooSmall = sobel_8u.size * 30 / 100  # If contour isn't covering 30% of total area of image then it probably is too small
    for tupl in level1:
        contour = contours[tupl[0]]
        area = cv2.contourArea(contour)
        if area > tooSmall:
            # cv2.drawContours(img, [contour], 0, (0,255,0),2, cv2.LINE_AA, maxLevel=1)
            cv2.drawContours(img, [contour], 0, (0, 0, 0), 2, cv2.LINE_AA, maxLevel=1)
            significant.append([contour, area])

    significant.sort(key=lambda x: x[1])
    return [x[0] for x in significant]


def segment(img):
    # Remove noise
    blurred = cv2.GaussianBlur(img, (5, 5), 0)

    # Remove noise
    shifted = cv2.pyrMeanShiftFiltering(blurred, 25, 15)

    # Edge operator
    sobel = np.max(np.array([getSobel(shifted[:, :, 0]), getSobel(shifted[:, :, 1]), getSobel(shifted[:, :, 2])]),
                   axis=0)

    # Noise reduction trick, from http://sourceforge.net/p/octave/image/ci/default/tree/inst/edge.m#l182
    mean = np.mean(sobel)

    # Zero any values less than mean. This reduces a lot of noise.
    sobel[sobel <= mean] = 0
    sobel[sobel > 255] = 255

    sobel_8u = np.asarray(sobel, np.uint8)

    # Find contours
    significant = findSignificantContours(img, sobel_8u)

    # Mask
    mask = sobel.copy()
    mask[mask > 0] = 0
    cv2.fillPoly(mask, significant, 255)
    # Invert mask
    mask = np.logical_not(mask)

    # Finally remove the background
    img[mask] = 0

    return img, sobel, shifted


def findMinColor(im):

    imout = im.copy()
    forground, sobel, shifted = segment(im)
    hsv = cv2.cvtColor(forground, cv2.COLOR_BGR2HSV)
    #hsv = cv2.cvtColor(shifted, cv2.COLOR_BGR2HSV)

    # Bin the h values
    min_white_px = 3800 #3801 for f84 test image
    color_step = 1
    color_bins = np.zeros(shape=(int(math.ceil(255 / color_step)),))
    i = 0
    for x in range(0, 255, color_step):
        lower = np.array([x, 0, 0])
        upper = np.array([x + color_step - 1, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)
        #masked_im = cv2.bitwise_and(im, im, mask=mask)
        #cv2.putText(masked_im, str(i), (230, 50), cv2.FONT_HERSHEY_DUPLEX, 0.8, (0, 255, 0))
        #cv2.imshow("masked", masked_im)
        #cv2.waitKey(0)
        _, contours, _ = cv2.findContours(mask, 1, 2)
        # find the biggest area
        n_white_px = 0
        if len(contours) != 0:
            c = max(contours, key=cv2.contourArea)
            n_white_px = cv2.contourArea(c)
        if n_white_px < min_white_px:
            color_bins[i] = 0
        else:
            color_bins[i] = n_white_px
        i = i + 1

    # Find the min bin that is not zero
    minval = np.min(color_bins[np.nonzero(color_bins)])
    idx = np.where(color_bins == minval)

    # Draw shape around min color
    offset = idx[0] * color_step
    lower = np.array([offset[0], 0, 0])
    upper = np.array([offset[0] + color_step - 1, 255, 255])
    mask = cv2.inRange(hsv, lower, upper)
    _, contours, _ = cv2.findContours(mask, 1, 2)
    c = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(c)
    cv2.rectangle(imout, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Find the v range of the min hsv bin
    min_range = [color_step * idx[0], (color_step * idx[0]) + color_step - 1]

    #sobel = cv2.resize(sobel, (1920, 1080))
    #shifted = cv2.resize(shifted, (1920, 1080))
    #gray_test = cv2.resize(gray_test, (1920, 1080))
    #forground = cv2.resize(forground, (1920, 1080))
    #imout = cv2.resize(imout, (1920, 1080))

    #cv2.imshow("Sobel", sobel)
    #cv2.imshow("Shifted", shifted)
    #cv2.imshow("hsv", hsv)
    #cv2.imshow("Forground", forground)
    #cv2.imshow("Output", imout)
    #cv2.waitKey(0)

    return min_range, imout

#im = cv2.imread("/home/sam/Desktop/f84.jpg")
#_,_, = findMinColor(im)
