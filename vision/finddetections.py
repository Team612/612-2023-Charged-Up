import numpy as np
import cv2
import glob
import os
from datetime import datetime
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob('chessboards/*.jpg')

save_location = '/Users/FILL_IN_USER_HERE_OR_REST_OF_PATH/612-2023-Charged-Up/chessboards' #change save location by copying path in your workspace
vid = cv2.VideoCapture(0)
iterate = 0
while(True):
    #camera capture
    ret1, img = vid.read()
    # converting to gray scale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Find the chess board corners
    ret2, corners = cv2.findChessboardCorners(gray, (7,6),None)
    
    if ret2 == True:
        cv2.imwrite(os.path.join(save_location, f"calib_image_{iterate}.jpeg"),img)
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)
        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (7,6), corners2,ret2)
        iterate+=1
        key = cv2.waitKey(1)
        if key == 27: #ESC
            break
    #key processing (ESC: end)
    key = cv2.waitKey(1)
    if key == 27 or iterate == 20: #ESC
        break
    
    #screen projection
    cv2.imshow('CalibCamera', img)
vid.release()
cv2.destroyAllWindows()