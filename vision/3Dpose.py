import cv2
import numpy as np
import glob
with np.load('B.npz') as X:
    mtx, dist, rvecs, tvecs = [X[i] for i in ('camera_matrix','distortion','rotation_vectors','location_vectors')]

def draw(img, corners, pts1):
    corner = (int(corners[0].ravel()[0]),int(corners[0].ravel()[1]))
  
    pts1x = (int(pts1[0].ravel()[0]), int(pts1[0].ravel()[1]))
    pts1y = (int(pts1[1].ravel()[0]), int(pts1[1].ravel()[1]))
    pts1z = (int(pts1[2].ravel()[0]), int(pts1[2].ravel()[1]))


    cv2.line(img, corner, pts1x, (255,0,0), 5)
    cv2.line(img, corner, pts1y, (0,255,0), 5)
    cv2.line(img, corner, pts1z, (0,0,255), 5)
    return img

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)

for fname in glob.glob('chessboards/*.jpeg'):
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (7,6),None)
    if ret == True:
        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        # Find the rotation and translation vectors.
        retval, rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners2, mtx, dist)
        # project 3D points to image plane
        imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
        img = draw(img,corners2,imgpts)
        cv2.imshow('img',img)
        k = cv2.waitKey(0) & 0xff
cv2.destroyAllWindows()