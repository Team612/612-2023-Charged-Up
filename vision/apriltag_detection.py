import cv2
import copy
import numpy as np
import math
from pupil_apriltags import Detector


vid = cv2.VideoCapture(0)
with np.load('B.npz') as X:
    mtx, dist, rvecs, tvecs = [X[i] for i in ('camera_matrix','distortion','rotation_vectors','location_vectors')]

print(mtx)
at_detector = Detector(
    families="tag16h5", 
    nthreads=1,
    quad_sigma=0.0, 
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)
def draw_tags(
    image,
    tags
):
    for tag in tags:
        tag_family = tag.tag_family
        tag_id = tag.tag_id
        center = tag.center
        corners = tag.corners
        dm = tag.decision_margin
        hamming = tag.hamming

        center = (int(center[0]), int(center[1]))
        corner_01 = (int(corners[0][0]), int(corners[0][1]))
        corner_02 = (int(corners[1][0]), int(corners[1][1]))
        corner_03 = (int(corners[2][0]), int(corners[2][1]))
        corner_04 = (int(corners[3][0]), int(corners[3][1]))

        if(hamming == 0 and dm >= 30 and 1<=tag_id<=8):
            # Center
            cv2.circle(image, (center[0], center[1]), 5, (0, 0, 255), 2)

            #

            # Each side
            cv2.line(image, (corner_01[0], corner_01[1]),
                    (corner_02[0], corner_02[1]), (255, 0, 0), 2)
            cv2.line(image, (corner_02[0], corner_02[1]),
                    (corner_03[0], corner_03[1]), (255, 0, 0), 2)
            cv2.line(image, (corner_03[0], corner_03[1]),
                    (corner_04[0], corner_04[1]), (0, 255, 0), 2)
            cv2.line(image, (corner_04[0], corner_04[1]),
                    (corner_01[0], corner_01[1]), (0, 255, 0), 2)
            cv2.putText(image, str(tag_id), (center[0] - 10, center[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2, cv2.LINE_AA)
            
            if(tag_id==1):
                x = tag.pose_t[0][0] 
                y = tag.pose_t[1][0]
                w = tag.pose_t[2][0]
                
                cv2.putText(image, "x:" + str( round(x/w,2)) + " y:" + str(round(y/w,2)) + " z:" + str(round(w,2)),(10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,cv2.LINE_AA)
                cv2.putText(image, str(round(tag.pose_R[0][0],4)) + " " + str(round(tag.pose_R[1][0],4)) + " " + str(round(tag.pose_R[2][0],4)),(10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,cv2.LINE_AA)
                
    return image

while(True):

    #camera capture
    ret, image = vid.read()
    if not ret:
        break
    debug_image = copy.deepcopy(image)

    # start detection
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    tags = at_detector.detect(
        image,
        estimate_tag_pose=True, 
        camera_params=[mtx[0,0], mtx[1,1], mtx[0,2], mtx[1,2]],
        tag_size=0.1524,
    )

    # for tag in tags:
    #     print(tag.pose_t)
    

    #drawing
    debug_image = draw_tags(debug_image,tags)

    #key processing (ESC: end)
    key = cv2.waitKey(1)
    if key == 27: #ESC
        break
    
    #screen projection
    cv2.imshow('AprilTag', debug_image)

vid.release()
cv2.destroyAllWindows()