import numpy as np
import cv2
import cv2.aruco as aruco
import time

def convert_corn(arr):
    '''
    Takes arbitrary orders of coordinates in:
    returns TL, TR, BR, BL according to pixel values where (0,0) = top left
    and positive y is down, and positive x is to the right 
    '''
    a = arr[0][:]
    norms = np.zeros(len(a))
    for i in range(len(a)):
        norms[i] = np.linalg.norm(a[i])
    bigi = np.where(norms == np.max(norms))
    smalli = np.where(norms == np.min(norms))
    norms[bigi] = -1
    norms[smalli] = -1
    
    randi = np.where(norms == max(norms))
    norms[randi] = -1
    randi2 = np.where(norms == max(norms))
    c1 = a[smalli] # (smallest, smallest) -> top left pixel coordinate
    c3 = a[bigi] # (biggest, biggest)   -> bottom right pixel coordinate
    c2 = a[randi]
    c4 = a[randi2]
    if(c2[0][0] < c4[0][0]):
        temp = c2[:]
        c2 = c4[:]
        c4 = temp
    
    return(np.array([c1, c2, c3, c4]).reshape(1,4,2))

def get_corners(showVid, nframes):

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    real_corners = np.array([[[[58.25,  -33.79],
       [58.25,  -37.79],
       [62.25,  -37.79],
       [62.25, -33.79]]],


     [[[58.25,  22.8],
       [58.25,  18.8],
       [62.25,  18.8],
       [62.25,  22.8]]],


     [[[11.25,  22.8],
       [11.25,  18.8],
       [15.25,  18.8],
       [15.25,  22.8]]],


     [[[11.25,  -33.79],
       [11.25,  -37.79],
       [15.25,  -37.79],
       [15.25,  -33.79]]]]) 

    avg_corners = np.zeros((4, 1, 4, 2))
    N = nframes
    realN = 0
    for i in range(N):
        ret, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(3) 
        parameters = aruco.DetectorParameters_create()
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
        if(0 in ids and 1 in ids and 2 in ids and 3 in ids and len(corners) == 4):
            if (ids[0][0] == 2 and ids[1][0] == 3 and ids[2][0] == 0 and ids[3][0] == 1):
                realN += 1
                for j in range(len(ids)):
                    newcorn = convert_corn(corners[j])
                    avg_corners[j] = avg_corners[j] + newcorn
                frame = aruco.drawDetectedMarkers(frame,corners, ids)
        if(showVid):
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    avg_corners = avg_corners / realN
    avg_corners = avg_corners.astype(int)
    return(avg_corners.reshape(1,16,2), real_corners.reshape(1,16,2) / 100)


if __name__ == "__main__":
    print(get_corners(True, 1000))
