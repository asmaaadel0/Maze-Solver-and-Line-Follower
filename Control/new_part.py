import numpy as np
import cv2 as cv2
import serial
import time

ser = serial.Serial("COM11", 9600, timeout = 1) #Change your port name COM... and your baudrate
def retrieveData():
     #ser.write(b'1')
     data = ser.readline().decode('ascii')
     return data
    
def detect_lines(src,dst,cdstP,new_image):
    
#     src = src[180:610, 0:790]
    linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 10)
    kernel = np.ones((35,35), np.uint8)
    
    if linesP is not None:
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)
            cv2.line(new_image, (l[0], l[1]), (l[2], l[3]), (255,255,255), 3, cv2.LINE_AA)
        
    cv2.imshow("Source", src)
    cv2.imshow("Detected Lines (in red) - Standard Hough Line Transform", cdstP)
    new_image = cv2.dilate(new_image, kernel, iterations=1)
    
    cv2.imshow("new", new_image)
    return linesP


def detection(prev,cur,src,cdstP,new_image):
#     src = src[180:610, 0:790]
    
    img_hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)    
    mask1 = cv2.inRange(img_hsv, (0,50,20), (5,255,255))
    mask2 = cv2.inRange(img_hsv, (175,50,20), (180,255,255))
    mask = cv2.bitwise_or(mask1, mask2 )
    cv2.imshow("mask", mask)
    
    
    contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    if(contours):
        contour = max(contours, key = len)
        (x_axis,y_axis),radius = cv2.minEnclosingCircle(contour)
        center = (int(x_axis),int(y_axis))
        radius = int(radius)
        
        cv2.drawContours(cdstP, [max(contours, key = len)], 0, (0,255,0),2, cv2.LINE_AA, maxLevel=1)
        cv2.circle(cdstP, (int(x_axis),int(y_axis)), radius=10, color=(255, 0, 0), thickness=10)
        if(new_image[int(y_axis)-30:int(y_axis)+30,int(x_axis)-30:int(x_axis)+30].any()):
            print('yes')
            cv2.circle(cdstP, (int(x_axis),int(y_axis)), radius=5, color=(255, 255, 255), thickness=5)
            cur='1'
            if(prev != cur):
             ser.write(b'1')
             prev='1'
             cur=prev
             print('inside if "1"')
        else:
            print('no')
            cur='0'
            if(prev != cur):
             ser.write(b'0')
             prev='0'
             cur=prev
             print('inside if "0"')
        
                
        cv2.imshow("Source", src)
        cv2.imshow("Detected Lines (in red) - Standard Hough Line Transform", cdstP)
    
    else:
        print('no')
        cur='0'
        if(prev != cur):
            ser.write(b'0')
            prev='0'
            cur=prev
            print('inside if "0"')

        
        
    return prev
    

#-----------------------------------------------------
cap = cv2.VideoCapture(1)
# address='http://192.168.1.11:8080/video'
# cap.open(address)
# cap = cv2.VideoCapture("new.mp4") 

if not cap.isOpened():
    print("Cannot open camera")
    exit()

time.sleep(2)
ret, frame = cap.read()   
ret, frame = cap.read() 
dim = (800, 800)
frame = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)
cv2.imshow("frame zft", frame)
print(frame)
dst = cv2.Canny(frame, 50, 200)    
cdstP = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
new_image=np.zeros(cdstP.shape)

lines=detect_lines(frame,dst,cdstP,new_image)
# while(not lines):
#     ret, frame = cap.read()
#     dim = (800, 800)
#     frame = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)
#     print('zft')
#     cv2.imshow("frame zft", frame)
#     lines=detect_lines(frame,dst,cdstP,new_image)

print('gggggg')
prev='-1'
cur='0'
while True:
    ret, frame = cap.read()
    
   
    dim = (800, 800)
    frame = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)

    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
        
    prev=detection(prev,cur,frame,cdstP,new_image)
    print(prev)
    if cv2.waitKey(1) == ord('q'):
        break
        
        
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()