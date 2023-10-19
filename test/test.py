import cv2
cam = cv2.VideoCapture(0)
while True:
    ret, img = cam.read(0)
    #detect a 3d gold cube and draw a rectangle around it
    


    cv2.imshow("Test", img)
    cv2.waitKey(1)
cam.release()
cv2.destroyAllWindows()