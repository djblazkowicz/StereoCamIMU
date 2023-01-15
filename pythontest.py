import cv2


cap = cv2.VideoCapture(0)
cap.set(3, 1280)
cap.set(4, 480)
cap.set(cv2.CAP_PROP_FPS, 60)

cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
#cap.set(cv2.CAP_PROP_EXPOSURE, -6)


while cap.isOpened():
    success, img = cap.read()
    #cv2.normalize(img, img, 0, 255, cv2.NORM_MINMAX)
    h, w, channels = img.shape
    #h, w = img.shape

    half = w//2

    if success:
        frameL = img[:,:half]
        frameR = img[:,half:]

    cv2.imshow('source',img)
    cv2.imshow('left',frameL)
    cv2.imshow('right',frameR)

    #print(img.shape)

    k = cv2.waitKey(1)
    if k == 27:
        cap.release()
        break
        

cv2.destroyAllWindows()
