import cv2
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)  # 0 or 1
for k in range(50):
    ret, frame = cap.read()
    print(k, ret, None if frame is None else frame.shape)
cap.release()