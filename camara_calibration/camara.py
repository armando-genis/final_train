import cv2

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))

if not cap.isOpened():
    print("Error: Camera could not be opened.")
else:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        cv2.imshow("Live feed", frame)

        key = cv2.waitKey(25)
        if key & 0xFF == ord('q'):  # press q to break
            break
        elif key & 0xFF == ord('s'):  # press s to save
            number = cv2.getTickCount()
            cv2.imwrite("img_" + str(number) + ".png", frame)
            print("Image saved")

    cap.release()
    cv2.destroyAllWindows()
