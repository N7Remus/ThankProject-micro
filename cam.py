import cv2
# itt a raspberry csak csinál egy képet és lementi
cam = cv2.VideoCapture(0)
img_counter = 0
ret, frame = cam.read()
img_name = "opencv_frame_{}.png".format(img_counter)
cv2.imwrite(img_name, frame)
print("{} written!".format(img_name))
img_counter += 1

cam.release()

