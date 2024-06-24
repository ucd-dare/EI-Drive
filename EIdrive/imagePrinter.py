import cv2

count = 0

def save_image(image, subfolder, count):
    filepath = f"/home/junshan/imageTest/{subfolder}"
    #print(f"printing image {count}")
    #cv2.imwrite(f"{filepath}/image{count}.jpg", image)