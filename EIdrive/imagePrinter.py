import cv2
import pygame

def save_CV(image, folder, count):
    filepath = f"/home/junshan/imageTest/yolo/{folder}"
    if count%10 == 0:
        cv2.imwrite(f"{filepath}/image{(count)}.jpg", image)

def save_pygame(surface, count):
    filepath = f"/home/junshan/imageTest/pygame"
    if count % 10 == 0:
        pygame.image.save(surface, f"{filepath}/image{count}.jpg")