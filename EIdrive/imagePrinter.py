import cv2
import pygame

def save_CV(image, folder, count, ID):
    return
    filepath = f"/home/junshan/imageTest/{folder}"
    #if count%10 == 0:
    cv2.imwrite(f"{filepath}/{ID}_image{count}.jpg", image)

def save_pygame(surface, count):
    return
    filepath = f"/home/junshan/imageTest/pygame"
    #if count % 10 == 0:
    pygame.image.save(surface, f"{filepath}/image{count}.jpg")