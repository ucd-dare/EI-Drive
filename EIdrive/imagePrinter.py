import cv2
import pygame

def save_CV(image, subfolder, count):
    filepath = f"/home/junshan/imageTest/{subfolder}"
    cv2.imwrite(f"{filepath}/image{count}.jpg", image)

def save_pygame(surface, subfolder, count):
    filepath = f"/home/junshan/imageTest/{subfolder}"
    pygame.image.save(surface, f"{filepath}/image{count}.jpg")