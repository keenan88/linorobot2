import torch
from PIL import Image, ImageDraw
import matplotlib.pyplot as plt
import os
import cv2, sys
import numpy as np
from ultralytics import YOLO
from math import floor
from pathlib import Path
import cv2
import random


depth_image_path = '/home/slam_images/0.0/' + 'front_32FC1_22.988889445.tiff'
depth_image = cv2.imread(depth_image_path, cv2.IMREAD_UNCHANGED)

color_image_path = '/home/slam_images/0.0/' + 'front_rgb8_22.988889445.png'
color_image = Image.open(color_image_path)

# model = YOLO("yolov8n-seg.pt")
model = YOLO("yolo11n-seg.pt")

# Perform segmentation
results = model(color_image_path)


for r in results:
    img = np.copy(r.orig_img)
    img_name = Path(r.path).stem  # source image base-name

    # Iterate each object contour (multiple detections)
    for ci, c in enumerate(r):
        #  Get detection class name
        label = c.names[c.boxes.cls.tolist().pop()]



# Create binary mask
b_mask = np.zeros(img.shape[:2], np.uint8)

#  Extract contour result
contour = c.masks.xy.pop()
#  Changing the type
contour = contour.astype(np.int32)
#  Reshaping
contour = contour.reshape(-1, 1, 2)

# Draw contour onto mask
_ = cv2.drawContours(b_mask, [contour], -1, (255, 255, 255), cv2.FILLED)


# mask3ch = cv2.cvtColor(b_mask, cv2.COLOR_GRAY2BGR)

# # Isolate object with binary mask
# isolated = cv2.bitwise_and(mask3ch, img)

isolated = np.dstack([img, b_mask])

cv2.imwrite('mask_isolatd.png', isolated)

mask_depths = []
xs = []
ys = []

with Image.open("mask_isolatd.png") as im:

    draw = ImageDraw.Draw(im)

    for mask_point in results[0].masks[0].xy[0]:
        draw.point(mask_point, fill='green')

    while len(xs) < 100:
        x = random.randint(0, im.size[0] - 1)
        y = random.randint(0, im.size[1] - 1)

        r, g, b, a = im.getpixel((x, y))

        if a != 0:

            xs.append(x)
            ys.append(y)   
            mask_depths.append(depth_image[y, x]) 

            draw.point([x, y], fill='orange')

    avg_x = int(sum(xs) / len(xs))
    avg_y = int(sum(ys) / len(ys))
    draw.point([avg_y, avg_x], fill='red')

    # write to stdout
    im.save("marked.png")

avg_depth = sum(mask_depths) / len(mask_depths)

print("avg_depth:", avg_depth)


