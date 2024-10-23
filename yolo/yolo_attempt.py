import torch
from PIL import Image, ImageDraw
import matplotlib.pyplot as plt
import os
import cv2



depth_image_path = '/home/slam_images/0.0/' + 'front_32FC1_22.988889445.tiff'
depth_image = cv2.imread(depth_image_path, cv2.IMREAD_UNCHANGED)




# if depth_image is None:
#     print("Error: Could not load depth image.")
#     exit()

# # Print the shape of the depth image
# print(f"Depth image shape: {depth_image.shape}")

# # Example: Get depth values at specific pixel coordinates
# # Replace with your specific coordinates
# pixels = [
#     (0, 0),   # Top-left corner
#     (0, depth_image.shape[1] - 1),  # Top-right corner
#     (depth_image.shape[0] - 1, 0),  # Bottom-left corner
#     (depth_image.shape[0] - 1, depth_image.shape[1] - 1),  # Bottom-right corner
#     (depth_image.shape[0] // 2, depth_image.shape[1] // 2)  # Center pixel
# ]

# # Access and print the depth values at the specified pixel locations
# for pixel in pixels:
#     y, x = pixel
#     depth_value = depth_image[y, x]
#     print(f"Depth at pixel ({x}, {y}): {depth_value} meters")

color_image_path = '/home/slam_images/0.0/' + 'front_rgb8_22.988889445.png'
color_image = Image.open(color_image_path)

model = torch.hub.load('ultralytics/yolov5', 'yolov5s')

# Perform object detection
results = model(color_image)

# Extract bounding boxes, labels, and confidence scores
detections = results.xyxy[0].numpy()  # Bounding boxes and other details

# Draw bounding boxes on the original image
draw = ImageDraw.Draw(color_image)

# Create a directory to save cropped images
output_dir = 'cropped_humans'
os.makedirs(output_dir, exist_ok=True)

human_count = 0

for det in detections:
    x1, y1, x2, y2, confidence, class_id = det
    label = results.names[int(class_id)]

    if label == 'person':  # Filter for humans only
        # Draw the rectangle on the original image
        draw.rectangle([x1, y1, x2, y2], outline="red", width=3)
        draw.text((x1, y1), f'{label} {confidence:.2f}', fill="red")

        # Crop the image based on the bounding box
        cropped_image = color_image.crop((x1, y1, x2, y2))

        # Save the cropped image with a unique filename
        cropped_image_path = os.path.join(output_dir, f'human_{human_count}.png')
        cropped_image.save(cropped_image_path)
        
        print(f"Saved cropped image: {cropped_image_path}")
        human_count += 1

# Show the original image with all bounding boxes
plt.imshow(color_image)
plt.axis('off')
plt.show()

# Optionally save the original image with bounding boxes
color_image.save('human_detected_image.png')
