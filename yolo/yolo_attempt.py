from PIL import Image, ImageDraw
import numpy as np
from ultralytics import YOLO
from pathlib import Path
import math, os, csv, random, cv2

def get_isolated_classes(model, color_image_path):

    # Yolov11 segmentation tutorial: https://docs.ultralytics.com/guides/isolating-segmentation-objects/
    
    result = model(color_image_path, verbose=False)[0]

    isolated_images = []

    for i, class_idx in enumerate(result.boxes.cls.tolist()):

        label = result.names[class_idx]
        conf = result.boxes.conf[i]

        if label == 'person' and conf >= 0.6:

            img = np.copy(result.orig_img)

            b_mask = np.zeros(img.shape[:2], np.uint8)
            contour = result.masks.xy[i]
            contour = contour.astype(np.int32)
            contour = contour.reshape(-1, 1, 2)
            _ = cv2.drawContours(b_mask, [contour], -1, (255, 255, 255), cv2.FILLED)
            isolated = np.dstack([img, b_mask])
            isolated_image = Image.fromarray(isolated.astype('uint8'))

            isolated_images.append(isolated_image)

    return isolated_images

def get_isolated_image_realworld_xyz(im, depth_image_path):

    depth_image = cv2.imread(depth_image_path, cv2.IMREAD_UNCHANGED)

    real_x_dist_samples = []
    real_y_dist_samples = []
    real_z_dist_samples = []

    image_xs = []
    image_ys = []

    fx = 131.03448486328125
    fy = 131.03448486328125
    cx = 150.0
    cy = 100.0

    draw = ImageDraw.Draw(im)

    while len(image_xs) < 100:
        x_img = random.randint(0, im.size[0] - 1)
        y_img = random.randint(0, im.size[1] - 1)

        _, _, _, a = im.getpixel((x_img, y_img))

        if a != 0 and not math.isinf(depth_image[y_img, x_img]):

            image_xs.append(x_img)
            image_ys.append(y_img)   

            x_real_world = (x_img - cx) * depth_image[y_img, x_img] / fx
            y_real_world = (y_img - cy) * depth_image[y_img, x_img] / fy

            real_x_dist_samples.append(x_real_world) 
            real_y_dist_samples.append(y_real_world) 
            real_z_dist_samples.append(depth_image[y_img, x_img]) 

            draw.point([x_img, y_img], fill='orange')

    # avg_x_img = int(sum(image_xs) / len(image_xs))
    # avg_y_img = int(sum(image_ys) / len(image_ys))

    # draw.point([avg_y_img, avg_x_img], fill='red')

    avg_real_x_dist = sum(real_x_dist_samples) / len(real_x_dist_samples)
    avg_real_y_dist = sum(real_y_dist_samples) / len(real_y_dist_samples)
    avg_real_z_dist = sum(real_z_dist_samples) / len(real_z_dist_samples)

    return avg_real_x_dist, avg_real_y_dist, avg_real_z_dist, im

def find_nearest_image(cam, timestamp, image_dir, image_encoding):
        image_files = [f for f in os.listdir(image_dir) if f.startswith(f"{cam}_{image_encoding}")]
        if not image_files:
            return None

        nearest_image = min(image_files, key=lambda x: abs(int(x.split('_')[2].split('.')[0]) + int(x.split('_')[2].split('.')[1]) / 1e9 - timestamp))
        return os.path.join(image_dir, nearest_image)

def read_robot_path(csv_file):
    robot_path = []
    with open(csv_file, 'r') as file:
        reader = csv.reader(file)
        next(reader)
        for row in reader:

            robot_path.append(
                {
                     't': float(row[0]),
                     'x': float(row[1]),
                     'y': float(row[2]),
                     'yaw': float(row[3]),
                }
            )
    return robot_path

slam_recording_dir = '/home/slam_images/0.0/'

robot_traj = read_robot_path(slam_recording_dir + 'transforms.csv')

obstacles_coords = []

model = YOLO("yolo11n-seg.pt")

for cam in ['front', 'left', 'rear', 'right']:

    print("Camera: ", cam)

    for point in robot_traj:    

        nearest_depth_image_path = find_nearest_image(cam, point['t'], slam_recording_dir, "32FC1")
        nearest_color_image_path = find_nearest_image(cam, point['t'], slam_recording_dir, "rgb8")

        isolated_images = get_isolated_classes(model, nearest_color_image_path)

        for i, isolated_image in enumerate(isolated_images):

            cam_to_obj_x, cam_to_obj_y, cam_to_obj_z, im = get_isolated_image_realworld_xyz(isolated_image, nearest_depth_image_path)

            img_filename = "marked/" + cam + "/" + str(point['t']) + "_" + str(i) + ".png"

            im.save(img_filename)

            # Camera frame is centered at camera, with:
            # Z pointing out from the lens
            # X pointing right
            # Y pointing down

            # Base link is at the centre of the robot, with:
            # X pointing out the front of the robot
            # Y pointing out the left of the robot
            # Z pointing upwards

            base_link_to_cam_x = 0.41
            base_link_to_cam_y = 0.247

            if cam == 'front':

                base_link_to_obj_x = base_link_to_cam_x + cam_to_obj_z
                base_link_to_obj_y = -cam_to_obj_x

            elif cam == 'left':

                base_link_to_obj_x = cam_to_obj_x
                base_link_to_obj_y = base_link_to_cam_y +  cam_to_obj_z

            elif cam == 'rear':

                base_link_to_obj_x = -(base_link_to_cam_x + cam_to_obj_z)
                base_link_to_obj_y = cam_to_obj_x

            elif cam == 'right':

                base_link_to_obj_x = -cam_to_obj_x
                base_link_to_obj_y = -(base_link_to_cam_y +  cam_to_obj_z)



            x_map_to_obj = point['x'] + math.cos(point['yaw']) * base_link_to_obj_x + math.sin(point['yaw']) * base_link_to_obj_y
            y_map_to_obj = point['y'] + math.sin(point['yaw']) * base_link_to_obj_x + math.cos(point['yaw']) * base_link_to_obj_y

            obstacles_coords.append(
                [
                    point['t'],
                    x_map_to_obj,
                    y_map_to_obj,
                    img_filename
                ]
            )
    print()

save_file = 'obstacle_coords.csv'
with open(save_file, 'w') as file:
    writer = csv.writer(file)

    writer.writerows(obstacles_coords)



