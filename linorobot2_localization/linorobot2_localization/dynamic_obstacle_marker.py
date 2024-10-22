import yaml
from PIL import Image, ImageDraw
import csv
import numpy as np

# Function to read the YAML file
def read_yaml(yaml_file):
    with open(yaml_file, 'r') as file:
        return yaml.safe_load(file)

# Function to convert XY coordinates to pixel coordinates
def convert_coords(xy_coords, height_px, origin, resolution):
    pixel_coords = []
    for x, y in xy_coords:
        pixel_x = int((x - origin[0]) / resolution)
        pixel_y = height_px - int((y - origin[1]) / resolution)  # Map coordinates use inverted Y
        pixel_coords.append((pixel_x, pixel_y))
    return pixel_coords

# Function to draw red dots on the map
def draw_dots(image, pixel_coords, dot_radius=3):
    draw = ImageDraw.Draw(image)
    for x, y in pixel_coords:
        y_ = y
        draw.ellipse([x - dot_radius, y_ - dot_radius, x + dot_radius, y + dot_radius], fill='red')

# Function to read XY coordinates from a CSV file
def read_csv(csv_file):
    xy_coords = []
    with open(csv_file, 'r') as file:
        reader = csv.reader(file)
        next(reader) # skip header
        for row in reader:
            xy_coords.append((float(row[0]), float(row[1])))
    return xy_coords

# Main function
def main(pgm_file, yaml_file, csv_file, output_file):
    # Read the map YAML file
    map_data = read_yaml(yaml_file)
    resolution = map_data['resolution']  # Metres per pixel
    origin = map_data['origin']  # (x, y, theta) in metres, the map's origin in the world

    # Open the PGM image file
    image = Image.open(pgm_file).convert('RGB')

    height_px = image.size[1]

    # Read the XY coordinates from the CSV
    xy_coords = read_csv(csv_file)

    print(xy_coords)

    xy_coords.append(origin[0:2])

    # Convert the XY coordinates to pixel coordinates
    pixel_coords = convert_coords(xy_coords, height_px, origin, resolution)

    print(pixel_coords)

    # Draw the red dots on the map
    draw_dots(image, pixel_coords)

    # Save the resulting image
    image.save(output_file)
    print(f"Image saved as {output_file}")

if __name__ == '__main__':
    # Input files
    map_dir = '/home/humble_ws/src/linorobot2_navigation/maps/'
    pgm_file = map_dir + 'greenhouse2.pgm'         # Path to the PGM map file
    yaml_file = map_dir + 'greenhouse2.yaml'       # Path to the YAML file
    csv_file = "/home/humble_ws/src/linorobot2_navigation/slam_images/0.0/clicked_points.csv"     # Path to the CSV file with xy coordinates

    # Output file
    output_file = map_dir + 'map_with_dots.png'  # Path to save the output image

    # Run the script
    main(pgm_file, yaml_file, csv_file, output_file)
