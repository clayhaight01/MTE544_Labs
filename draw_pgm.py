import sys
import yaml
from PIL import Image, ImageDraw

def generate_map(image_file):
    if image_file == "room1.pgm":
        start = [0, 0, 0]  # Obstacle 1
        obstacle_list = [
            (5, 5, 1),
            (3, 6, 2),
            (3, 8, 2),
            (3, 10, 2),
            (7, 5, 2),
            (9, 5, 2),
            (8, 10, 1),
            (6, 12, 1),
        ]  # [x,y,size(radius)]
    elif image_file == "room2.pgm":
        start = [0, 0, 0]  # Obstacle 2
        obstacle_list = [
            (1, 2.5, 1),
            (6, 2.5, 1),
            (11, 2.5, 1),
            (3.5, 5, 1),
            (8.5, 5, 1),
            (1, 7.5, 1),
            (6, 7.5, 1),
            (11, 7.5, 1),
        ]  # [x,y,size(radius)]
    else:
        raise ValueError("Invalid image file name. Choose 'room1.pgm' or 'room2.pgm'.")

    # Map settings
    rand_area = (-2, 15)
    resolution = 0.05
    start = [float(-start[0] + rand_area[0]), float(start[1] + rand_area[0]), 0.0]
    width = int((rand_area[1] - rand_area[0]) / resolution)
    height = width

    # Create image and draw obstacles
    image = Image.new('L', (width, height), 'white')
    draw = ImageDraw.Draw(image)

    for x, y, radius in obstacle_list:
        x = int((x - rand_area[0]) / resolution)
        y = height - int((y - rand_area[0]) / resolution)
        radius = int(radius / resolution)
        upper_left = (x - radius, y - radius)
        lower_right = (x + radius, y + radius)
        draw.ellipse([upper_left, lower_right], fill='black')

    # Save the image
    image.save(image_file)

    # Prepare and save YAML data in same format as previous labs
    yaml_data = {
        'image': image_file,
        'mode': 'trinary',
        'resolution': resolution,
        'origin': start,
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25
    }

    # Save YAML data
    yaml_file = image_file.replace('.pgm', '.yaml')
    with open(yaml_file, 'w') as file:
        yaml.dump(yaml_data, file, default_flow_style=None)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script_name.py <image_file>")
        sys.exit(1)
    
    image_file = sys.argv[1]
    generate_map(image_file)
