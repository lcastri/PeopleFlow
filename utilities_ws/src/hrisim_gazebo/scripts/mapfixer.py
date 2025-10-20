import cv2

ROT_ANGLE = 0  # in degrees
SHIFT_X = -2  # horizontal shift in meters
SHIFT_Y = 0.5  # vertical shift in meters

# Load the map image
path = '/home/lcastri/git/PeopleFlow/utilities_ws/src/hrisim_gazebo/tiago_maps/warehouse'
map_image = cv2.imread(path + '/map.pgm', cv2.IMREAD_UNCHANGED)

# Get the dimensions of the image
(h, w) = map_image.shape[:2]

# Define the rotation matrix
center = (w // 2, h // 2)
rotation_matrix = cv2.getRotationMatrix2D(center, ROT_ANGLE, 1.0)

# Calculate new image size to avoid cropping
cos = abs(rotation_matrix[0, 0])
sin = abs(rotation_matrix[0, 1])
new_w = int((h * sin) + (w * cos))
new_h = int((h * cos) + (w * sin))

# Adjust the rotation matrix to account for the translation (rotation shifting)
rotation_matrix[0, 2] += (new_w / 2) - center[0]
rotation_matrix[1, 2] += (new_h / 2) - center[1]

# Calculate pixel shifts for horizontal and vertical shifts
pixel_shift_x = int(SHIFT_X / 0.05)  # Horizontal shift (meters to pixels)
pixel_shift_y = int(SHIFT_Y / 0.05)  # Vertical shift (meters to pixels)

# Add horizontal and vertical shifts
rotation_matrix[0, 2] += pixel_shift_x  # Horizontal shift
rotation_matrix[1, 2] += pixel_shift_y  # Vertical shift

# Perform the rotation and translation (shift)
rotated_image = cv2.warpAffine(map_image, rotation_matrix, (new_w, new_h), flags=cv2.INTER_NEAREST)

# Save the rotated and shifted map
cv2.imwrite(path + f'/map_rotated_{ROT_ANGLE}_shifted_{SHIFT_X}_{SHIFT_Y}.pgm', rotated_image)

