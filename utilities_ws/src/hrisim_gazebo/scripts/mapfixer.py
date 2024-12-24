import cv2

ROT_ANGLE = 92 # in degrees
SHIFT = 0.5 # in meters

# Load the map image
path = '/home/lcastri/git/PeopleFlow/utilities_ws/src/hrisim_gazebo/tiago_maps/warehouse'
map_image = cv2.imread(path+'/map.pgm', cv2.IMREAD_UNCHANGED)

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

# Calculate pixel shift for 1 meter shift
shift_pixels = int(SHIFT / 0.05)  # convert meters to pixels

# Add vertical shift (move the map up or down)
# Positive values move it down, negative values move it up
rotation_matrix[1, 2] += shift_pixels  # Applying the shift

# Perform the rotation and translation (shift)
rotated_image = cv2.warpAffine(map_image, rotation_matrix, (new_w, new_h), flags=cv2.INTER_NEAREST)

# Save the rotated map
cv2.imwrite(path + f'/map_rotated_{ROT_ANGLE}_shifted_{SHIFT}.pgm', rotated_image)
