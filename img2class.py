from matplotlib import pyplot as plt
import numpy as np
from PIL import Image
import cv2

def flip_image(image_file):
    # open the original image
    original_img = Image.open(image_file)
    # Flip the original image vertically
    flipped_img = original_img.transpose(method=Image.FLIP_TOP_BOTTOM)
    flipped_img.save(image_file)

CUBOID_CLASS = 255.0
CYLINDER_CLASS = 128.0
ROAD_CLASS = 0

ROAD_BGR = [255, 0, 0]
CYLINDER_BGR = [0, 255, 0]
CUBOID_BGR = [0, 0, 255]
# read images in
image_file = "semantic_map_314.png"
image = plt.imread(image_file)
image = np.uint8(image*255)
plt.imshow(image,'gray')
plt.show()

semantic_image = np.zeros((image.shape[0], image.shape[1], 3))
semantic_image[image == CUBOID_CLASS] = CUBOID_BGR
semantic_image[image == CYLINDER_CLASS] = CYLINDER_BGR
semantic_image[image == ROAD_CLASS] = ROAD_BGR
cv2.imwrite("classImage/perch1.png", semantic_image)
# generate class images based on the channel value

cuboid_image = np.zeros((image.shape[0], image.shape[1], 3))
cuboid_image[image == CUBOID_CLASS] = 0
cuboid_image[image != CUBOID_CLASS] = 255
plt.imshow(cuboid_image)
plt.show()
cv2.imwrite("classImage/class3.png", cuboid_image)

cylinder_image = np.zeros((image.shape[0], image.shape[1], 3))
cylinder_image[image == CYLINDER_CLASS] = 0
cylinder_image[image != CYLINDER_CLASS] = 255
plt.imshow(cylinder_image)
plt.show()
cv2.imwrite("classImage/class4.png", cylinder_image)

ground_image = np.zeros((image.shape[0], image.shape[1], 3))
ground_image[image == 0] = 0
ground_image[image != 0] = 255
plt.imshow(ground_image)
plt.show()
cv2.imwrite("classImage/class1.png", ground_image)
# generate dummy image
dummy_image = np.ones((image.shape[0], image.shape[1], 3))*255
cv2.imwrite("classImage/class0.png", dummy_image)
cv2.imwrite("classImage/class2.png", dummy_image)
cv2.imwrite("classImage/class5.png", dummy_image)

flip_image("classImage/class1.png")
flip_image("classImage/class3.png")
flip_image("classImage/class4.png")