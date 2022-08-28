import numpy as np
import cv2

CUBOID_LENGTH = 0.4
CYLINDER_RADIUS = 0.15
CYLINDER_LABEL = 1
CUBOID_LABEL = 2
FLOOR_LABEL = 0
CYLINDER_COLOR = (255, 0, 0) # BLUE
CUBOID_COLOR = (0, 0, 255) # RED
class map:
    def __init__(self, length, width, resolution):
        '''
        length: the length of vicon space in meter
        width: the width of vicon space in meter
        resolution: the size of cells in meter
        '''
        self.length = length
        self.width = width
        self.resolution = resolution
        self.image_size_x = int(self.length/self.resolution)
        self.image_size_y = int(self.width/self.resolution)
        self.semantic_map = np.zeros((self.image_size_y, self.image_size_x, 3),dtype = np.uint8)
        self.semantic_map.fill(255)

    def world2image(self, world_x, world_y):
        '''
        world_x: x coord in meter in world
        return:
            tuple: (image_x,image_y)
        '''
        R_center_upperleft = np.array([[0, -1], [1, 0]])
        T = np.array([self.length/2, - self.width/2]).reshape(-1,1)
        upperleft_coord = R_center_upperleft @ np.array([[world_x],[world_y]]) + T
        image_x_meter = upperleft_coord.flatten()[0]
        image_y_meter = - upperleft_coord.flatten()[1]
        image_x_pixel = int(image_x_meter/self.resolution)
        image_y_pixel = int(image_y_meter/self.resolution)
        return (image_x_pixel, image_y_pixel)

    def generate_map(self,map_name):
        cv2.imwrite(map_name,self.semantic_map)

    def add_object(self, shape, location):
        '''
        location: (x,y) in world frame
        shape: "cuboid" or "cylinder"
        '''
        cylinder_radius_pixel = int(np.ceil(CYLINDER_RADIUS/self.resolution))
        cuboid_length_pixel = int(np.ceil(CUBOID_LENGTH/self.resolution))
        (center_x_pixel, center_y_pixel) = self.world2image(location[0], location[1])
        if(shape == "cuboid"):
            start_point = (center_x_pixel - int(np.ceil(cuboid_length_pixel/2)) , center_y_pixel - int(np.ceil(cuboid_length_pixel/2)))
            end_point = (center_x_pixel + int(np.ceil(cuboid_length_pixel/2)) , center_y_pixel + int(np.ceil(cuboid_length_pixel/2)))
            thickness = -1
            self.semantic_map = cv2.rectangle(self.semantic_map, start_point, end_point, CUBOID_COLOR, thickness)
        elif(shape == "cylinder"):
            thickness = -1
            center_coordinates = (center_x_pixel, center_y_pixel)
            self.semantic_map = cv2.circle(self.semantic_map, center_coordinates, cylinder_radius_pixel, CYLINDER_COLOR, thickness)
    


length = 20
width = 10
resolution = 0.01
vicon_map = map(length,width, resolution)
vicon_map.add_object("cylinder", (-3,-2.5))
vicon_map.add_object("cuboid", (3, 2.5))
vicon_map.generate_map("fake_map.png")


