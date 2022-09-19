import numpy as np
import cv2
from scipy.spatial.transform import Rotation 
# import math3d as m3d

CUBOID_LENGTH = 0.56
CYLINDER_RADIUS = 0.15
CYLINDER_LABEL = 1
CUBOID_LABEL = 2
FLOOR_LABEL = 0
ROAD_COLOR = (255,0,0) # blue
CYLINDER_COLOR = (0, 255, 0) # green
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
        self.semantic_map[:,:,0] = ROAD_COLOR[0]
        self.semantic_map[:,:,1] = ROAD_COLOR[1]
        self.semantic_map[:,:,2] = ROAD_COLOR[2]
    def world2image(self, world_x, world_y):
        '''
        world_x: x coord in meter in world
        return:
            tuple: (image_x,image_y)
        '''
        R_center_upperleft = np.array([[0, 1], [-1, 0]])
        T = np.array([self.length/2, self.width/2]).reshape(-1,1)
        upperleft_coord = R_center_upperleft @ np.array([[world_x],[-world_y]]) + T

        image_x_meter = upperleft_coord.flatten()[0]
        image_y_meter = upperleft_coord.flatten()[1]
        image_x_pixel = int(image_x_meter/self.resolution)
        image_y_pixel = int(image_y_meter/self.resolution)

        print("x: {}, y: {}".format(image_x_pixel, image_y_pixel))
        return (image_x_pixel, image_y_pixel)

    def generate_map(self,map_name):
        cv2.imwrite(map_name,self.semantic_map)

    def rotate_pts(self, pts, yaw):
        '''
        pts: (n,2) numpy array
        yaw: the yaw angle in radian
        '''
        rot = np.array([
            [np.cos(yaw), -np.sin(yaw)],
            [np.sin(yaw), np.cos(yaw)]
        ])

        rotated_pts = rot @ pts.T     # (2,n)
        rotated_pts = np.floor(rotated_pts).T

        return rotated_pts


    def add_object(self, shape, location, yaw = 0):
        '''
        location: (x,y) in world frame
        shape: "cuboid" or "cylinder"
        '''
        cylinder_radius_pixel = int(np.ceil(CYLINDER_RADIUS/self.resolution))
        cuboid_length_pixel = int(np.ceil(CUBOID_LENGTH/self.resolution))
        (center_x_pixel, center_y_pixel) = self.world2image(location[0], location[1])

        # compare
        # self.vicon_to_image_tf(location[0], location[1])
        #(center_x_pixel, center_y_pixel) = self.vicon_to_image_tf(location[0], location[1])

        if(shape == "cuboid"):
            # start_point = (center_x_pixel - int(np.ceil(cuboid_length_pixel/2)) , center_y_pixel - int(np.ceil(cuboid_length_pixel/2)))
            # end_point = (center_x_pixel + int(np.ceil(cuboid_length_pixel/2)) , center_y_pixel + int(np.ceil(cuboid_length_pixel/2)))
            # self.semantic_map = cv2.rectangle(self.semantic_map, start_point, end_point, CUBOID_COLOR, thickness)

            # upper_left = (center_x_pixel - int(np.ceil(cuboid_length_pixel/2)) , center_y_pixel - int(np.ceil(cuboid_length_pixel/2)))
            # upper_right = (center_x_pixel + int(np.ceil(cuboid_length_pixel/2)) , center_y_pixel - int(np.ceil(cuboid_length_pixel/2)))
            # bottom_left = (center_x_pixel - int(np.ceil(cuboid_length_pixel/2)) , center_y_pixel + int(np.ceil(cuboid_length_pixel/2)))
            # bottom_right = (center_x_pixel + int(np.ceil(cuboid_length_pixel/2)) , center_y_pixel + int(np.ceil(cuboid_length_pixel/2)))

            rot_rectangle = ((center_x_pixel, center_y_pixel), (cuboid_length_pixel, cuboid_length_pixel), yaw/3.1415 * 180)
            box = cv2.boxPoints(rot_rectangle) 
            box = np.int0(box) #Convert into integer values
            thickness = -1
            self.semantic_map = cv2.drawContours(self.semantic_map,[box],0,CUBOID_COLOR,thickness)
        elif(shape == "cylinder"):
            thickness = -1
            center_coordinates = (center_x_pixel, center_y_pixel)
            self.semantic_map = cv2.circle(self.semantic_map, center_coordinates, cylinder_radius_pixel, CYLINDER_COLOR, thickness)
    
# box1:
x1 = -1.4973929616871815
y1 = -1.7704407619446498
euler_1 = Rotation.from_quat( [0.002831956647539634,
  0.0023566296730900198,
  -0.017344619962709796,
  0.9998427828796284]).as_euler('zxy') 
yaw1 = euler_1[0]
# box2:
x2 =  1.1536837266153823
y2 = -4.44593942631957
euler_2 = Rotation.from_quat([-0.009430718981790567,
  0.005138266613762591,
  0.04811504494182678,
  0.9987840618501771]).as_euler('zxy')
yaw2 = euler_2[0]
# box3:
x3 = -0.35316053863590646
y3 = -6.022340390317486
euler_3 = Rotation.from_quat([-0.004470304264602129,
   0.0032587819797408567,
  -0.027708804710167366,
   0.9996007297222848]).as_euler('zxy')
yaw3 = euler_3[0]
# cylinder1:

xc_1 = -1.2546445992279214
yc_1 =  -4.076940875874319
# cylinder2:
xc_2 = 1.419320370903466
yc_2 =  -2.621207574946877
# cylinder3:
xc_3 = -0.7994720985198609
yc_3 =  -2.6000136337700672

length = 88 * 0.3
width = 22 *0.3
resolution = 0.01
vicon_map = map(length,width, resolution)
vicon_map.add_object("cylinder", (xc_1,yc_1))
vicon_map.add_object("cylinder", (xc_2,yc_2))
vicon_map.add_object("cylinder", (xc_3, yc_3))

vicon_map.add_object("cuboid", (x1, y1), yaw1)
vicon_map.add_object("cuboid", (x2, y2), yaw2)
vicon_map.add_object("cuboid", (x3, y3), yaw3)
vicon_map.generate_map("fake_map.png")

vicon_map_class1 = np.where(vicon_map.semantic_map[:,:,0] == 255, 0, 255) # road
vicon_map_class3 = np.where(vicon_map.semantic_map[:,:,2] == 255, 0, 255) # build
vicon_map_class4 = np.where(vicon_map.semantic_map[:,:,1] == 255, 0, 255) # veg

cv2.imwrite("class1.png",vicon_map_class1)
cv2.imwrite("class3.png",vicon_map_class3)
cv2.imwrite("class4.png",vicon_map_class4)