# publish semantic map for cross view localizer

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class semantic_map_node():
    def __init__(self):
        # Params
        self.image_filename = '/home/jiuzhou/top_down_render_ws/src/fake_semantic_map/classImage/perch1.png'
        self.image = cv2.imread(self.image_filename)
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)

        # Publishers
        self.pub = rospy.Publisher('sem_map_pub', Image, queue_size=10)

    def start(self):
        rospy.loginfo("Semantic Images Publisher")
        #rospy.spin()
        while not rospy.is_shutdown():
            rospy.loginfo('publishing image')
            br = CvBridge()
            if self.image is not None:
                self.pub.publish(br.cv2_to_imgmsg(self.image, 'bgr8'))
            self.loop_rate.sleep()


if __name__ == '__main__':
    rospy.init_node("semantic_map_pub", anonymous=True)
    my_node = semantic_map_node()
    my_node.start()