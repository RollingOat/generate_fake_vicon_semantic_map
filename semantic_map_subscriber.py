# subscribe to semantic map published by overhead camera and save it to the directories used by top down render
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class semantic_map_sub_node():
    def __init__(self):
        # Params
        self.dirToSave = ''
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(0.1)

        # Publishers
        # self.pub = rospy.Publisher('sem_map_pub', Image, queue_size=10)
        self.sub = rospy.Subscriber('sem_map_sub', Image, self.callback)

    def callback(self,data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv2.imsave(self.dirToSave, )

    def start(self):
        rospy.loginfo("Subscribe to Semantic map")
        while not rospy.is_shutdown():
            rospy.loginfo('subscribe semantic image')
            br = CvBridge()
            self.pub.publish(br.cv2_to_imgmsg(self.image, 'bgr8'))
            self.loop_rate.sleep()


if __name__ == '__main__':
    rospy.init_node("semantic_map_pub", anonymous=True)
    my_node = semantic_map_sub_node()
    my_node.start()