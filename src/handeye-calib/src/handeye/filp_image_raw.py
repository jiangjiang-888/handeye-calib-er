import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

def image_callback(msg):
    try:
        # 将ROS图像消息转换为OpenCV图像格式
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # 水平翻转图像
        flipped_image = cv2.flip(cv_image, 1)  # 1表示水平翻转，0表示垂直翻转，-1表示水平和垂直同时翻转
        
        # 将翻转后的图像再次转换为ROS图像消息
        flipped_msg = bridge.cv2_to_imgmsg(flipped_image, "bgr8")

        # 发布翻转后的图像到新的话题（假设话题名为 /camera/rgb/flipped_image）
        flipped_image_publisher.publish(flipped_msg)
    except CvBridgeError as e:
        print(e)

if __name__ == '__main__':
    rospy.init_node('image_flip_node', anonymous=True)
    
    # 订阅原始图像话题
    image_subscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)
    
    # 创建一个发布器来发布翻转后的图像
    flipped_image_publisher = rospy.Publisher('/camera/rgb/flipped_image', Image, queue_size=10)
    
    rospy.spin()
