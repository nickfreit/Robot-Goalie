# command to start up kinect camera
# roslaunch freenect_launch freenect.launch rgb_frame_id:=camera_rgb_optical_frame depth_frame_id:=camera_depth_optical_frame

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/depth/image_raw",Image,self.callback)
    self.bgs = cv2.createBackgroundSubtractorMOG2()
    self.bgs.setHistory(150)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    except CvBridgeError as e:
      print(e)

    filter_frame, x, y = self.filterFrame(cv_image)
    if (x >= 0):
      print(str(x) + " " + str(y) + " " + str(cv_image[y][x]))
    cv2.imshow("Image window", filter_frame)
    cv2.waitKey(3)

    '''
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "passthrough"))
    except CvBridgeError as e:
      print(e)'''

  def filterFrame(self, frame):
    #ret, frame_thresh = cv2.threshold(frame, 1, 1000, cv2.THRESH_TOZERO_INV)
    frame_mask = self.bgs.apply(frame)
    ret, frame_mask = cv2.threshold(frame_mask, 50, 255, cv2.THRESH_BINARY)
    frame_mask = cv2.erode(frame_mask, None, iterations=2)
    frame_mask = cv2.erode(frame_mask, None, iterations=1)
    frame_mask, contours, hierarchy = cv2.findContours(frame_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    x = y = -1
    if len(contours) > 0:
      c = max(contours, key=cv2.contourArea)
      M = cv2.moments(c)
      center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
      (x, y) = center
      cv2.circle(frame_mask, (x, y), 5, 0, 3);
    return frame_mask, int(x), int(y)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
