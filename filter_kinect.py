# command to start up kinect camera
# roslaunch freenect_launch freenect.launch rgb_frame_id:=camera_rgb_optical_frame depth_frame_id:=camera_depth_optical_frame

import collections

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy
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
    self.w = 640
    self.h = 480
    self.minDistance = -10
    self.scaleFactor = .0021
    self.centers = collections.deque(maxlen=30)
    self.output_file = open("output.txt", "w")
    #self.out = cv2.VideoWriter('outpy.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 30, (640, 480))

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    except CvBridgeError as e:
      print(e)

    filter_frame, x, y = self.filterFrame(cv_image)
    
    if (x >= 0):
      real_z = cv_image[y][x]
      self.centers.append((x, y, real_z))
      (real_x, real_y) = self.pixelToReal(x, y, real_z)
      self.output_file.write(str(real_x) + " " + str(real_y) + " " + str(real_z) + '\n')
    x_plane, y_plane = self.predictPosition()
    
    print(x_plane, y_plane)
    cv2.circle(filter_frame, (int(x_plane), int(y_plane)), 5, 255, 3)
    
    
    
      #print(str(real_x) + " " + str(real_y) + " " + str(real_z))
    #for center in self.centers:
      #cv2.circle(filter_frame, (center[0], center[1]), 5, 255, 3);
    cv2.imshow("Image window", filter_frame)
    #filter_frame = cv2.cvtColor(filter_frame, cv2.COLOR_GRAY2BGR)
    #self.out.write(filter_frame)
    cv2.waitKey(3)

  def filterFrame(self, frame):
    ret, frame = cv2.threshold(frame, 3000, 0, cv2.THRESH_TOZERO_INV)
    ret, frame = cv2.threshold(frame, 1000, 0, cv2.THRESH_TOZERO)
    frame_mask = self.bgs.apply(frame)
    ret, frame_mask = cv2.threshold(frame_mask, 50, 255, cv2.THRESH_BINARY)
    frame_mask = cv2.erode(frame_mask, None, iterations=2)
    frame_mask = cv2.dilate(frame_mask, None, iterations=1)
    frame_mask, contours, hierarchy = cv2.findContours(frame_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    x = y = -1
    if len(contours) > 0:
      c = max(contours, key=cv2.contourArea)
      M = cv2.moments(c)
      center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
      (x, y) = center
      cv2.circle(frame_mask, (x, y), 5, 0, 3);
    return frame_mask, int(x), int(y)

  def predictPosition(self):
    x = [i[0] for i in self.centers]
    y = [i[1] for i in self.centers]
    z = [i[2] for i in self.centers]
    p_xz = numpy.polyfit(z, x, 2)
    p_yz = numpy.polyfit(z, y, 2)

    x_plane = numpy.polyval(p_xz, 1000)
    y_plane = numpy.polyval(p_yz, 1000)
    return x_plane, y_plane

  def pixelToReal(self, x, y, real_z):
    real_x = (x - self.w/2) * (real_z + self.minDistance) * self.scaleFactor
    real_y = (y - self.h/2) * (real_z + self.minDistance) * self.scaleFactor
    return real_x, real_y

  def calcCurrentVelocity(self):
    print center

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
