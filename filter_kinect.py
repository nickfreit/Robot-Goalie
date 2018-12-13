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
from geometry_msgs.msg import Point, Pose, Quaternion
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import matplotlib.pyplot as plt
#from mpl_toolkits.mplot3d import Axes3D



class image_converter:

  def __init__(self):
    self.current_pub = rospy.Publisher("/ball_tracker/current", Pose, queue_size=1)
    self.predicted_pub = rospy.Publisher("/to_move", Pose, queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/depth/image_raw",Image,self.callback)
    self.bgs = cv2.createBackgroundSubtractorMOG2()
    self.bgs.setHistory(150)
    self.w = 640
    self.h = 480
    self.minDistance = -10
    self.scaleFactor = .0021
    self.centers = []
    self.output_file = open("output.txt", "w")
    self.plane_offset = 100
    self.max_distance = 3500
    self.frame_number = 0
    self.min_area = 800
    self.max_area = 2500
    self.white_circle_x = -1
    self.white_circle_y = -1
    self.num_points = 8
    #self.out = cv2.VideoWriter('outpy.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 30, (640, 480))

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    except CvBridgeError as e:
      print(e)

    filter_frame, x, y = self.filterFrame(cv_image)

    if (x >= 0):
      real_z = cv_image[y][x]
      if len(self.centers) == 0 or real_z < self.centers[-1][2]:
          real_x, real_y = self.pixelToReal(x, y, real_z)

          if self.frame_number > 1:
              self.centers.append((real_x, real_y, real_z))
          self.frame_number += 1
          print("Pixel " + str(x) + " " + str(y) + " " + str(real_z))
          print("Real " + str(real_x) + " " + str(real_y) + " " + str(real_z))
          #print("Robot " + str(real_z/1000) + " " + str(-real_x/1000) + " " + str((real_y + 930)/1000))

          print(self.frame_number)
      #(real_x, real_y) = self.pixelToReal(x, y, real_z)
      #print str(0.1) + " " + str(real_x/1000) + " " + str(real_y/1000)
      #self.current_pub.publish(Pose(Point(0.1, -real_x/1000, real_y/1000),
                                    #Quaternion(-0.5, 0.5, 0.5, 0.5)))
      #self.output_file.write(str(real_x) + " " + str(real_y) + " " + str(real_z) + '\n')


    if (self.frame_number == self.num_points):
      # Check that z values are decreasing

      z = []
      y = []
      x = []

      prev = self.max_distance
      for point in self.centers:
          if point[2] > prev:
              print("Error, invaild points")

          prev = point[2]
          z.append(point[2])
          y.append(point[1])
          x.append(point[0])





      x_plane, y_plane = self.predictPosition()
      z.append(0.1)
      y.append(y_plane)
      x.append(x_plane)


      xz_plot = plt.figure()
      plt.scatter(z,x)
      plt.show()

      yz_plot = plt.figure()
      plt.scatter(z,y)
      plt.show()


      self.white_circle_x = x_plane
      self.white_circle_y = y_plane
      print("predicted at ")
      print(str(x_plane) + "  " + str(y_plane))

      print Point(0.1, x_plane/1000, (y_plane+930)/1000)
      self.predicted_pub.publish(Pose(Point(0.1, x_plane/1000, (y_plane+930)/1000),
                                    Quaternion(-0.5, 0.5, 0.5, 0.5)))

    #print(x_plane, y_plane)
    cv2.circle(filter_frame, (int(self.white_circle_x), int(self.white_circle_y)), 5, 255, 3)


    cv2.imshow("Image window", filter_frame)

    #filter_frame = cv2.cvtColor(filter_frame, cv2.COLOR_GRAY2BGR)
    #self.out.write(filter_frame)

    cv2.waitKey(3)

  def filterFrame(self, frame):
    ret, frame = cv2.threshold(frame, self.max_distance, 0, cv2.THRESH_TOZERO_INV)
    ret, frame = cv2.threshold(frame, self.plane_offset, 0, cv2.THRESH_TOZERO)
    frame_mask = self.bgs.apply(frame)
    ret, frame_mask = cv2.threshold(frame_mask, 50, 255, cv2.THRESH_BINARY)
    frame_mask = cv2.erode(frame_mask, None, iterations=4)
    frame_mask = cv2.dilate(frame_mask, None, iterations=1)
    frame_mask, contours, hierarchy = cv2.findContours(frame_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    x = y = -1
    if len(contours) > 0:
      c = max(contours, key=cv2.contourArea)
      area = cv2.contourArea(c)
      #print area
      if area > self.min_area and area < self.max_area:
          M = cv2.moments(c)
          center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
          (x, y) = center
          cv2.circle(frame_mask, (x, y), 5, 0, 3);
    return frame_mask, int(x), int(y)

  def predictPosition(self):
    x = [i[0] for i in self.centers]
    y = [i[1] for i in self.centers]
    z = [i[2] for i in self.centers]
    p_xz = numpy.polyfit(z, x, 1)
    p_yz = numpy.polyfit(z, y, 2)

    x_plane = numpy.polyval(p_xz, self.plane_offset)
    y_plane = numpy.polyval(p_yz, self.plane_offset)
    return x_plane, y_plane

  def pixelToReal(self, x, y, real_z):
    real_x = -(x - self.w/2) * (real_z/10.0 + self.minDistance) * self.scaleFactor  # times 10 to convert from cm to mm
    real_y = -(y - self.h/2) * (real_z/10.0 + self.minDistance) * self.scaleFactor
    return real_x*10.0, real_y*10.0

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
