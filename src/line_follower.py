#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('rafael/raspicam_node/image/compressed', CompressedImage, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('rafael/cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        self.logcount = 0
        self.lostcount = 0

    def image_callback(self, msg):

        # get image from camera
        image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding = "bgr8")

        # filter out everything that's not yellow
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # lower_yellow = numpy.array([30,0,130])
        # upper_yellow = numpy.array([80,70,255]) # masking tape, far test bench

        lower_yellow = numpy.array([30,0,110])
        upper_yellow = numpy.array([80,60,255]) # masking tape, under the stairs (redder light)

        mask = cv2.inRange(hsv,  lower_yellow, upper_yellow)
        masked = cv2.bitwise_and(image, image, mask=mask)

    # clear all but a 20 pixel band near the top of the image
        h, w, d = image.shape
        search_top = int(3 * h /4)
        search_bot = search_top + 30
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        cv2.imshow("band", mask)

    # Compute the "centroid" and display a red circle to denote it
        M = cv2.moments(mask)
        self.logcount += 1
        print("M00 %d %d" % (M['m00'], self.logcount))

        if M['m00'] > 0:
            cx = int(M['m10']/M['m00']) -30 #50 IS THE DISPLACEMENT VALUE OF THE CENTROID; ADJUST THIS FROM -100 (left of the line) to 0 (directly on top of the line ) to 100 (right of the line)
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

            # Move at 0.2 M/sec
            # add a turn if the centroid is not in the center
            # Hope for the best. Lots of failure modes.
            err = cx - w/2
            self.twist.linear.x = 0.2 
            self.twist.angular.z = -float(err) / 100
            self.cmd_vel_pub.publish(self.twist)
        cv2.imshow("image", image)
        cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()