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

        # Logcount is number of times image callback ran. Not important but fun.
        self.logcount = 0
        # Lostcount was initially useless but is used here to determine the "find the line" behavior
        self.lostcount = 0

    def image_callback(self, msg):

        # get image from camera
        image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding = "bgr8")

        # filter out not masking tape (get it? __masking__ tape?)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_yellow = numpy.array([30,0,130])
        upper_yellow = numpy.array([80,70,255]) # masking tape, far test bench (bright line, red and gray carpet)

        # lower_yellow = numpy.array([30,0,110])
        # upper_yellow = numpy.array([80,60,255]) # masking tape, under the stairs (dim redder light)

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

        # If it finds something in the mask image (so it sees the color)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00']) -30 #50 IS THE DISPLACEMENT VALUE OF THE CENTROID; ADJUST THIS FROM -100 (left of the line) to 0 (directly on top of the line ) to 100 (right of the line)
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

            # Move at 0.2 M/sec
            # add a turn if the centroid is not in the center
            # Hope for the best. Lots of failure modes.
            err = cx - w/2
            self.twist.linear.x = 0.2 
            self.twist.angular.z = -float(err) / 70   # originally 100, but more aggressive tuning
            self.cmd_vel_pub.publish(self.twist)

            # "cooldown" on how long robot has not seen line
            self.lostcount = self.lostcount - 5
        # Can't see the color
        else: 
            # For "doubling back": If it can't see the line, it can turn to scan for it or turn around to go back
            # Also, the longer it sees nothing, it starts to go forward too. So it will start to "spiral" around the space for the line.
            # I think this is a good way to ensure it won't miss it.
            self.twist.linear.x = self.lostcount / 50
            self.twist.angular.z = .2
            self.cmd_vel_pub.publish(self.twist)

            # Increase the time lost
            self.lostcount = self.lostcount + 1
        cv2.imshow("image", image)
        cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()