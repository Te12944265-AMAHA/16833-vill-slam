#!/usr/bin/env python
import re
from urllib2 import urlopen
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import sys

if __name__ == '__main__':
    # mjpg-streamer URL
    ip = sys.argv[1]
    url = 'http://' + ip + ':8080/?action=stream'
    print(url)
    stream = urlopen(url)
        
    # Read the boundary message and discard
    stream.readline()

    clen_re = re.compile(b'Content-Length: (\d+)\\r\\n')

    im_pub = rospy.Publisher("/blaser_camera/image_color", Image, queue_size = 10)
    dark_im_pub = rospy.Publisher("/blaser_camera/image_dark", Image, queue_size = 10)
    bright_im_pub = rospy.Publisher("/blaser_camera/image_bright", Image, queue_size = 10)

    rospy.init_node("blaser_image_pub")
    bridge = CvBridge()

    # Read each frame
    while not rospy.is_shutdown():
          
        stream.readline()                    # content type
        
        try:                                 # content length
            m = clen_re.match(stream.readline()) 
            clen = int(m.group(1))
        except:
            exit(0)
        
        ts_str = stream.readline()                    # timestamp
        sec  = int(ts_str[13:-2].split('.')[0])
        nsec = int(ts_str[13:-2].split('.')[1]) * 1000

        shutter_str = stream.readline()
        #print(shutter_str)
        shutter = int(shutter_str[9])

        stream.readline()                    # empty line
       
        # Read frame into the preallocated buffer
        im_str = stream.read(clen)
        
        stream.readline() # endline
        stream.readline() # boundary
            
        # This line will need to be different when using OpenCV 2.x
        img = cv2.imdecode(np.fromstring(im_str, count=clen, dtype=np.uint8), flags=cv2.IMREAD_COLOR)
        
        im_msg = bridge.cv2_to_imgmsg(img, "bgr8")
        im_msg.header.stamp = rospy.Time(sec, nsec)
        im_msg.header.frame_id = "camera"
        im_pub.publish(im_msg)

        #print("shutter:%d" % shutter)
        if shutter == 0:
            dark_im_pub.publish(im_msg)
        elif shutter == 1:
            bright_im_pub.publish(im_msg)
        else:
            print("invalid shutter code: %d" % shutter)
