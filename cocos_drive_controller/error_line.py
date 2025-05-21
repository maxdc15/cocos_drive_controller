#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import cv2 as cv
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class LineErrorNode(Node):
    def __init__(self):
        super().__init__('line_error_node')
        self.get_logger().info('Line error node initialized')

        # Parameters
        self.declare_parameter('thresh',	60)
        self.declare_parameter('roi_height',	0.25)
        self.declare_parameter('roi_width', 	0.5)
        self.declare_parameter('rate',		30.0)

        self.thresh	= self.get_parameter('thresh').value
        self.roi_height	= self.get_parameter('roi_height').value
        self.roi_width	= self.get_parameter('roi_width').value

        self.pub_err = self.create_publisher(Float32, '/line_error', 10)
        self.bridge = CvBridge()

        self.sub_img = self.create_subscription(Image, '/video_source/raw', self.image_callback, 10)
        
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error al convertir imagen: {e}")
            return

        # Height and width of the image
        h, w = frame.shape[:2]
        
        # Vertical ROI
        roi_h = frame[int(h * (1 - self.roi_height)) : h, :]
        
        # Horizontal ROI
        roi_w = int(w * self.roi_width)
        start_x = int((w - roi_w) // 2)
        end_x = start_x + roi_w
        
        # Crop the image
        roi = roi_h[:, start_x:end_x]

        # Convert to grayscale and apply binary threshold
        gray = cv.cvtColor(roi, cv.COLOR_BGR2GRAY)
        _, bin_img = cv.threshold(gray, 0, 255, cv.THRESH_BINARY_INV + cv.THRESH_OTSU)

        contours, _ = cv.findContours(bin_img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if not contours:
            return

        biggest = max(contours, key=cv.contourArea)
        M = cv.moments(biggest)
        if M['m00'] == 0:
            return

        cx = int(M['m10'] / M['m00']) # X coordinate of the centroid
        w_roi = roi.shape[1]
        error_norm = (cx - w_roi // 2) / (w_roi // 2)

        msg_out = Float32()
        msg_out.data = float(error_norm)
        self.pub_err.publish(msg_out)

        #DEBUG 
        #cv.circle(roi, (cx, int(M['m01']/M['m00'])), 4, (0,255,0), -1)
	    #cv.putText(roi, f"err: {error_norm:+.2f}", (10,20), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)
        #cv.imshow("ROI", roi)
	    #cv.waitKey(1)

    def destroy_node(self):
        cv.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = LineErrorNode()
    try: rclpy.spin(node)
    except Exception as error: print(error)
    except KeyboardInterrupt: print("\nNode terminated by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
