# this code runs a node that provides a custom image service: other nodes can request a stitched image from this service
# once a request is received, the node stitches 2 images (one from each camera) together and sends then to the client
# this is the fast image stitcher because it does the whole stitching process only on the first request, and reuses the parameters to stitch the following frames
# like this, feature detection and matching is only done once

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from masterproef_interfaces.srv import GetStitchedImage
import cv2
import time

class FastImageStitcherNode(Node):
    def __init__(self):
        super().__init__('fast_image_stitcher')
        self.bridge = CvBridge()

        self.cap1 = cv2.VideoCapture("/dev/video0")
        self.cap2 = cv2.VideoCapture("/dev/video4")

        self.stitcher = cv2.Stitcher_create()
        self.initialized = False
        self.target_stitched_size = None

        self.get_logger().info("Initializing fast image stitcher...")

        # Keep retrying until initialization succeeds
        while not self.initialized:
            success, msg = self.initialize_stitcher()
            if success:
                self.get_logger().info(msg)
            else:
                self.get_logger().warn(f"Initialization failed: {msg}, retrying in 2 seconds...")
                time.sleep(2)

        self.srv = self.create_service(GetStitchedImage, 'get_stitched_image', self.handle_stitch_request)
        self.get_logger().info("Fast stitcher service ready at 'get_stitched_image'")

    def grab_synchronized_frames(self):
        self.cap1.grab()
        self.cap2.grab()
        ret1, frame1 = self.cap1.retrieve()
        ret2, frame2 = self.cap2.retrieve()

        if not ret1 or not ret2:
            return None, None
        frame1 = cv2.resize(frame1, (640, 480))
        frame2 = cv2.resize(frame2, (640, 480))
        return frame1, frame2

    def initialize_stitcher(self):
        frame1, frame2 = self.grab_synchronized_frames()
        if frame1 is None or frame2 is None:
            return False, "Failed to grab frames for initialization."

        self.get_logger().info("Estimating transform for fast stitching...")
        status = self.stitcher.estimateTransform([frame1, frame2])
        if status != cv2.Stitcher_OK:
            return False, f"Transform estimation failed with status {status}"

        status, preview = self.stitcher.composePanorama([frame1, frame2])
        if status != cv2.Stitcher_OK:
            return False, f"Initial composition failed with status {status}"

        self.target_stitched_size = (preview.shape[1], preview.shape[0])
        self.initialized = True
        return True, f"Stitcher initialized. Output size: {self.target_stitched_size}"

    def handle_stitch_request(self, request, response):
        frame1, frame2 = self.grab_synchronized_frames()
        if frame1 is None or frame2 is None:
            response.success = False
            response.message = "Failed to grab synchronized frames"
            return response

        status, stitched = self.stitcher.composePanorama([frame1, frame2])
        if status != cv2.Stitcher_OK:
            response.success = False
            response.message = f"Stitching failed with code {status}"
            return response

        stitched_resized = cv2.resize(stitched, self.target_stitched_size)
        img_msg = self.bridge.cv2_to_imgmsg(stitched_resized, encoding="bgr8")

        response.image = img_msg
        response.success = True
        response.message = "Stitching succeeded"
        return response

    def destroy_node(self):
        self.cap1.release()
        self.cap2.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FastImageStitcherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

