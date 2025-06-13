# this code runs a node that provides a custom image service: other nodes can request a stitched image from this service
# once a request is received, the node stitches 2 images (one from each camera) together and sends then to the client
# this is the slow image stitcher because it does the whole stitching process each time a request is made, so feature detection and matching is done each time
# the service also rescales every stitched image to the same dimensions as the first stitched image to set a level playing field

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from masterproef_interfaces.srv import GetStitchedImage
import cv2

class SlowImageStitcherNode(Node):
    def __init__(self):
        super().__init__('slow_image_stitcher')
        self.bridge = CvBridge()

        self.cap1 = cv2.VideoCapture("/dev/video0")
        self.cap2 = cv2.VideoCapture("/dev/video4")

        self.stitcher = cv2.Stitcher_create()
        self.stitcher.setPanoConfidenceThresh(0.2)

        self.first_call = True
        self.target_stitched_size = None

        self.srv = self.create_service(GetStitchedImage, 'get_stitched_image', self.handle_stitch_request)
        self.get_logger().info("Stitcher service ready at 'get_stitched_image'")

    def handle_stitch_request(self, request, response):
        ret1, frame1 = self.cap1.read()
        ret2, frame2 = self.cap2.read()

        if not ret1 or not ret2:
            response.success = False
            response.message = "Camera read failed"
            return response

        if self.first_call:
            h1, w1 = frame1.shape[:2]
            h2, w2 = frame2.shape[:2]
            self.get_logger().info(f"Frame1 dimensions: {w1}x{h1}")
            self.get_logger().info(f"Frame2 dimensions: {w2}x{h2}")

        frame1 = cv2.resize(frame1, (640, 480))
        frame2 = cv2.resize(frame2, (640, 480))

        status, stitched = self.stitcher.stitch([frame1, frame2])

        if status != cv2.Stitcher_OK:
            response.success = False
            response.message = f"Stitching failed with code {status}"
            return response

        if self.first_call:
            self.target_stitched_size = stitched.shape[1], stitched.shape[0]  # (width, height)
            self.get_logger().info(f"Initial stitched dimensions: {self.target_stitched_size[0]}x{self.target_stitched_size[1]}")
            self.first_call = False

        if self.target_stitched_size:
            stitched_resized = cv2.resize(stitched, self.target_stitched_size)
        else:
            stitched_resized = stitched  # fallback, shouldn't happen

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
    node = SlowImageStitcherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
