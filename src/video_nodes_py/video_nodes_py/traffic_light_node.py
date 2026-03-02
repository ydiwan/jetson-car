import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2
import numpy as np
import tensorflow as tf
from PIL import Image as PILImage

class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detector')
        self.bridge = CvBridge()
        
        # Publisher for traffic light state (True = Go, False = Stop)
        self.cmd_pub = self.create_publisher(Bool, '/traffic_light/go_signal', 10)
        
        # Subscriber for camera images
        self.sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data 
        )
        
        # Load TensorFlow Model
        MODEL_NAME = 'ssd_mobilenet_v1_coco_11_06_2017' 
        PATH_TO_CKPT = MODEL_NAME + '/frozen_inference_graph.pb'
        
        self.get_logger().info("Loading TensorFlow Model...")
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.compat.v1.GraphDef()
            with tf.io.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
                
        self.sess = tf.compat.v1.Session(graph=self.detection_graph)
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.get_logger().info("Model Loaded successfully.")

    def detect_red_and_yellow(self, img_np, Threshold=0.01):
        # Using the color detection logic from main.py
        desired_dim = (30, 90)
        img = cv2.resize(img_np, desired_dim, interpolation=cv2.INTER_LINEAR)
        img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        lower_red = np.array([0, 70, 50])
        upper_red = np.array([10, 255, 255])
        mask0 = cv2.inRange(img_hsv, lower_red, upper_red)

        lower_red1 = np.array([170, 70, 50])
        upper_red1 = np.array([180, 255, 255])
        mask1 = cv2.inRange(img_hsv, lower_red1, upper_red1)

        lower_yellow = np.array([21, 39, 64])
        upper_yellow = np.array([40, 255, 255])
        mask2 = cv2.inRange(img_hsv, lower_yellow, upper_yellow)

        mask = mask0 + mask1 + mask2
        rate = np.count_nonzero(mask) / (desired_dim[0] * desired_dim[1])
        return rate > Threshold

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            image_np_expanded = np.expand_dims(cv_image, axis=0)

            # Actual detection
            (boxes, scores, classes) = self.sess.run(
                [self.detection_boxes, self.detection_scores, self.detection_classes],
                feed_dict={self.image_tensor: image_np_expanded})

            stop_flag = False
            boxes = np.squeeze(boxes)
            scores = np.squeeze(scores)
            classes = np.squeeze(classes).astype(np.int32)
            
            im_height, im_width, _ = cv_image.shape

            for i in range(min(20, boxes.shape[0])):
                # Class 10 is traffic light
                if scores[i] > 0.5 and classes[i] == 10: 
                    ymin, xmin, ymax, xmax = tuple(boxes[i].tolist())
                    (left, right, top, bottom) = (int(xmin * im_width), int(xmax * im_width),
                                                  int(ymin * im_height), int(ymax * im_height))
                    
                    crop_img = cv_image[top:bottom, left:right]
                    
                    if crop_img.size > 0 and self.detect_red_and_yellow(crop_img):
                        stop_flag = True

            # Publish the command: True means Go, False means Stop
            msg_out = Bool()
            msg_out.data = not stop_flag
            self.cmd_pub.publish(msg_out)

        except Exception as e:
            self.get_logger().error(f"Error in traffic light detection: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()