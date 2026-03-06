import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import serial
import cv2
import numpy as np
import struct

class OpenMVNode(Node):
    def __init__(self):
        super().__init__('openmv_node')
        
        
        self.declare_parameter("port", "/dev/ttyACM2") 
        port = self.get_parameter("port").value

        self.publisher_ = self.create_publisher(Image, '/driver_perspective/image_raw', 10)
        self.bridge = CvBridge()
        
        try:
            self.serial_port = serial.Serial(port, 115200, timeout=1)
            self.get_logger().info(f"Connected to OpenMV on {port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to OpenMV: {e}")
            self.serial_port = None
            
        self.timer = self.create_timer(0.01, self.read_serial_stream)

    def read_serial_stream(self):
        if self.serial_port is None:
            return

        while self.serial_port.in_waiting >= 4:
            
            # Wait for the Start Byte (0xAA)
            if self.serial_port.read(1) == b'\xAA':
                
                # Confirm the Second Confirm Byte (0x55)
                if self.serial_port.read(1) == b'\x55':
                    
                    # Read the 2-byte size of the incoming JPEG
                    size_bytes = self.serial_port.read(2)
                    if len(size_bytes) == 2:
                        size = struct.unpack("<H", size_bytes)[0]
                        
                        # Read the exact number of bytes for the image
                        img_data = self.serial_port.read(size)
                        
                        if len(img_data) == size:
                            # 5. Decode the raw bytes back into an OpenCV Image
                            np_arr = np.frombuffer(img_data, np.uint8)
                            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                            
                            if frame is not None:
                                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                                self.publisher_.publish(msg)
                                
                                # Flush the buffer to prevent delays
                                self.serial_port.reset_input_buffer()
                                return

def main(args=None):
    rclpy.init(args=args)
    node = OpenMVNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()