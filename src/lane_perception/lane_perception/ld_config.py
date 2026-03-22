import rclpy
from rclpy.node import Node

class LdConfig:
    def __init__(self, node: Node):
        # Declare parameters with default values
        node.declare_parameter("scale_height", 3)
        node.declare_parameter("scale_res", 0.53)
        node.declare_parameter("input_img_width", 1280)
        node.declare_parameter("steps", 3)
        node.declare_parameter("min_area", 100) # old
        # node.declare_parameter("min_area", 50)
        node.declare_parameter("max_area", 2000)
        node.declare_parameter("min_lane_width", 8)
        node.declare_parameter("max_lane_width", 20)
        node.declare_parameter("min_road_width", 200)
        node.declare_parameter("max_road_width", 500)
        node.declare_parameter("center_offset", 0.0)
        node.declare_parameter("median_window", 10)
        node.declare_parameter("median_threshold", 5.0)
        node.declare_parameter("position_conf_threshold", 0.1)

        # Bird-Eye View transformation parameters
        # node.declare_parameter("top_left_src", [174.0, 100.0])
        # node.declare_parameter("top_right_src", [496.0, 0.0])
        # node.declare_parameter("btm_left_src", [30.0, 100.0])
        # node.declare_parameter("btm_right_src", [627.0, 0.0])
        
        node.declare_parameter("top_left_src", [129.0, 13.0])
        node.declare_parameter("top_right_src", [558.0, 13.0])
        node.declare_parameter("btm_left_src", [3.0, 111.0])
        node.declare_parameter("btm_right_src", [674.0, 104.0])
        

        # Retrieve and store parameters
        self.scale_height = node.get_parameter("scale_height").value
        self.scale_res = node.get_parameter("scale_res").value
        self.rgb_img_width = node.get_parameter("input_img_width").value
        self.steps = node.get_parameter("steps").value
        self.min_area = node.get_parameter("min_area").value
        self.max_area = node.get_parameter("max_area").value
        self.min_lane_width = node.get_parameter("min_lane_width").value
        self.max_lane_width = node.get_parameter("max_lane_width").value
        self.min_road_width = node.get_parameter("min_road_width").value
        self.max_road_width = node.get_parameter("max_road_width").value
        self.center_offset = node.get_parameter("center_offset").value
        self.median_window = node.get_parameter("median_window").value
        self.median_threshold = node.get_parameter("median_threshold").value
        self.pos_conf_threshold = node.get_parameter("position_conf_threshold").value

        # Store points as tuples for OpenCV
        self.top_left_src = tuple(node.get_parameter("top_left_src").value)
        self.top_right_src = tuple(node.get_parameter("top_right_src").value)
        self.btm_left_src = tuple(node.get_parameter("btm_left_src").value)
        self.btm_right_src = tuple(node.get_parameter("btm_right_src").value)

        # Logging
        node.get_logger().info(f"The Video Input width will be: {self.rgb_img_width}")
        node.get_logger().info(f"param info: scale resolution by: {self.scale_res}")
        node.get_logger().info(f"Get the lower 1/{self.scale_height} of the image")
        node.get_logger().info(f"Min and Max for pixel area for lane will be: {self.min_area} and {self.max_area}")