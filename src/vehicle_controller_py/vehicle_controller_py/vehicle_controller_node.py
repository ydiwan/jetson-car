import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import Int32, Float64, String

# Import our Vehicle class
from .vehicle import Vehicle

class VehicleControllerNode(Node):
    def __init__(self):
        super().__init__('vehicle_controller')
        
        # 1. Declare and Get Parameters
        self.declare_parameter("vicon_kp", 25.0)
        self.declare_parameter("ld_kp", 5.0)
        self.declare_parameter("alpha", 0.95)
        self.declare_parameter("servo_min", 1100)
        self.declare_parameter("servo_max", 1800)
        self.declare_parameter("servo_usb", "/dev/ttyACM0")

        vicon_kp = self.get_parameter("vicon_kp").value
        ld_kp = self.get_parameter("ld_kp").value
        alpha = self.get_parameter("alpha").value
        servo_min = self.get_parameter("servo_min").value
        servo_max = self.get_parameter("servo_max").value
        servo_path = self.get_parameter("servo_usb").value

        # 2. Setup Publishers (for the GPIO node)
        self.l_pwm_pub = self.create_publisher(Int32, "gpio/pwm_left", qos_profile_sensor_data)
        self.r_pwm_pub = self.create_publisher(Int32, "gpio/pwm_right", qos_profile_sensor_data)

        # 3. Instantiate the Vehicle Brain
        self.car = Vehicle(
            vicon_kp, ld_kp, alpha, 
            servo_min, servo_max, servo_path, 
            self.r_pwm_pub, self.l_pwm_pub
        )
        
        # Startup state
        self.car.update_speed_manual(800, 800)
        self.car.motor_on(True)

        # 4. Setup Subscriptions
        self.vicon_sub = self.create_subscription(PoseStamped, "vicon_pose", self.vicon_callback, qos_profile_sensor_data)
        self.waypoints_sub = self.create_subscription(PoseArray, "waypoints", self.waypoints_callback, qos_profile_sensor_data)
        
        self.ld_delta_sub = self.create_subscription(Int32, "lane_detect/delta", self.ld_delta_callback, qos_profile_sensor_data)
        self.ld_pos_conf_sub = self.create_subscription(Float64, "/lane_detect/position_confidence", self.ld_pos_conf_callback, qos_profile_sensor_data)
        self.sym_conf_sub = self.create_subscription(Float64, "/lane_detect/symmetry_confidence", self.ld_sym_conf_callback, qos_profile_sensor_data)
        self.tl_sub = self.create_subscription(String, '/traffic_light_state', self.tl_callback, qos_profile_sensor_data)
        
        # 5. State Variables
        self.ld_delta = 0
        self.waypoints = []
        self.current_target_idx = 0
        self.can_go = True

        # 6. Control Loop Timer (100Hz / 10ms)
        self.timer = self.create_timer(0.01, self.control_loop)

    def destroy_node(self):
        self.get_logger().info("Vehicle Controller Node is shutting down")
        self.car.update_steering(1500)
        self.car.motor_on(False)
        super().destroy_node()

    # ============== Callbacks ===================================================
    def vicon_callback(self, msg: PoseStamped):
        self.car.vicon_update_pose(msg.pose)

    def waypoints_callback(self, msg: PoseArray):
        self.waypoints = []
        for pose in msg.poses:
            self.waypoints.append((pose.position.x, pose.position.y))

    def ld_delta_callback(self, msg: Int32):
        self.ld_delta = msg.data

    def ld_pos_conf_callback(self, msg: Float64):
        self.car.set_pos_conf(msg.data)

    def ld_sym_conf_callback(self, msg: Float64):
        self.car.set_sym_conf(msg.data)
        
    def tl_callback(self, msg: String):
        if msg.data == "RED":
            self.can_go = False
        else:
            self.can_go = True

    # ============== Main Loop ===================================================
    def control_loop(self):
        
        if not self.can_go:
            self.get_logger().info("Red/Yellow Light Detected. Stopping vehicle.")
            self.car.motor_on(False)
            return
        
        if not self.waypoints or self.current_target_idx >= len(self.waypoints):
            self.get_logger().debug("Reached all waypoints, YAY!")
            self.car.update_steering(1500)  # center the steering
            self.car.motor_on(False)        # turn off the motors
            return

        target_x = self.waypoints[self.current_target_idx][0]
        target_y = self.waypoints[self.current_target_idx][1]

        # calculate distance to target
        distance = self.car.vicon_cal_dis(target_x, target_y)

        # check if close enough to target
        if distance <= 100:
            self.current_target_idx += 1
            if self.current_target_idx < len(self.waypoints):
                target_x = self.waypoints[self.current_target_idx][0]
                target_y = self.waypoints[self.current_target_idx][1]
            else:
                return # Handled on the next loop iteration

        self.get_logger().debug(f"Current target; ({target_x:.2f}, {target_y:.2f})")
        self.car.motor_on(True)  # make sure the motor is on =)
        
        # Step the fusion controller
        self.car.control_step((target_x, target_y), self.ld_delta)

def main(args=None):
    rclpy.init(args=args)
    node = VehicleControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()