import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import usb.core
import usb.util

class JoyNode(Node):
    def __init__(self):
        super().__init__('joy_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Logitech F710 identifyer
        self.vendor_id = 0x046d
        self.product_id = 0xc219

        self.dev = usb.core.find(idVendor=self.vendor_id, idProduct=self.product_id)

        if self.dev is None:
            self.get_logger().error("Logitech controller not found! Make sure switch is on 'D'.")
            return

        if self.dev.is_kernel_driver_active(0):
            try:
                self.dev.detach_kernel_driver(0)
                self.get_logger().info("Detached broken kernel driver.")
            except usb.core.USBError as e:
                self.get_logger().error(f"Could not detach kernel driver: {e}")

        self.dev.set_configuration()
        self.endpoint = self.dev[0][(0,0)][0]

        # State tracking
        self.analog_mode_warning_printed = False
        self.get_logger().info("USB Controller Initialized. Ready to drive!")

        # Run at 20 Hz
        self.timer = self.create_timer(0.05, self.read_usb)

    def apply_deadzone(self, value, center=128, deadzone=20):
        """Ignores slight stick drift around the center."""
        if abs(value - center) < deadzone:
            return center
        return value

    def read_usb(self):
        try:
            data = self.dev.read(self.endpoint.bEndpointAddress, self.endpoint.wMaxPacketSize, timeout=10)

            lb_pressed = (data[6] & 0x01) != 0 # Deadman
            rb_pressed = (data[6] & 0x02) != 0 # Boost

            is_analog_mode = (data[7] == 116)
            if is_analog_mode and not self.analog_mode_warning_printed:
                self.get_logger().warn("MODE BUTTON ACTIVE: Left stick is now Analog!")
                self.analog_mode_warning_printed = True
            elif not is_analog_mode and self.analog_mode_warning_printed:
                self.get_logger().info("Mode Button Disabled: Left stick is digital.")
                self.analog_mode_warning_printed = False

            throttle = 0.0
            if is_analog_mode:
                raw_y = self.apply_deadzone(data[2])
                throttle = -(raw_y - 128) / 128.0
            else:
                if data[5] == 0:
                    throttle = 1.0
                elif data[5] == 4:
                    throttle = -1.0

            raw_x = self.apply_deadzone(data[3])
            steering = -(raw_x - 128) / 128.0

            msg = Twist()

            if lb_pressed: # Must hold Deadman to move
                speed_multiplier = 2.0 if rb_pressed else 1.0

                msg.linear.x = float(throttle * speed_multiplier)
                msg.angular.z = float(steering * 0.5) # Max 0.5 radians (~28 deg)
                self.get_logger().info(f"SENDING COMMAND -> Speed: {msg.linear.x} | Steering: {msg.angular.z}")
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.0

            self.publisher_.publish(msg)

        except usb.core.USBError:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = JoyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()