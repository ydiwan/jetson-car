import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class MaestroInterfaceNode(Node):
    def __init__(self):
        super().__init__('maestro_interface_node')
        
        self.declare_parameter('servo_usb', '/dev/ttyACM0')
        self.declare_parameter('steer_channel', 0)
        self.declare_parameter('servo_min', 1100)
        self.declare_parameter('servo_center', 1500)
        self.declare_parameter('servo_max', 1800)
        self.declare_parameter('max_steer_angle', 0.52) 

        self.usb_path = self.get_parameter('servo_usb').value
        self.steer_ch = self.get_parameter('steer_channel').value
        self.pwm_min = self.get_parameter('servo_min').value
        self.pwm_center = self.get_parameter('servo_center').value
        self.pwm_max = self.get_parameter('servo_max').value
        self.max_angle = self.get_parameter('max_steer_angle').value

        try:
            self.serial_port = serial.Serial(self.usb_path, timeout=1.0)
            self.get_logger().info(f"Connected to Maestro on {self.usb_path}")
        except serial.SerialException as e:
            self.get_logger().error(f"Hardware failure: {e}")

        self.steer_sub = self.create_subscription(
            Float32, '/vehicle/steering_angle', self.steer_callback, 10)

    def send_maestro_command(self, channel: int, target_us: int):
        target_q_us = int(target_us * 4)
        command = bytearray([0x84, channel, target_q_us & 0x7F, (target_q_us >> 7) & 0x7F])
        try:
            self.serial_port.write(command)
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def steer_callback(self, msg: Float32):
        angle_rad = max(min(msg.data, self.max_angle), -self.max_angle)
        pwm_range_half = (self.pwm_max - self.pwm_min) / 2
        pwm_target = self.pwm_center - int((angle_rad / self.max_angle) * pwm_range_half)
        self.send_maestro_command(self.steer_ch, pwm_target)

    def destroy_node(self):
        self.send_maestro_command(self.steer_ch, self.pwm_center)
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MaestroInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()