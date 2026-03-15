import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import serial

class MaestroInterfaceNode(Node):
    def __init__(self):
        super().__init__('maestro_interface_node')
        
        # Parameters
        self.declare_parameter('servo_usb', '/dev/ttyACM0')
        self.declare_parameter('steer_channel', 0)
        self.declare_parameter('left_enable_channel', 1)  # Motor Enable L
        self.declare_parameter('right_enable_channel', 2) # Motor Enable R
        
        self.declare_parameter('servo_min', 1100)
        self.declare_parameter('servo_center', 1500)
        self.declare_parameter('servo_max', 2100)
        self.declare_parameter('max_steer_angle', 0.52) 

        self.usb_path = self.get_parameter('servo_usb').value
        self.steer_ch = self.get_parameter('steer_channel').value
        self.l_en_ch = self.get_parameter('left_enable_channel').value
        self.r_en_ch = self.get_parameter('right_enable_channel').value
        
        self.pwm_min = self.get_parameter('servo_min').value
        self.pwm_center = self.get_parameter('servo_center').value
        self.pwm_max = self.get_parameter('servo_max').value
        self.max_angle = self.get_parameter('max_steer_angle').value

        # Serial Init
        try:
            self.serial_port = serial.Serial(self.usb_path, timeout=1.0)
            self.get_logger().info(f"Connected to Maestro on {self.usb_path}")
        except serial.SerialException as e:
            self.get_logger().error(f"Hardware failure: {e}")
            raise RuntimeError(e)

        # Send Enable to motors
        self.get_logger().info("Enabling Motor Drivers on Maestro CH 1 & 2...")
        self.set_pin(self.l_en_ch, 1)
        self.set_pin(self.r_en_ch, 1)

        # Subscriptions
        self.steer_sub = self.create_subscription(
            Float32, '/vehicle/steering_angle', self.steer_callback, 10)

    def set_pin(self, channel: int, value: int):
        """Sends a logical HIGH (7000) or LOW (0) to a Maestro pin."""
        servo_value = 7000 if value != 0 else 0
        command = bytearray([0x84, channel, servo_value & 0x7F, (servo_value >> 7) & 0x7F])
        try:
            self.serial_port.write(command)
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def send_maestro_command(self, channel: int, target_us: int):
        """Standard servo pulse command."""
        target_q_us = int(target_us * 4)
        command = bytearray([0x84, channel, target_q_us & 0x7F, (target_q_us >> 7) & 0x7F])
        try:
            self.serial_port.write(command)
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def cmd_callback(self, msg: Twist):
        """Dynamic kill-switch based on target velocity."""
        if msg.linear.x == 0.0:
            self.set_pin(self.l_en_ch, 0)
            self.set_pin(self.r_en_ch, 0)
        else:
            self.set_pin(self.l_en_ch, 1)
            self.set_pin(self.r_en_ch, 1)

    def steer_callback(self, msg: Float32):
        angle_rad = max(min(msg.data, self.max_angle), -self.max_angle)
        
        # Fix steering asym
        if angle_rad > 0:
            ratio = angle_rad / self.max_angle
            pwm_target = self.pwm_center - int(ratio * (self.pwm_center - self.pwm_min))
        else:
            ratio = abs(angle_rad) / self.max_angle
            pwm_target = self.pwm_center + int(ratio * (self.pwm_max - self.pwm_center))
            
        self.send_maestro_command(self.steer_ch, pwm_target)

    def destroy_node(self):
        """Safety kill-switch on shutdown."""
        self.get_logger().info("Disabling Motor Drivers...")
        self.set_pin(self.l_en_ch, 0)
        self.set_pin(self.r_en_ch, 0)
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