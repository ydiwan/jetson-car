import serial
import rclpy

class MscIf:
    def __init__(self, usb_path: str, logger=None):
        self.usb_path = usb_path
        self.logger = logger or rclpy.logging.get_logger("maestro_servo_controller")
        
        try:
            # pyserial automatically handles the raw termios flag configuration
            self.serial_port = serial.Serial(self.usb_path, timeout=1.0)
            self.logger.info(f"Maestro USB Servo Controller has successfully initialized using {self.usb_path} path.")
        except serial.SerialException as e:
            self.logger.error(f"Failed to open servo USB ({self.usb_path}): {e}")
            raise RuntimeError(f"failed to open usb path to Servo Controller: {e}")

    def __del__(self):
        """Clean up the serial port when the object is destroyed."""
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()

    def set_servo(self, servo_pin: int, servo_value: int):
        """
        Sets the position of a standard servo.
        The Pololu MSC expects target values in units of quarter-microseconds.
        """
        converted = int(servo_value * 4)
        
        # Protocol: 0x84, channel, lower 7 bits, upper 7 bits
        command = bytearray([
            0x84,
            servo_pin,
            converted & 0x7F,
            (converted >> 7) & 0x7F
        ])

        self.logger.debug(
            f"Command: {command[0]:02x} {command[1]:02x} {command[2]:02x} {command[3]:02x} "
            f"| Setting servo {servo_pin} ({servo_value} us)."
        )
        
        try:
            self.serial_port.write(command)
        except serial.SerialException as e:
            self.logger.error(f"Failed to send command: {e}")
            raise RuntimeError(f"Failed to send command: {e}")

    def set_pin(self, gpio_pin: int, value: int):
        """
        Sets a pin as a digital output high or low.
        """
        if value == 0:
            servo_value = 0
            self.logger.debug(f"Turning motor pin {gpio_pin} off")
        else:
            # Any value over 6000 will equal logical high on the servo pin
            servo_value = 7000
            self.logger.debug(f"Turning motor pin {gpio_pin} on")

        # Create bytearray command without the *4 conversion
        command = bytearray([
            0x84,
            gpio_pin,
            servo_value & 0x7F,
            (servo_value >> 7) & 0x7F
        ])

        self.logger.debug(
            f"Command: {command[0]:02x} {command[1]:02x} {command[2]:02x} {command[3]:02x} "
            f"| Setting GPIO pin {gpio_pin} to logical state {value}."
        )
        
        try:
            self.serial_port.write(command)
        except serial.SerialException as e:
            self.logger.error(f"Failed to send command: {e}")
            raise RuntimeError(f"Failed to send command: {e}")