/*********************************************************************
 * This is a c++ header to interface withe the Pololu Maestro Servo Controller.
 * The Nvidia Jetson Orin Nano pwm chip has only 8 bit resolution which is
 * not enough for servo control.
 *********************************************************************/

#ifndef MAESTRO_SERVO_CONTROLLER_INTERFACE_H
#define MAESTRO_SERVO_CONTROLLER_INTERFACE_H

#include <termios.h>  //for serial commuication options

#include <rclcpp/logging.hpp>

class msc_if {
   private:
    int fd;                  // the file descriptor for path of the usb device
    struct termios options;  // for fd options

   public:
    // msc_if();
    msc_if(const char* usb_path);
    ~msc_if();
    static const rclcpp::Logger& get_logger()  // for logging write write error
    {
        static rclcpp::Logger logger = rclcpp::get_logger("maestro_servo_controller");
        return logger;
    }

    void set_servo(const unsigned char servo_pin, const unsigned short servo_value);

    void set_pin(const unsigned char gpio_pin, int value);
};

#endif