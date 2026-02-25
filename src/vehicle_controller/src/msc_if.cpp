#include "../include/vehicle_controller/msc_if.h"

#include <fcntl.h>    //access the fd
#include <termios.h>  //for serial commuication options
#include <unistd.h>   //for closing the file descriptor

#include <cstring>
#include <rclcpp/logging.hpp>  //for logging write error
#include <stdexcept>
#include <string>

// adding
using std::runtime_error;
using std::to_string;

msc_if::msc_if(const char *usb_path) {
    this->fd = open(usb_path, O_RDWR | O_NOCTTY);
    if (fd == -1) {
        RCLCPP_ERROR(get_logger(), "Failed to open servo USB (%s)", usb_path);
        throw runtime_error("failed to open usb path to Servo Controller");
    }

    tcgetattr(this->fd, &(this->options));
    this->options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
    this->options.c_oflag &= ~(ONLCR | OCRNL);
    this->options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tcsetattr(this->fd, TCSANOW, &(this->options));

    RCLCPP_INFO(get_logger(),
                "Maestro USB Servo Controller has successfully initalized "
                "using %s path.",
                usb_path);
}

msc_if::~msc_if() { close(this->fd); }

void msc_if::set_servo(const unsigned char servo_pin, const unsigned short servo_value) {
    // MSC expects servo value in units of quarter of usecs
    unsigned short converted = servo_value * 4;
    unsigned char command[] = {0x84, servo_pin,
                               static_cast<unsigned char>(converted & 0x7F),
                               static_cast<unsigned char>(converted >> 7 & 0x7F)};

    RCLCPP_DEBUG(get_logger(), "Command: %10x %10x %10x %10x \n", command[0], command[1],
                 command[2], command[3]);
    RCLCPP_DEBUG(get_logger(), "Setting servo %d (%d us).\n", converted, servo_value);
    int bytes_send = write(this->fd, command, sizeof(command));

    if (bytes_send == -1) {
        // failed to send command
        int error = errno;
        RCLCPP_ERROR(get_logger(), "Failed to send command %s (errno += %d, fd = %d)",
                     strerror(error), error, this->fd);

        throw runtime_error("Failed to send command: %s");
    }
}


void msc_if::set_pin(const unsigned char gpio_pin, int value){
      unsigned short servo_value;  // Add this declaration

      if(value == 0){
          servo_value = 0;
          RCLCPP_DEBUG(get_logger(), "Turning motor pin %d off", gpio_pin);
      } else{
          //any value over 6000 will equal logical high on the servo pin
          servo_value = 7000;
          RCLCPP_DEBUG(get_logger(), "Turning motor pin %d on", gpio_pin);

      }

      unsigned short converted = servo_value;
      unsigned char command[] = {0x84, gpio_pin,  
                                 static_cast<unsigned char>(converted & 0x7F),
                                 static_cast<unsigned char>(converted >> 7 & 0x7F)};

    RCLCPP_DEBUG(get_logger(), "Command: %10x %10x %10x %10x \n", command[0], command[1],
                 command[2], command[3]);
    RCLCPP_DEBUG(get_logger(), "Setting servo %d (%d us).\n", converted, servo_value);
    int bytes_send = write(this->fd, command, sizeof(command));

    if (bytes_send == -1) {
        // failed to send command
        int error = errno;
        RCLCPP_ERROR(get_logger(), "Failed to send command %s (errno += %d, fd = %d)",
                     strerror(error), error, this->fd);

        throw runtime_error("Failed to send command: %s");
    }
}