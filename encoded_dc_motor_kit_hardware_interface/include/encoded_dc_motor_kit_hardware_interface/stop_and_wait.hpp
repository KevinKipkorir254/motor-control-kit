#ifndef STOP_AND_WAIT_HPP
#define STOP_AND_WAIT_HPP

#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <libserial/SerialPort.h>
using namespace LibSerial;

int read_data_data(uint16_t* read_output, LibSerial::SerialPort* serial_port);
int write_data_data(unsigned char *outgoing_data, LibSerial::SerialPort* serial_port);
void delay(int milliseconds);

#endif