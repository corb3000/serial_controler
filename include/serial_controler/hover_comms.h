#ifndef HOVER_COMMS_H
#define HOVER_COMMS_H


#include <cstring>
#include <string>
#include <serial/serial.h>
#include "protocol.h"

class HoverComms
{
 

public:

  void setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);
  SerialFeedback readValues();
  void setMotorValues(double joints [2]);
  bool connected() const { return serial_conn_.isOpen(); } //proberbly not used 

  std::string sendMsg(const std::string &msg_to_send, bool print_output = false);
  SerialFeedback read_msg;
  SerialCommand write_msg;
  

private:
  serial::Serial serial_conn_;  ///< Underlying serial connection
  void checkDecode();
  static const uint8_t read_lenth = 22;
  uint8_t read_buffer [read_lenth] = { };
};

#endif // HOVER_COMMS_H