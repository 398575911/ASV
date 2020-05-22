/*
***********************************************************************
* serialport.h:
* handle serial ports (RS-232/422/485)
* This header file can be read by C++ compilers
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _SERIALPORT_H_
#define _SERIALPORT_H_

#include "common/logging/include/easylogging++.h"

#include <libserialport.h>
#include <string>

#include <unistd.h>

namespace ASV::common {

class serialport {
 public:
  explicit serialport(const std::string &port_name, const int baudrate,
                      const unsigned int timeout)
      : port_name_(port_name),
        baud_rate_(baudrate),
        timeout_(timeout),
        max_bytes_(300) {
    initializeport();
  }
  ~serialport() {
    /* Close ports and free resources. */
    sp_close(s_port_);
    sp_free_port(s_port_);
  }

  void list_ports() {}  // list_ports

  // std::string readline() {
  //   // data buffer
  //   char *buf = (char *)malloc(max_bytes_ + 1);
  //   /* Try to receive the data on the other port. */
  //   int result = check(sp_blocking_read(s_port_, buf, max_bytes_, timeout_));

  //   /* Check if we received the same data we sent. */
  //   buf[result] = '\0';

  //   // convert char* to string
  //   std::string str_read(buf);
  //   free(buf);

  //   return str_read;
  // }  // readline

  void readline(char *buf, int read_bytes) {
    /* Try to receive the data on the other port. */
    int result = check(sp_blocking_read(s_port_, buf, read_bytes, timeout_));

    /* Check if we received the same data we sent. */
    // buf[result] = '\0';

  }  // readline

  std::string readline(std::size_t max_bytes = 1000,
                       const std::string &eol = "\n") {
    std::size_t len_eol = eol.length();

    // data buffer
    std::string t_buffer;
    std::size_t read_so_far = 0;
    while (1) {
      char buf[1];

      /* Try to receive the data on port with timeout = 1, max_byte = 1 */
      std::size_t result =
          static_cast<std::size_t>(sp_blocking_read(s_port_, buf, 1, timeout_));

      t_buffer += std::string(buf);
      read_so_far += result;

      if (result == 0) {
        break;  // Timeout occured on reading 1 byte
      }
      if ((read_so_far >= len_eol) &&
          (t_buffer.substr(read_so_far - len_eol, std::string::npos) == eol)) {
        break;  // EOL found
      }
      if (read_so_far == max_bytes) {
        break;  // Reached the maximum read length
      }
    }

    return t_buffer;
  }  // readline

  // send data to serial port
  void writeline(const std::string &send_buffer) {
    writeline(send_buffer.c_str());

  }  // writeline

  // send data to serial port
  void writeline(const char *send_buffer) {
    // data buffer
    int size = sizeof(send_buffer) / sizeof(*send_buffer);
    /* Try to receive the data on the other port. */
    // int result = check(sp_blocking_write(s_port_, send_buffer, size,
    // timeout_));

    /* Check whether we sent all of the data. */
    if (check(sp_blocking_write(s_port_, send_buffer, size, timeout_)) != size)
      CLOG(ERROR, "serialport") << "Timed out in writing port";

  }  // writeline

  // send data to serial port
  void writeline(const unsigned char *send_buffer) {
    // data buffer
    int size = sizeof(send_buffer) / sizeof(*send_buffer);
    /* Try to receive the data on the other port. */
    // int result = check(sp_blocking_write(s_port_, send_buffer, size,
    // timeout_));

    /* Check whether we sent all of the data. */
    if (check(sp_blocking_write(s_port_, send_buffer, size, timeout_)) != size)
      CLOG(ERROR, "serialport") << "Timed out in writing port";

  }  // writeline

 private:
  void initializeport() {
    /* Open and configure port. */
    sp_get_port_by_name(port_name_.c_str(), &s_port_);
    sp_open(s_port_, SP_MODE_READ_WRITE);

    sp_set_baudrate(s_port_, baud_rate_);
    sp_set_bits(s_port_, 8);
    sp_set_parity(s_port_, SP_PARITY_NONE);
    sp_set_stopbits(s_port_, 1);
    sp_set_flowcontrol(s_port_, SP_FLOWCONTROL_NONE);

  }  // initializeport

  /* Helper function for error handling. */
  int check(enum sp_return result) {
    switch (result) {
      case SP_ERR_ARG:
        CLOG(ERROR, "serialport") << "Invalid argument";
        return 0;
      case SP_ERR_FAIL:
        CLOG(ERROR, "serialport") << "Failed";
        return 0;
      case SP_ERR_SUPP:
        CLOG(ERROR, "serialport") << "Not supported.";
        return 0;
      case SP_ERR_MEM:
        CLOG(ERROR, "serialport") << "Couldn't allocate memory.";
        return 0;
      case SP_OK:
      default:
        return result;
    }
  }  // check

  const std::string port_name_;
  const int baud_rate_;
  const unsigned int timeout_;
  const int max_bytes_;

  struct sp_port *s_port_; /* The ports we will use. */

};  // end class serialport

}  // namespace ASV::common

#endif /* _SERIALPORT_H_ */
