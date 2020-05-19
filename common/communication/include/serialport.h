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

#include <libserialport.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <string>

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
    free(buf_);
    /* Close ports and free resources. */
    sp_close(s_port_);
    sp_free_port(s_port_);
  }

  void testssss() {
    for (int it = 0; it != 100; ++it) {
      /* Try to receive the data on the other port. */
      int result = sp_blocking_read(s_port_, buf_, max_bytes_, timeout_);

      /* Check if we received the same data we sent. */
      buf_[result] = '\0';
      printf("Received '%s'.\n", buf_);
    }
  }

 private:
  void initializeport() {
    /* Open and configure port. */
    sp_get_port_by_name(port_name_.c_str(), &s_port_);
    sp_open(s_port_, SP_MODE_READ_WRITE);

    sp_set_baudrate(s_port_, 115200);
    sp_set_bits(s_port_, 8);
    sp_set_parity(s_port_, SP_PARITY_NONE);
    sp_set_stopbits(s_port_, 1);
    sp_set_flowcontrol(s_port_, SP_FLOWCONTROL_NONE);

    // data buffer
    buf_ = (char *)malloc(max_bytes_ + 1);
  }  // initializeport

  const std::string port_name_;
  const int baud_rate_;
  const unsigned int timeout_;
  const int max_bytes_;

  char *buf_;
  struct sp_port *s_port_; /* The ports we will use. */

};  // end class serialport

}  // namespace ASV::common

#endif /* _SERIALPORT_H_ */
