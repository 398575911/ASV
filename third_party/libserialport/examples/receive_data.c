#include <libserialport.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Example of how to receive data.
 *
 * This example file is released to the public domain. */

/* Helper function for error handling. */
int check(enum sp_return result);

int main(int argc, char **argv) {
  /* This example can be used with one or two ports. With one port, it
   * will send data and try to receive it on the same port. This can be
   * done by connecting a single wire between the TX and RX pins of the
   * port.
   *
   * Alternatively it can be used with two serial ports connected to each
   * other, so that data can be sent on one and received on the other.
   * This can be done with two ports with TX/RX cross-connected, e.g. by
   * a "null modem" cable, or with a pair of interconnected virtual ports,
   * such as those created by com0com on Windows or tty0tty on Linux. */

  char *port_name = "/dev/ttyUSB0";

  /* The ports we will use. */
  struct sp_port *ports;

  /* Open and configure port. */
  printf("Looking for port %s.\n", port_name);
  check(sp_get_port_by_name(port_name, &ports));

  printf("Opening port.\n");
  check(sp_open(ports, SP_MODE_READ_WRITE));

  printf("Setting port to 115200 8N1, no flow control.\n");
  check(sp_set_baudrate(ports, 115200));
  check(sp_set_bits(ports, 8));
  check(sp_set_parity(ports, SP_PARITY_NONE));
  check(sp_set_stopbits(ports, 1));
  check(sp_set_flowcontrol(ports, SP_FLOWCONTROL_NONE));

  // Now start to receive it back.

  /* The data we will send. */
  int size = 200;

  /* We'll allow a 1 second timeout for send and receive. */
  unsigned int timeout = 100;

  /* On success, sp_blocking_write() and sp_blocking_read()
   * return the number of bytes sent/received before the
   * timeout expired. We'll store that result here. */
  int result;

  /* Allocate a buffer to receive data. */
  char *buf = malloc(size + 1);

  for (int it = 0; it != 100; ++it) {
    /* Try to receive the data on the other port. */
    result = check(sp_blocking_read(ports, buf, size, timeout));

    /* Check if we received the same data we sent. */
    buf[result] = '\0';
    printf("Received '%s'.\n", buf);
  }

  /* Free receive buffer. */
  free(buf);

  /* Close ports and free resources. */
  check(sp_close(ports));
  sp_free_port(ports);

  return 0;
}

/* Helper function for error handling. */
int check(enum sp_return result) {
  /* For this example we'll just exit on any error by calling abort(). */
  char *error_message;

  switch (result) {
    case SP_ERR_ARG:
      printf("Error: Invalid argument.\n");
      abort();
    case SP_ERR_FAIL:
      error_message = sp_last_error_message();
      printf("Error: Failed: %s\n", error_message);
      sp_free_error_message(error_message);
      abort();
    case SP_ERR_SUPP:
      printf("Error: Not supported.\n");
      abort();
    case SP_ERR_MEM:
      printf("Error: Couldn't allocate memory.\n");
      abort();
    case SP_OK:
    default:
      return result;
  }
}
