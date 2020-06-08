/*
***********************************************************************
* tcpclient.h:
* This header file can be read by C++ compilers
*
*  by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#ifndef _TCPCLIENT_H_
#define _TCPCLIENT_H_

#include <arpa/inet.h>
#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <string>
#include "common/logging/include/easylogging++.h"

namespace ASV::common {

class tcpclient {
 public:
  tcpclient(const std::string &_ip, const std::string &_port)
      : sockfd(0), results(-1), ip_server(_ip), port(_port) {}
  virtual ~tcpclient() { close(sockfd); }

  // send and recive data from socket server
  void TransmitData(char *recv_buffer, const char *send_buffer, int recv_size,
                    int send_size) {
    int send_bytes = send(sockfd, send_buffer, send_size, 0);
    if (send_bytes == -1) {
      CLOG(ERROR, "tcp-client") << "send";
      results = 1;
    }
    int recv_bytes = recv(sockfd, recv_buffer, recv_size, 0);
    if (recv_bytes == -1) {
      CLOG(ERROR, "tcp-client") << "recv";
      results = 1;
    }
  }  // TransmitData

  void TrytoConnect() {
    if (results != 0) {
      results = connect2server();
      sleep(1);
    }
  }  // TrytoConnect

  int getsocketresults() const noexcept { return results; }

 private:
  int sockfd;
  int results;
  std::string ip_server;
  std::string port;  // the port client will be connecting to
  // get sockaddr, IPv4 or IPv6:
  void *get_in_addr(struct sockaddr *sa) {
    if (sa->sa_family == AF_INET) {
      return &(((struct sockaddr_in *)sa)->sin_addr);
    }

    return &(((struct sockaddr_in6 *)sa)->sin6_addr);
  }

  int connect2server() {
    struct addrinfo hints, *servinfo, *p;
    char s[INET6_ADDRSTRLEN];
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    int rv = getaddrinfo(ip_server.c_str(), port.c_str(), &hints, &servinfo);
    if (rv != 0) {
      CLOG(ERROR, "tcp-client") << "getaddrinfo: " << gai_strerror(rv);
      return 1;
    }
    // loop through all the results and connect to the first we can
    for (p = servinfo; p != NULL; p = p->ai_next) {
      sockfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol);
      if (sockfd == -1) {
        CLOG(ERROR, "tcp-client") << "socket error";
        continue;
      }

      if (connect(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
        CLOG(ERROR, "tcp-client") << "socket connec";
        close(sockfd);
        continue;
      }

      break;
    }

    if (p == NULL) {
      CLOG(ERROR, "tcp-client") << "fail to connect";
      freeaddrinfo(servinfo);
      return 2;
    }
    inet_ntop(p->ai_family, get_in_addr((struct sockaddr *)p->ai_addr), s,
              sizeof s);
    CLOG(ERROR, "tcp-client") << "connecting to " << std::string(s);
    freeaddrinfo(servinfo);  // all done with this structure
    return 0;
  }  // connect2server
};   // end class tcpclient

}  // namespace ASV::common

#endif /* _TCPCLIENT_H_ */
