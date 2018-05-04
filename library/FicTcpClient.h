/*
  Used to connect fictrack to a tcp server for ball rotation
  to be delivered to another program.
*/

#ifndef FICTCPCLIENT_H_
#define FICTCPCLIENT_H_

#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>


class FicTcpClient {
public:
  
  FicTcpClient(const char* ip_addr, int port);
  ~FicTcpClient();
  void StartClient();
  void Send(double roll, double pitch, double yaw);

private:
  int mSocket;
  struct sockaddr_in mServerAddress;
};
  
#endif
