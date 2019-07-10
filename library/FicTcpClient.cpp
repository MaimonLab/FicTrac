
#include "FicTcpClient.h"
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>

FicTcpClient::FicTcpClient(const char* ip_addr, int port)
{
  mSocket = socket(AF_INET, SOCK_STREAM, 0);
  memset(&mServerAddress, '0', sizeof(mServerAddress));
  mServerAddress.sin_family = AF_INET;
  mServerAddress.sin_port = htons(port);
  if(inet_pton(AF_INET, ip_addr, &mServerAddress.sin_addr)<=0)
    {
      printf("\nThe address did not work for the tcp client\n");
    }

}

void FicTcpClient::StartClient()
{
  if(connect(mSocket, (struct sockaddr *)&mServerAddress, sizeof(mServerAddress)) < 0)
    {
      printf("\nConnect did not work\n");
    }
}

void FicTcpClient::Send(double roll, double pitch, double yaw)
{
  double sendBytes [3] = {roll, pitch, yaw};
  send(mSocket, sendBytes, sizeof(sendBytes), 0);
}
