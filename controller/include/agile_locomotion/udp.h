// /************************************************************************
// Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
// Use of this source code is governed by the MPL-2.0 license, see LICENSE.
// ************************************************************************/

// #ifndef _UNITREE_LEGGED_UDP_H_
// #define _UNITREE_LEGGED_UDP_H_

// #include "comm.h"
// #include "quadruped.h"
// #include <pthread.h>

// namespace UNITREE_LEGGED_SDK
// {

// constexpr int UDP_CLIENT_PORT = 8080;                       // local port
// constexpr int UDP_SERVER_PORT = 8007;                       // target port
// constexpr char UDP_SERVER_IP_BASIC[] = "192.168.123.10";    // target IP address
// constexpr char UDP_SERVER_IP_SPORT[] = "192.168.123.161";   // target IP address

// // Notice: User defined data(like struct) should add crc(4Byte) at the end.
// class UDP {
// public:
//     UDP(uint8_t level, HighLevelType highControl = HighLevelType::Basic);  // unitree dafault IP and Port
//     UDP(uint16_t localPort, const char* targetIP, uint16_t targetPort, int sendLength, int recvLength);
//     UDP(uint16_t localPort, uint16_t targetPort, int sendLength, int recvLength); // as server, client IP can change
//     ~UDP();
//     void InitCmdData(HighCmd& cmd);
//     void InitCmdData(LowCmd& cmd);
//     void switchLevel(int level);

//     int SetSend(HighCmd&);
//     int SetSend(LowCmd&);
//     int SetSend(char* cmd);
//     void GetRecv(HighState&);
//     void GetRecv(LowState&);
//     void GetRecv(char*);
//     int Send();
//     int Recv(); // directly save in buffer
    
//     UDPState udpState;
//     char*    targetIP;
//     uint16_t targetPort;
//     char*    localIP;
//     uint16_t localPort;
// private:
//     void init(uint16_t localPort, const char* targetIP, uint16_t targetPort);
    
//     uint8_t levelFlag = HIGHLEVEL;   // default: high level
//     int sockFd;
//     bool connected; // udp only works when connected
//     int sendLength;
//     int recvLength;
//     char* recvTemp;
//     char* recvBuf;
//     char* sendBuf;
//     int lose_recv;
//     pthread_mutex_t sendMut;
//     pthread_mutex_t recvMut;
// };

// }

// #endif
/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#ifndef _UNITREE_LEGGED_UDP_H_
#define _UNITREE_LEGGED_UDP_H_

#include "comm.h"
#include "quadruped.h"
#include <pthread.h>
#include <stdint.h>

/*
    UDP critical configuration:

    1. initiativeDisconnect: if need disconnection after connected, another ip/port can access after disconnection

                 /--- block             will block till data come
    2. recvType  ---- block + timeout   will block till data come or timeout
                 \--- non block         if no data will return immediately

                  /--- Y  ip/port will be set later
    3. setIpPort:
                  \--- N  ip/port not specified, as a server wait for connect
*/

namespace UNITREE_LEGGED_SDK {

constexpr int UDP_CLIENT_PORT = 8080;                      // local port
constexpr int UDP_SERVER_PORT = 8007;                      // target port
constexpr char UDP_SERVER_IP_BASIC[] = "192.168.123.10";   // target IP address
constexpr char UDP_SERVER_IP_SPORT[] = "192.168.123.161";  // target IP address

typedef enum {
  nonBlock = 0x00,
  block = 0x01,
  blockTimeout = 0x02,
} RecvEnum;

// Notice: User defined data(like struct) should add crc(4Byte) at the end.
class UDP {
 public:
  UDP(uint8_t level, uint16_t localPort, const char* targetIP, uint16_t targetPort);  // udp use dafault length according to level
  UDP(uint16_t localPort, const char* targetIP, uint16_t targetPort,
      int sendLength, int recvLength, bool initiativeDisconnect = false, RecvEnum recvType = RecvEnum::nonBlock);
  UDP(uint16_t localPort,
      int sendLength, int recvLength, bool initiativeDisconnect = false, RecvEnum recvType = RecvEnum::nonBlock, bool setIpPort = false);
  ~UDP();

  void SetIpPort(const char* targetIP, uint16_t targetPort);  // if not indicated at constructor function
  void SetRecvTimeout(int time);                              // use in RecvEnum::blockTimeout  (unit: ms)

  void SetDisconnectTime(float callback_dt, float disconnectTime);  // initiativeDisconnect = true, disconnect for another IP to connect
  void SetAccessibleTime(float callback_dt, float accessibleTime);  // check if can access data

  int Send();
  int Recv();  // directly save in buffer

  void InitCmdData(HighCmd& cmd);
  void InitCmdData(LowCmd& cmd);
  int SetSend(char*);
  int SetSend(HighCmd&);
  int SetSend(LowCmd&);
  void GetRecv(char*);
  void GetRecv(HighState&);
  void GetRecv(LowState&);

  UDPState udpState;
  char* targetIP;
  uint16_t targetPort;
  char* localIP;
  uint16_t localPort;
  bool accessible = false;  // can access or not

 private:
  void init(uint16_t localPort, const char* targetIP = NULL, uint16_t targetPort = 0);

  int sockFd;
  bool connected;  // udp works with connect() function, rather than server mode
  int sendLength;
  int recvLength;
  int lose_recv;

  char* recvBuf;
  char* recvAvaliable;
  char* sendBuf;
  pthread_mutex_t sendMutex;
  pthread_mutex_t recvMutex;
  pthread_mutex_t udpMutex;

  bool nonblock = true;
  int blockTimeout = -1;              // use time out method or not, (unit: ms)
  bool initiativeDisconnect = false;  //
};

}  // namespace UNITREE_LEGGED_SDK

#endif
