// /************************************************************************
// Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
// Use of this source code is governed by the MPL-2.0 license, see LICENSE.
// ************************************************************************/

// #ifndef _UNITREE_LEGGED_COMM_H_
// #define _UNITREE_LEGGED_COMM_H_

// #include <stdint.h>
// #include <array>
// namespace UNITREE_LEGGED_SDK 
// {

// 	constexpr int HIGHLEVEL = 0xee;
// 	constexpr int LOWLEVEL  = 0xff;
// 	constexpr int TRIGERLEVEL = 0xf0;
// 	constexpr double PosStopF = (2.146E+9f);
// 	constexpr double VelStopF = (16000.0f);
// 	extern const int HIGH_CMD_LENGTH;   // sizeof(HighCmd)
// 	extern const int HIGH_STATE_LENGTH; // sizeof(HighState)
// 	extern const int LOW_CMD_LENGTH;    // shorter than sizeof(LowCmd),   bytes compressed LowCmd length
// 	extern const int LOW_STATE_LENGTH;  // shorter than sizeof(LowState), bytes compressed LowState length
// #pragma pack(1)
// 	// typedef struct
// 	// {
// 	// 	uint8_t off; // set 0xA5 to turn off the battery, please try it under the premise of ensuring safety
// 	// 	uint8_t, 3> reserve;
// 	// } BmsCmd;

// 	// typedef struct
// 	// {
// 	// 	uint8_t version_h;
// 	// 	uint8_t version_l;
// 	// 	uint8_t bms_status;                // 0x00 : wakeup, 0X01 :  discharge, 0x02 : charge, 0x03 : charger, 0x04 : precharge, 0x05 : charge_err, 0x06 : waterfall_light, 0x07 : self_discharge, 0x08 : junk.
// 	// 	uint8_t SOC;                       // SOC 0-100%
// 	// 	int32_t current;                   // （unit: mA)
// 	// 	uint16_t cycle;                    // The current number of cycles of the battery
// 	// 	int8_t, 2> BQ_NTC;      // x1 degrees centigrade
// 	// 	int8_t, 2> MCU_NTC;     // x1 degrees centigrade
// 	// 	uint16_t, 10> cell_vol; // cell voltage mV
// 	// } BmsState;
// 	typedef struct
// 	{
// 		float x;
// 		float y;
// 		float z;
// 	} Cartesian;

// 	typedef struct
// 	{
// 		float quaternion[4];               // quaternion, normalized, (w,x,y,z)
// 		float gyroscope[3];                // angular velocity （unit: rad/s)
// 		float accelerometer[3];            // m/(s2)
// 		float rpy[3];                      // euler angle（unit: rad)
// 		int8_t temperature;
// 	} IMU;                                 // when under accelerated motion, the attitude of the robot calculated by IMU will drift.

// 	typedef struct
// 	{
// 		uint8_t r;
// 		uint8_t g;
// 		uint8_t b;
// 	} LED;                                 // foot led brightness: 0~255

// 	typedef struct
// 	{
// 		uint8_t mode;                      // motor working mode 
// 		float q;                           // current angle (unit: radian)
// 		float dq;                          // current velocity (unit: radian/second)
// 		float ddq;                         // current acc (unit: radian/second*second)
// 		float tauEst;                      // current estimated output torque (unit: N.m)
// 		float q_raw;                       // current angle (unit: radian)
// 		float dq_raw;                      // current velocity (unit: radian/second)
// 		float ddq_raw;
// 		int8_t temperature;                // current temperature (temperature conduction is slow that leads to lag)
// 		uint32_t reserve[2];
// 	} MotorState;                          // motor feedback

// 	typedef struct
// 	{
// 		uint8_t mode;                      // desired working mode
// 		float q;                           // desired angle (unit: radian) 
// 		float dq;                          // desired velocity (unit: radian/second)
// 		float tau;                         // desired output torque (unit: N.m)
// 		float Kp;                          // desired position stiffness (unit: N.m/rad )
// 		float Kd;                          // desired velocity stiffness (unit: N.m/(rad/s) )
// 		uint32_t reserve[3];
// 	} MotorCmd;                            // motor control

// 	typedef struct
// 	{
// 		uint8_t levelFlag;                 // flag to distinguish high level or low level
// 		uint16_t commVersion;
// 		uint16_t robotID;
// 		uint32_t SN; 
// 		uint8_t bandWidth;
// 		IMU imu;
// 		MotorState motorState[20];
// 		int16_t footForce[4];              // force sensors
// 		int16_t footForceEst[4];           // force sensors
// 		uint32_t tick;                     // reference real-time from motion controller (unit: us)
// 		uint8_t wirelessRemote[40];        // wireless commands
// 		uint32_t reserve;
// 		uint32_t crc;
// 	} LowState;                            // low level feedback

// 	typedef struct 
// 	{
// 		uint8_t levelFlag;
// 		uint16_t commVersion;
// 		uint16_t robotID;
// 		uint32_t SN;
// 		uint8_t bandWidth;
// 		MotorCmd motorCmd[20];
// 		LED led[4];
// 		uint8_t wirelessRemote[40];
// 		uint32_t reserve;
// 		uint32_t crc;
// 	} LowCmd;                              // low level control

// 	typedef struct
// 	{
// 		uint8_t levelFlag;
// 		uint16_t commVersion;
// 		uint16_t robotID;
// 		uint32_t SN;
// 		uint8_t bandWidth;
// 		uint8_t mode;
// 		IMU imu;
// 		float forwardSpeed;               
// 		float sideSpeed;                  
// 		float rotateSpeed;                
// 		float bodyHeight;                 
// 		float updownSpeed;                 // speed of stand up or squat down
// 		float forwardPosition;             // front or rear displacement, an integrated number form kinematics function, usually drift
// 		float sidePosition;                // left or right displacement, an integrated number form kinematics function, usually drift
// 		Cartesian footPosition2Body[4];    // foot position relative to body
// 		Cartesian footSpeed2Body[4];       // foot speed relative to body
// 		int16_t footForce[4];
// 		int16_t footForceEst[4];
// 		uint32_t tick;                     // reference real-time from motion controller (unit: us)
// 		uint8_t wirelessRemote[40];
// 		uint32_t reserve;
// 		uint32_t crc;
// 	} HighState;                           // high level feedback

// 	typedef struct
// 	{
// 		uint8_t levelFlag;
// 		uint16_t commVersion;
// 		uint16_t robotID;
// 		uint32_t SN;
// 		uint8_t bandWidth;
// 		uint8_t mode;                      // 0:idle, default stand      1:forced stand     2:walk continuously
// 		float forwardSpeed;                // speed of move forward or backward, scale: -1~1
// 		float sideSpeed;                   // speed of move left or right, scale: -1~1
// 		float rotateSpeed;	               // speed of spin left or right, scale: -1~1
// 		float bodyHeight;                  // body height, scale: -1~1
// 		float footRaiseHeight;             // foot up height while walking (unavailable now)
// 		float yaw;                         // unit: radian, scale: -1~1
// 		float pitch;                       // unit: radian, scale: -1~1
// 		float roll;                        // unit: radian, scale: -1~1
// 		LED led[4];
// 		uint8_t wirelessRemote[40];
// 		uint8_t AppRemote[40];
// 		uint32_t reserve;
// 		uint32_t crc;
// 	} HighCmd;                             // high level control

// #pragma pack()

// 	typedef struct     
// 	{
// 		unsigned long long TotalCount;     // total loop count
// 		unsigned long long SendCount;      // total send count
// 		unsigned long long RecvCount;      // total receive count
// 		unsigned long long SendError;      // total send error 
// 		unsigned long long FlagError;      // total flag error 
// 		unsigned long long RecvCRCError;   // total reveive CRC error	
// 		unsigned long long RecvLoseError;  // total lose package count	
// 	} UDPState;                            // UDP communication state

// 	constexpr int HIGH_CMD_LENGTH   = (sizeof(HighCmd));
// 	constexpr int HIGH_STATE_LENGTH = (sizeof(HighState));
	
// }

// #endif
/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#ifndef _UNITREE_LEGGED_COMM_H_
#define _UNITREE_LEGGED_COMM_H_

#include <stdint.h>
// #include <array>

namespace UNITREE_LEGGED_SDK
{

  constexpr int HIGHLEVEL = 0xee;
  constexpr int LOWLEVEL = 0xff;
  constexpr int TRIGERLEVEL = 0xf0;
  constexpr double PosStopF = (2.146E+9f);
  constexpr double VelStopF = (16000.0f);
  extern const int HIGH_CMD_LENGTH;   // sizeof(HighCmd)
  extern const int HIGH_STATE_LENGTH; // sizeof(HighState)
  extern const int LOW_CMD_LENGTH;    // shorter than sizeof(LowCmd),   bytes compressed LowCmd length
  extern const int LOW_STATE_LENGTH;  // shorter than sizeof(LowState), bytes compressed LowState length

#pragma pack(1)

  typedef struct
  {
    uint8_t off; // set 0xA5 to turn off the battery, please try it under the premise of ensuring safety
    uint8_t reserve[3];
  } BmsCmd;

  typedef struct
  {
    uint8_t version_h;
    uint8_t version_l;
    uint8_t bms_status;                // 0x00 : wakeup, 0X01 :  discharge, 0x02 : charge, 0x03 : charger, 0x04 : precharge, 0x05 : charge_err, 0x06 : waterfall_light, 0x07 : self_discharge, 0x08 : junk.
    uint8_t SOC;                       // SOC 0-100%
    int32_t current;                   // （unit: mA)
    uint16_t cycle;                    // The current number of cycles of the battery
    int8_t BQ_NTC[2];      // x1 degrees centigrade
    int8_t MCU_NTC[2];     // x1 degrees centigrade
    uint16_t cell_vol[10]; // cell voltage mV
  } BmsState;

  typedef struct
  {
    float x;
    float y;
    float z;
  } Cartesian;

  typedef struct
  {
    float quaternion[4];    // quaternion, normalized, (w,x,y,z)
    float gyroscope[3];     // angular velocity （unit: rad/s)    (raw data)
    float accelerometer[3]; // acceleration （unit: m/(s2))       (raw data)
    float rpy[3];           // euler angle（unit: rad)
    int8_t temperature;                 // the temperature of imu (unit: °C)
  } IMU;                                // when under accelerated motion, the attitude of the robot calculated by IMU will drift.

  typedef struct
  {
    uint8_t r;
    uint8_t g;
    uint8_t b;
  } LED; // reserve

  typedef struct
  {
    uint8_t mode;       // motor working mode. Servo : 0x0A, Damping : 0x00，Overheat ： 0x08.
    float q;            // current angle (unit: radian)
    float dq;           // current velocity (unit: radian/second)
    float ddq;          // current acc (unit: radian/second*second)
    float tauEst;       // current estimated output torque (unit: N.m)
    float q_raw;        // reserve
    float dq_raw;       // reserve
    float ddq_raw;      // reserve
    int8_t temperature; // current temperature (temperature conduction is slow that leads to lag)
    uint32_t reserve[2];
  } MotorState; // motor feedback

  typedef struct
  {
    uint8_t mode; // desired working mode. Servo : 0x0A, Damping : 0x00.
    float q;      // desired angle (unit: radian)
    float dq;     // desired velocity (unit: radian/second)
    float tau;    // desired output torque (unit: N.m)
    float Kp;     // desired position stiffness (unit: N.m/rad )
    float Kd;     // desired velocity stiffness (unit: N.m/(rad/s) )
    uint32_t reserve[3];
  } MotorCmd; // motor control

  typedef struct
  {
    uint8_t head[2]; // reserve
    uint8_t levelFlag;           // reserve
    uint8_t frameReserve;        // reserve

    uint32_t SN[2];      // reserve
    uint32_t version[2]; // reserve
    uint16_t bandWidth;              // reserve
    IMU imu;
    MotorState motorState[20];
    BmsState bms;
    int16_t footForce[4];       // Data from foot airbag sensor
    int16_t footForceEst[4];    // reserve，typically zero
    uint32_t tick;                          // reference real-time from motion controller (unit: ms)
    uint8_t wirelessRemote[40]; // Data from Unitree Joystick.
    uint32_t reserve;

    uint32_t crc;
  } LowState; // low level feedback

  typedef struct
  {
    uint8_t head[2]; // reserve
    uint8_t levelFlag;           // reserve
    uint8_t frameReserve;        // reserve

    uint32_t SN[2];      // reserve
    uint32_t version[2]; // reserve
    uint16_t bandWidth;
    MotorCmd motorCmd[20];
    BmsCmd bms;
    uint8_t wirelessRemote[40]; // reserve
    uint32_t reserve;

    uint32_t crc;
  } LowCmd; // low level control

  typedef struct
  {
    uint8_t head[2]; // reserve
    uint8_t levelFlag;           // reserve
    uint8_t frameReserve;        // reserve

    uint32_t SN[2];      // reserve
    uint32_t version[2]; // reserve
    uint16_t bandWidth;
    IMU imu;
    MotorState motorState[20];
    BmsState bms;
    int16_t footForce[4];           // Data from foot airbag sensor
    int16_t footForceEst[4];        // reserve，typically zero
    uint8_t mode;                               // The current mode of the robot
    float progress;                             // reserve
    uint8_t gaitType;                           // 0.idle  1.trot  2.trot running  3.climb stair  4.trot obstacle
    float footRaiseHeight;                      // (unit: m, default: 0.08m), foot up height while walking
    float position[3];              // (unit: m), from own odometry in inertial frame, usually drift
    float bodyHeight;                           // (unit: m, default: 0.28m),
    float velocity[3];              // (unit: m/s), forwardSpeed, sideSpeed, rotateSpeed in body frame
    float yawSpeed;                             // (unit: rad/s), rotateSpeed in body frame
    float rangeObstacle[4];         // Distance to nearest obstacle
    Cartesian footPosition2Body[4]; // foot position relative to body
    Cartesian footSpeed2Body[4];    // foot speed relative to body
    uint8_t wirelessRemote[40];     // Data from Unitree Joystick.
    uint32_t reserve;

    uint32_t crc;
  } HighState; // high level feedback

  typedef struct
  {
    uint8_t head[2]; // reserve, no need to set.
    uint8_t levelFlag;           // reserve. No need to set, only need to set UDP class.
    uint8_t frameReserve;        // reserve

    uint32_t SN[2];      // reserve
    uint32_t version[2]; // reserve
    uint16_t bandWidth;              // reserve
    uint8_t mode;                    // 0. idle, default stand
                                     // 1. force stand (controlled by dBodyHeight + ypr)
                                     // 2. target velocity walking (controlled by velocity + yawSpeed)
                                     // 3. target position walking (controlled by position + ypr[0]), reserve
                                     // 4. path mode walking (reserve for future release), reserve
                                     // 5. position stand down.
                                     // 6. position stand up
                                     // 7. damping mode
                                     // 8. recovery stand
                                     // 9. backflip, reserve
                                     // 10. jumpYaw, only left direction. Note, to use this mode, you need to set mode = 1 first.
                                     // 11. straightHand. Note, to use this mode, you need to set mode = 1 first.

    uint8_t gaitType;              // 0.idle
                                   // 1.trot
                                   // 2.trot running
                                   // 3.climb stair
                                   // 4.trot obstacle
    uint8_t speedLevel;            // reserve
    float footRaiseHeight;         // (unit: m, range: -0.06~0.03m, default: 0.09m), foot up height while walking, delta value
    float bodyHeight;              // (unit: m, range: -0.13~0.03m, default: 0.31m), delta value
    float position[2]; // (unit: m), desired position in inertial frame, reserve
    float euler[3];    // (unit: rad), roll pitch yaw in stand mode
                                   // (range: roll : -0.75~0.75rad)
                                   // (range: pitch: -0.75~0.75rad)
                                   // (range: yaw  : -0.6~0.6rad)
    float velocity[2]; // (unit: m/s), forwardSpeed, sideSpeed in body frame
                                   // (range: trot : vx:-1.1~1.5m/s,  vy:-1.0~1.0m/s)
                                   // (range: run  : vx:-2.5~3.5m/s,  vy:-1.0~1.0m/s)
                                   // (range: stair: vx:-0.2~0.25m/s, vy:-0.15~0.15m/s)
    float yawSpeed;                // (unit: rad/s), rotateSpeed in body frame
                                   // (range: trot : -4.0~4.0rad/s)
                                   // (range: run  : -4.0~4.0rad/s)
                                   // (range: stair: -0.7~0.7rad/s)
    BmsCmd bms;
    LED led[4];                 // reserve
    uint8_t wirelessRemote[40]; // reserve
    uint32_t reserve;

    uint32_t crc;
  } HighCmd; // high level control

#pragma pack()

  typedef struct
  {
    unsigned long long TotalCount;    // total loop count
    unsigned long long SendCount;     // total send count
    unsigned long long RecvCount;     // total receive count
    unsigned long long SendError;     // total send error
    unsigned long long FlagError;     // total flag error
    unsigned long long RecvCRCError;  // total reveive CRC error
    unsigned long long RecvLoseError; // total lose package count
  } UDPState;                         // UDP communication state

} // namespace UNITREE_LEGGED_SDK

#endif
