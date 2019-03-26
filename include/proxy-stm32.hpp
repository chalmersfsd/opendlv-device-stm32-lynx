/*
 * Copyright (C) 2018  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef STM32
#define STM32

/*===========================================================================*/
/* Docker-STM32 protocol related stuff.                                      */
/*===========================================================================*/
#define DELIMITER  '|'
#define MSG_END  ';'
#define SET  "set"
#define STATUS  "status"
#define STATUS_DELIMITER ':'

/* --- status_msgs --- */
#define ASMS  "asms"
#define CLAMPED_SENSOR  "clamped_sensor"
#define EBS_ACTUATOR  "ebs_actuator"
#define EBS_LINE  "ebs_line"
#define EBS_OK  "ebs_ok"
#define POSITION_RACK  "position_rack"
#define PRESSURE_RAG  "pressure_rag"
#define SERVICE_TANK  "service_tank"
#define STEER_POS  "steer_pos"

/* --- setter_msgs --- */
#define CLAMP_SET  "clamp_set"
#define COMPRESSOR  "compressor"
#define EBS_RELIEF  "ebs_relief"
#define EBS_SPEAKER  "ebs_speaker"
#define FINISHED  "finished"
#define HEART_BEAT  "heart_beat"
#define SERVICE_BREAK  "service_break"
#define SHUTDOWN  "shutdown"
#define RACK_LEFT  "rack_left"
#define RACK_RIGHT "rack_right"
#define REDUNDENCY "redundency"
#define SPARE "spare"

#define STEER_SPEED "steer_speed"
#define BRAKE_PRESSURE "brake_pressure"
#define ASSI_BLUE "assi_blue"
#define ASSI_RED "assi_red"
#define ASSI_GREEN "assi_green"

#include "opendlv-standard-message-set.hpp"
#include "serialport.hpp"
#include <stdlib.h>
#include <string.h>

#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <map>
#include "request.hpp"
class STM {
	public:
	 STM(bool verbose, uint32_t id);
	 ~STM();
	 
	 public:
	  void body(cluon::OD4Session &od4);
	  void collectRequests(std::string type, unsigned int pin, int value);
	 // ANALOG FUNCTIONS AND VARIABLES
	 private:
		uint32_t count = 0;
	  uint32_t m_senderStampOffsetAnalog;
	  uint32_t m_senderStampOffsetGpio;
	  uint32_t m_senderStampOffsetPwm;
	  bool m_debug;
	  void setUp();
	  void tearDown();

	  float m_conversionConst;
	  std::vector<uint16_t> m_pins;
	  const uint16_t m_analogPinSteerPosition = 0;
	  const uint16_t m_analogPinSteerPositionRack = 6;
	  const uint16_t m_analogPinServiceTank = 2;
	  const uint16_t m_analogPinPressureReg = 5;
	  const uint16_t m_analogPinEbsLine = 1;
	  const uint16_t m_analogPinEbsActuator = 3;
	  
	  const uint16_t m_gpioPinAsms = 115;
    const uint16_t m_gpioPinEbsOk = 49;
    const uint16_t m_gpioPinClampSensor = 112;
	  
	  const double m_analogConvSteerPosition = 80.38;
	  const double m_analogConvEbsLine = 378.5;
	  const double m_analogConvServiceTank = 377.6;
	  const double m_analogConvEbsActuator = 377.9;
	  const double m_analogConvPressureReg = 378.7;
	  const double m_analogConvSteerPositionRack = 80.86;

	  const double m_analogOffsetSteerPosition = 27.74;
	  const double m_analogOffsetEbsLine = 0.11;
	  const double m_analogOffsetServiceTank = 0.11;
	  const double m_analogOffsetEbsActuator = 0.11;
	  const double m_analogOffsetPressureReg = 0;
	  const double m_analogOffsetSteerPositionRack = 31.06;
	  
	  //Pins of BeagleboneBlack and their functionalities
	  std::map<int, std::string> BBB_GPIO;
	  std::map<int, std::string> BBB_PWM;
	  std::map<std::string, int> BBB_Analog;
	  
	 // GPIO/SwitchStateRequest handler
	 public:
		uint32_t getSenderStampOffsetGpio();
		std::vector<Request> m_GpioRequests;
		
   // PWM request handler
    uint32_t getSenderStampOffsetPwm();
		std::vector<Request> m_PwmRequests;
		
	 //  send function
	 void send(serial::Serial* port);
	 unsigned int sendWithACK(serial::Serial* port, std::string payload, std::string netstringMsg);
	 std::string encodePayload(std::string type, Request rq);
	 std::string encodeNetstring(const std::string payload);
	 
	 //  read function
	 void SendStatusRequestToSTM(serial::Serial* port);
	 void GetBytesFromSTM(const std::string data);
   void extractPayload();
   void decodePayload(cluon::OD4Session* od4, cluon::OD4Session* od4Gpio, bool rackPos, bool steerPos, bool ebsLine, bool ebsAct, bool servTank, bool pressReg, bool asms, bool clamped, bool ebsOK);
   float sendBackAnalog(cluon::OD4Session * od4, uint16_t pin, uint32_t rawVal);
	 std::string receiveBuffer;
	 void sendBackDigital(cluon::OD4Session * od4Gpio, uint16_t pin, uint32_t val);
	 
	 // send/read flags
	 bool sendOK;
	 bool readOK;
	 bool bytesAvailable;
	 //Decoded payload container
	 std::vector<std::string> m_Payloads;
	 
	 //view raw analog signals for debugging
	 float rawSteerPosition;
	 float rawSteerPositionRack;
	 float rawServiceTank;
	 float rawPressureReg;
	 float rawEbsLine;
	 float rawEbsActuator;
	 //view digital inputs for debugging
	 bool rawAsms;
	 bool rawClamped;
	 bool rawEbsOK;
	 void viewAnalogRaw(bool rackPos, bool steerPos, bool ebsLine, bool ebsAct, bool servTank, bool pressReg);

};

#define ASMS  "asms"
#define CLAMPED_SENSOR  "clamped_sensor"
#define EBS_ACTUATOR  "ebs_actuator"
#define EBS_LINE  "ebs_line"
#define EBS_OK  "ebs_ok"
#define POSITION_RACK  "position_rack"
#define PRESSURE_RAG  "pressure_rag"
#define SERVICE_TANK  "service_tank"
#define STEER_POS  "steer_pos"

#endif
