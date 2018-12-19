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
	  const double m_analogOffsetSteerPositionRack = 28.06;
	  
	  //Pins of BeagleboneBlack and their functionalities
	  std::map<int, std::string> BBB_GPIO;
	  std::map<std::string, int> BBB_Analog;
	 // GPIO/SwitchStateRequest handler
	 public:
		uint32_t getSenderStampOffsetGpio();
		std::vector<Request> m_GpioRequests;
		
   // PWM request handler
    uint32_t getSenderStampOffsetPwm();
		std::vector<Request> m_PwmRequests;
		
	 // Generic send function
	 void send(serial::Port* port);
	 std::string encodePayload(Request rq);
	 std::string encodeNetstring(const std::string payload);
	 
	 // Generic read function
	 void read(const std::string data);
   void extractPayload();
   void decodePayload(cluon::OD4Session* od4);
   void sendBackAnalog(cluon::OD4Session * od4, uint16_t pin, uint32_t rawVal);
	 std::string receiveBuffer;
	 
	 //Decoded payload container
	 std::vector<std::string> m_Payloads;
};

#endif
