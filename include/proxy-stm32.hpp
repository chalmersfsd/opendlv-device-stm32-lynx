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

#include "opendlv-standard-message-set.hpp"
#include "serialport.hpp"
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include "request.hpp"
class STM {
	public:
	 STM(bool verbose, uint32_t id);
	 ~STM();
	 
	 public:
	  void body(cluon::OD4Session &od4);
	  void decode(cluon::OD4Session &od4, std::string msg);
	  void sendBackReadings(cluon::OD4Session &od4);
	  void collectRequests(std::string type, unsigned int pin, int value);
	 // ANALOG FUNCTIONS AND VARIABLES
	 private:
		uint32_t count = 0;
	  uint32_t m_senderStampOffsetAnalog;
	  uint32_t m_senderStampOffsetGpio;
	  bool m_debug;
	  void setUp();
	  void tearDown();

	  std::vector<std::pair<uint16_t, float>> getAnalogReadings();
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
	 // GPIO/SwitchStateRequest handler
	 public:
	  void sendNetstring(serial::Port* port, int16_t pin, bool value);
		uint32_t getSenderStampOffsetGpio();
		std::vector<Request> m_GpioRequests;
		
   // PWM request handler
    uint32_t getSenderStampOffsetPwm();
		std::vector<Request> m_PwmRequests;
   
};

#endif
