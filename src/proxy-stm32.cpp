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

#include "cluon-complete.hpp"
#include "proxy-stm32.hpp"

#include <fstream>
#include <iostream>
#include <cstring>
#include <vector>
#include <string>
#include <ctime>
#include <chrono>

STM::STM(bool verbose, uint32_t id)
    : m_conversionConst(1)
    , m_senderStampOffsetAnalog(id*1000+200)
    , m_senderStampOffsetGpio(id*1000)
    , m_pins()
    , m_debug(verbose)
{
	STM::setUp();
}

STM::~STM()
{
}

void STM::setUp() 
{
	//string of pins, later used for getAnalogReadings()
	std::vector<std::string> pinsVecString = {"0", "1", "2", "3", "5", "6"};
  for(std::string const& str : pinsVecString) {
    m_pins.push_back(std::stoi(str));
  }
}

void STM::sendBackReadings(cluon::OD4Session &od4)
{
}

void STM::decode(cluon::OD4Session &od4, std::string msg)
{//DECODE BYTES FROM STM32F4 into readings
 //------------------------------SYNC THIS CODE WITH MAX--------------------------------------
 /* Currently assumed payload as: "<type: 1 byte>.<pin: 1 byte>.<value: 4bytes><\n>
 - exp: "a.0.1030\n" (raw analog reading from pin 0 (m_analogPinSteerPosition) with value of 1030
 - exp: "d.112.1\n" (digital (GPIO-I) reading from pin 112 (ClampSensor) with value of 1
 - each message has 9 bytes (6 payload + 2 seperator + 1 EOL), and is seperated by a EOL character \n
 */
 uint16_t receivedLength = msg.length();
 uint16_t correctLength = 9; //a correct message should have 9 bytes
 if(receivedLength <  correctLength && m_debug){
 	//std::cout << "Message corrupted, received: " << msg << " (bytes: " << receivedLength << ")\n";
 }
 else{
 	//if(m_debug)
  //std::cout << "Received: " << msg << std::endl;
 	char dataType = msg[0]; //AIN or GPIO-I
 	uint16_t pin = (uint8_t)msg[2]-48; //pin identifier (ascii to int)
 	uint32_t rawVal = std::stoi(msg.substr(4,4)); //value
 	//std::cout << "data type: " << dataType << ", pin: " << pin << ", value: " << rawVal << std::endl; 
 	if(dataType == 'a'){ // broadcast analog readings, depending on the pin function
				std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
      cluon::data::TimeStamp sampleTime = cluon::time::convert(tp);
      int16_t senderStamp = (int16_t) pin + m_senderStampOffsetAnalog;
        
			float value;
				//Convert raw readings(0-4019) to actual measurements (mm, bar)		
        if(pin == m_analogPinSteerPosition){
          value = (float)rawVal/((float) m_analogConvSteerPosition)-((float) m_analogOffsetSteerPosition);
        }else if(pin == m_analogPinEbsLine){
          value = (float)rawVal/((float) m_analogConvEbsLine)-((float) m_analogOffsetEbsLine);
        }else if(pin == m_analogPinServiceTank){
          value = (float)rawVal/((float) m_analogConvServiceTank)-((float) m_analogOffsetServiceTank);
        }else if(pin == m_analogPinEbsActuator){
          value = (float)rawVal/((float) m_analogConvEbsActuator)-((float) m_analogOffsetEbsActuator);
        }else if(pin == m_analogPinPressureReg){
          value = (float)rawVal/((float) m_analogConvPressureReg)-((float) m_analogOffsetPressureReg);
        }else if(pin == m_analogPinSteerPositionRack){
          value = (float)rawVal/((float) m_analogConvSteerPositionRack)-((float) m_analogOffsetSteerPositionRack);
        }
				//Package & send the messages to other microservices
        if(pin == m_analogPinSteerPosition || pin == m_analogPinSteerPositionRack){
          opendlv::proxy::GroundSteeringReading msgSteer;
          msgSteer.groundSteering(value);
          od4.send(msgSteer, sampleTime, senderStamp);
        }else if(pin == m_analogPinEbsLine || pin == m_analogPinServiceTank || pin == m_analogPinEbsActuator || pin == m_analogPinPressureReg){
          opendlv::proxy::PressureReading msgPressure;
          msgPressure.pressure(value);
          od4.send(msgPressure, sampleTime, senderStamp);
        }else{
          opendlv::proxy::VoltageReading msg;
          msg.torque(value);
          od4.send(msg, sampleTime, senderStamp);
        }
	}
	else{ // broadcast GPIO-I readings
	}
 }
}

void STM::collectRequests(std::string type, unsigned int pin, int value)
{
   unsigned int maxRequest = 50;
   Request request(type, pin, value);
   if(type == "gpio" && m_GpioRequests.size() < maxRequest){
     m_GpioRequests.push_back(request);
   }
   else if(type == "pwm" && m_PwmRequests.size() < maxRequest){
     m_PwmRequests.push_back(request);
   }
   if(m_debug){
     //std::cout << "Size of m_GpioRequests: " << m_GpioRequests.size() << std::endl;
     //std::cout << "Size of m_PwmRequests: " << m_PwmRequests.size() << std::endl;
   }
}
// Convert SwitchStateRequestGpio into bytes to STM32F4
void STM::sendNetstring(serial::Port* port, int16_t pin, bool value){
	//Netstrings have the following format:
  // ASCII Number representing the length of the payload + ':' + payload + ','
	std::stringstream payloadStream;
  payloadStream << "d." << std::to_string(pin) << '.'  << std::to_string((uint32_t)value) << '.';
  //Get the length of payload
	std::string lengthStr = std::to_string(payloadStream.str().length());
	//Append delimiter ',' after payload bytes
	payloadStream << ',';
	
	std::stringstream msgStream;
	//Append payload size + ':' in front of payload bytes
	msgStream << lengthStr << ':' << payloadStream.str();
	//Write to serial port
	port->write(msgStream.str());
}

uint32_t STM::getSenderStampOffsetGpio(){
	return m_senderStampOffsetGpio;
}
std::vector<std::pair<uint16_t, float>> STM::getAnalogReadings() {
}

uint32_t STM::getSenderStampOffsetPwm(){
}

void STM::tearDown() 
{
}
