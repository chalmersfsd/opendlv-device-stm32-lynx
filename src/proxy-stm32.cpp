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
	BBB_GPIO[65] = CLAMP_SET;
	BBB_GPIO[45] = COMPRESSOR;
	BBB_GPIO[61] = EBS_RELIEF;
	BBB_GPIO[44] = EBS_SPEAKER;
	BBB_GPIO[66] = FINISHED;
	BBB_GPIO[27] = HEART_BEAT;	
	BBB_GPIO[46] = RACK_RIGHT;
	BBB_GPIO[47] = RACK_LEFT;
	BBB_GPIO[69] = SERVICE_BREAK;
	BBB_GPIO[68] = REDUNDENCY;
	BBB_GPIO[67] = SHUTDOWN;
	BBB_GPIO[62] = SPARE;
	
	BBB_GPIO[49] = EBS_OK;
	BBB_GPIO[112] = CLAMPED_SENSOR;
	BBB_GPIO[115] = ASMS;
	
	BBB_Analog[STEER_POS] = 0;
	BBB_Analog[EBS_LINE] = 1;
	BBB_Analog[SERVICE_TANK] = 2;
	BBB_Analog[EBS_ACTUATOR] = 3;
	BBB_Analog[PRESSURE_RAG] = 5;
	BBB_Analog[POSITION_RACK] = 6;
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
/* --- Send requests to STM32F4 --- */
void STM::send(serial::Port* port)
{
  //Encode & send GPIO requests
  if(m_GpioRequests.size() > 0){
    for(Request rq : m_GpioRequests)
    {
       std::string payload = encodePayload(rq);
       std::string netstringMsg = encodeNetstring(payload);
       //send netstring request over serial port
       port->write(netstringMsg);
       //for debuging
       //std::cout << netstringMsg << std::endl;
    }
    // clear all GPIO requests
    m_GpioRequests.clear();
  }
  
}

std::string STM::encodeNetstring(const std::string payload)
{
  return std::to_string(payload.length()) + ":" + payload + MSG_END;
}

std::string STM::encodePayload(Request rq)
{
  unsigned int pin = rq.m_pin;
  int value = rq.m_value;
  std::map<int,std::string>::iterator it;
  
  it = BBB_GPIO.find(pin);
  if (it == BBB_GPIO.end()){
    //std::cout << "ERROR in encodePayload(): cannot find requested pin: " << pin << std::endl;
    return "error";
  } 
  else
  {
    std::string payload = std::string(SET) + DELIMITER + it->second + DELIMITER + std::to_string(value);
    return payload;
  }
}

void STM::read(const std::string data)
{  
	//store read bytes in buffer
	int bufferSize = 256;
	if(receiveBuffer.length() <= bufferSize)
	{
	std::stringstream ss;
	ss << receiveBuffer << data;
	receiveBuffer = ss.str();
	}
}

/* Decode payloads sent from STM32 and send back to other microservices */
void STM::sendBackAnalog(cluon::OD4Session * od4, uint16_t pin, uint32_t rawVal)
{
		//Currently using old BBB cape, which has 12bit ADC 0-1.8V, while STM32 has 12bit ADC 0-3.3V, thus need to scale down the raw value
		rawVal = 1.8/3.3*rawVal;
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
    	od4->send(msgSteer, sampleTime, senderStamp);
		}else if(pin == m_analogPinEbsLine || pin == m_analogPinServiceTank || pin == m_analogPinEbsActuator || pin == m_analogPinPressureReg){
    	opendlv::proxy::PressureReading msgPressure;
    	msgPressure.pressure(value);
    	od4->send(msgPressure, sampleTime, senderStamp);
		}else{
    	opendlv::proxy::VoltageReading msg;
    	msg.torque(value);
    	od4->send(msg, sampleTime, senderStamp);
		}	
}
void STM::decodePayload(cluon::OD4Session * od4)
{
	//std::cout << "m_Payloads.size() == " << m_Payloads.size() << std::endl;
  if(m_Payloads.size() > 0) // Send Analog readings
  {
    for(std::string payload : m_Payloads)
    {    
      char* endptr;
      size_t pos = std::string::npos;
      unsigned int delimiterPos1;
      unsigned int rawVal;
      float value;
      unsigned int pin = -1;
      std::string pinFunc;
      std::map<std::string,int>::iterator it;
      
      pinFunc = EBS_LINE;
      pos = payload.find(pinFunc);
      if(pos != std::string::npos) {   	
				it = BBB_Analog.find(pinFunc);
				if (it == BBB_Analog.end()){
    			//std::cout << "ERROR: cannot find requested pin: " << pinFunc << std::endl;
    		}
				else {
					pin = it->second;
					unsigned int delimiterPos1 = pos + pinFunc.length();
      		unsigned int rawVal = strtol(payload.c_str() + delimiterPos1 + 1, &endptr, 10);
      		//std::cout << "Received analog: " << pinFunc << " : " << rawVal << std::endl;
      		sendBackAnalog(od4, pin, rawVal);
				}	
			}
			
			pinFunc = SERVICE_TANK;
      pos = payload.find(pinFunc);
      if(pos != std::string::npos) {   	
				it = BBB_Analog.find(pinFunc);
				if (it == BBB_Analog.end()){
    			//std::cout << "ERROR: cannot find requested pin: " << pinFunc << std::endl;
    		}
				else {
					pin = it->second;
					unsigned int delimiterPos1 = pos + pinFunc.length();
      		unsigned int rawVal = strtol(payload.c_str() + delimiterPos1 + 1, &endptr, 10);
      		//std::cout << "Received analog: " << pinFunc << " : " << rawVal << std::endl;
      		sendBackAnalog(od4, pin, rawVal);
				}	
			}
			else {
				//std::cout << "ERROR: cannot find: " << pinFunc << " in payload" << std::endl;
			}
			
			pinFunc = EBS_ACTUATOR;
      pos = payload.find(pinFunc);
      if(pos != std::string::npos) {   	
				it = BBB_Analog.find(pinFunc);
				if (it == BBB_Analog.end()){
    			//std::cout << "ERROR: cannot find requested pin: " << pinFunc << std::endl;
    		}
				else {
					pin = it->second;
					unsigned int delimiterPos1 = pos + pinFunc.length();
      		unsigned int rawVal = strtol(payload.c_str() + delimiterPos1 + 1, &endptr, 10);
      		//std::cout << "Received analog: " << pinFunc << " : " << rawVal << std::endl;
      		sendBackAnalog(od4, pin, rawVal);
				}	
			}
			else {
				//std::cout << "ERROR: cannot find: " << pinFunc << " in payload" << std::endl;
			}
			
			pinFunc = PRESSURE_RAG;
      pos = payload.find(pinFunc);
      if(pos != std::string::npos) {   	
				it = BBB_Analog.find(pinFunc);
				if (it == BBB_Analog.end()){
    			//std::cout << "ERROR: cannot find requested pin: " << pinFunc << std::endl;
    		}
				else {
					pin = it->second;
					unsigned int delimiterPos1 = pos + pinFunc.length();
      		unsigned int rawVal = strtol(payload.c_str() + delimiterPos1 + 1, &endptr, 10);
      		//std::cout << "Received analog: " << pinFunc << " : " << rawVal << std::endl;
      		sendBackAnalog(od4, pin, rawVal);
				}	
			}
			else {
				//std::cout << "ERROR: cannot find: " << pinFunc << " in payload" << std::endl;
			}
			
			pinFunc = POSITION_RACK;
      pos = payload.find(pinFunc);
      if(pos != std::string::npos) {   	
				it = BBB_Analog.find(pinFunc);
				if (it == BBB_Analog.end()){
    			//std::cout << "ERROR: cannot find requested pin: " << pinFunc << std::endl;
    		}
				else {
					pin = it->second;
					unsigned int delimiterPos1 = pos + pinFunc.length();
      		unsigned int rawVal = strtol(payload.c_str() + delimiterPos1 + 1, &endptr, 10);
      		//std::cout << "Received analog: " << pinFunc << " : " << rawVal << std::endl;
      		sendBackAnalog(od4, pin, rawVal);
				}	
			}
			else {
				//std::cout << "ERROR: cannot find: " << pinFunc << " in payload" << std::endl;
			}
			
			pinFunc = STEER_POS;
      pos = payload.find(pinFunc);
      if(pos != std::string::npos) {   	
				it = BBB_Analog.find(pinFunc);
				if (it == BBB_Analog.end()){
    			//std::cout << "ERROR: cannot find requested pin: " << pinFunc << std::endl;
    		}
				else {
					pin = it->second;
					unsigned int delimiterPos1 = pos + pinFunc.length();
      		unsigned int rawVal = strtol(payload.c_str() + delimiterPos1 + 1, &endptr, 10);
      		//std::cout << "Received analog: " << pinFunc << " : " << rawVal << std::endl;
      		sendBackAnalog(od4, pin, rawVal);
				}	
			}
			else {
				//std::cout << "ERROR: cannot find: " << pinFunc << " in payload" << std::endl;
			}
    }
    m_Payloads.clear();
  }
}
void STM::extractPayload()
{
	if(receiveBuffer.length() > 3)
	{
	  char *colonSign = NULL;
		unsigned int lengthOfPayload = strtol(receiveBuffer.c_str(), &colonSign, 10);
				if (*colonSign == 0x3a) {

			// Found colon sign.
			// First, check if the buffer is as long as it is stated in the netstring.
			// This prevents the case where the colon is near the end of string, which can lead to out of range access later
			if(receiveBuffer.length() - (colonSign + 1 - receiveBuffer.c_str()) < (int)lengthOfPayload)
			{
			// Received data is too short. Skip further processing this part.
				return;
			}	   
			// Now, check if (receiveBuffer + 1 + lengthOfPayload) == MSG_END.
			if ((colonSign[1 + lengthOfPayload]) == MSG_END) {
			  // Successfully found a complete Netstring.
				int start = colonSign + 1 - receiveBuffer.c_str();
				std::string payload = receiveBuffer.substr(start,lengthOfPayload);
				//std::cout << payload << std::endl;
				if(m_Payloads.size() < 100) //Maximum payload storage
					m_Payloads.push_back(payload);
				// Remove decoded Netstring from receiveBuffer
				char* MsgEndPtr =  colonSign + 1 + lengthOfPayload;
				int numberOfCharToRemove = MsgEndPtr + 1 - receiveBuffer.c_str(); 
				receiveBuffer = receiveBuffer.erase(0, numberOfCharToRemove);
			
			} else // The message is corrupted (missing bytes), remove this message from buffer
			{
			  //Shift the bytes in buffer so that buffer starts with payload size (otherwise strtol won't work) 
			  unsigned int endPos = receiveBuffer.find(";");
			  if(endPos != std::string::npos && endPos+1 < receiveBuffer.length())
			  {
			    receiveBuffer = receiveBuffer.substr(endPos+1, receiveBuffer.length() - endPos);
			  }
			}
		} else // The message is corrupted (missing bytes), remove this message from buffer
		{   //Shift the bytes in buffer so that buffer starts with payload zie
			  unsigned int endPos = receiveBuffer.find(";");
			  if(endPos != std::string::npos && endPos+1 < receiveBuffer.length())
			  {
			    receiveBuffer = receiveBuffer.substr(endPos+1, receiveBuffer.length() - endPos);
			  }
	  }
	}
}

uint32_t STM::getSenderStampOffsetGpio(){
	return m_senderStampOffsetGpio;
}

uint32_t STM::getSenderStampOffsetPwm(){
}

void STM::tearDown() 
{
}
