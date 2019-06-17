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
    , m_senderStampOffsetPwm(id*1000+300)
    , m_pins()
    , m_debug(verbose)
    , sendOK(false)
    , readOK(false)
    , bytesAvailable(false)
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
	
	BBB_PWM[40] = STEER_SPEED;
	BBB_PWM[41] = BRAKE_PRESSURE;
	BBB_PWM[0] = ASSI_BLUE;
	BBB_PWM[20] = ASSI_RED;
	BBB_PWM[21] = ASSI_GREEN;
	
	BBB_Analog[STEER_POS] = 0;
	BBB_Analog[EBS_LINE] = 1;
	BBB_Analog[SERVICE_TANK] = 2;
	BBB_Analog[EBS_ACTUATOR] = 3;
	BBB_Analog[PRESSURE_RAG] = 5;
	BBB_Analog[POSITION_RACK] = 6;
}

void STM::collectRequests(std::string type, unsigned int pin, int value)
{
   unsigned int maxRequest = 100;
   Request* request = new Request(type, pin, value);
   if(type == "gpio" && m_GpioRequests.size() < maxRequest){
     m_GpioRequests.push(request);
   }
   else if(type == "pwm" && m_PwmRequests.size() < maxRequest){
     m_PwmRequests.push(request);
   }
   if(m_debug){
     //std::cout << "Size of m_GpioRequests: " << m_GpioRequests.size() << std::endl;
     //std::cout << "Size of m_PwmRequests: " << m_PwmRequests.size() << std::endl;
   }
}
/* --- Send gpio/pwm requests to STM32F4 --- */
void STM::send(serial::Serial* port)
{
  //Encode & send GPIO requests
  if(m_debug){
    std::cout << "m_GpioRequests.size() = " << m_GpioRequests.size() << std::endl;
		}
  if(m_GpioRequests.size() > 0){
       std::string payload = encodePayload("gpio",m_GpioRequests.front());
       std::string netstringMsg = encodeNetstring(payload);
       
       //send netstring request over serial port
       unsigned result = sendWithACK(port, payload, netstringMsg);
       if(m_debug){
         if(result == -1)
           std::cout << "[STM32 Proxy]: Error sending " << netstringMsg << " : no ACK from STM32" << std::endl;
         else
           std::cout << "[STM32 Proxy]: " << netstringMsg << " : successfully sent" << std::endl;
       }     
    // remove the processed request
    delete m_GpioRequests.front();
    m_GpioRequests.pop();
  }
  
  //Encode & send PWM requests
  if(m_debug){
    std::cout << "m_PwmRequests.size() = " << m_PwmRequests.size() << std::endl;
		}
  if(m_PwmRequests.size() > 0){
       std::string payload = encodePayload("pwm",m_PwmRequests.front());
       std::string netstringMsg = encodeNetstring(payload);
       
       //send netstring request over serial port
       unsigned result = sendWithACK(port, payload, netstringMsg);
       if(m_debug){
         if(result == -1)
           std::cout << "[STM32 Proxy]: Error sending " << netstringMsg << " : no ACK from STM32" << std::endl;
         else
           std::cout << "[STM32 Proxy]: " << netstringMsg << " : successfully sent" << std::endl;
       }
    // clear all pwm requests
    delete m_PwmRequests.front();
    m_PwmRequests.pop();
  } 
}

/* --- Send gpio/pwm requests to STM32F4 with ACK/NACK check --- */
unsigned int STM::sendWithACK(serial::Serial* port, std::string payload, std::string netstringMsg)
{
  port->write(netstringMsg);
  unsigned int result = 0;
  unsigned int resendAttempts=3;
  while(result == 0){
    // Wait for STM to send back an ACK/NACK message
    if(port->waitReadable()){
        std::string resultMsg = port->read((size_t)128);
        //if STM responds with ACK, then no need to resend
        if(resultMsg.find(payload + "|ACK") != std::string::npos)
          result = 1;
        else
        //there's an error, resend request
        {
          port->write(netstringMsg);
          resendAttempts--;
        }
      }
      
      if(resendAttempts == 0) // too many resend attempts without success, quit and return error code
      {
        result = -1;
      }
    }
    return result;
}

std::string STM::encodeNetstring(const std::string payload)
{
  return std::to_string(payload.length()) + ":" + payload + MSG_END;
}

std::string STM::encodePayload(std::string type, Request* rq)
{
  unsigned int pin = rq->m_pin;
  int value = rq->m_value;
  std::map<int,std::string>::iterator it;
  
  if(type == "gpio"){
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
  } else if(type == "pwm") {
    if(value >= 100000){ // steer right, convert to a negative pwm request
      value = value - 100000; //this is an offset added in logic-lynx-steering for distinguishing negative numbers (since standard pwm request is non-negative)
      value = -value;
    }
    //Convert old pwm values (0-50000) to duty cycle (0-100.00 percent)
    int dutyCycle = (int)(((float)value/50000.0)*10000);
    it = BBB_PWM.find(pin);
    if (it == BBB_PWM.end()){
      //std::cout << "ERROR in encodePayload(): cannot find requested pin: " << pin << std::endl;
      return "error";
    } 
    else
    {
      std::string payload = std::string(SET) + DELIMITER + it->second + DELIMITER + std::to_string(dutyCycle);
      return payload;
    }
  }
}

//** Functions for getting status from STM32Discovery **
void STM::SendStatusRequestToSTM(serial::Serial* port){
  // send get request and encode as netstring
  std::string payload = "get";
  std::string netstringMsg = encodeNetstring(payload);
  // send bytes over serial port
  port->write(netstringMsg);
}

void STM::GetBytesFromSTM(const std::string data)
{  
	//store read bytes in buffer
	int bufferSize = 2048;
	if(receiveBuffer.length() <= bufferSize)
	{
	//std::cout << data << std::endl;
	std::stringstream ss;
	ss << receiveBuffer << data;
	receiveBuffer = ss.str();
	}
}

/* Decode payloads sent from STM32 and send back to other microservices */
float STM::sendBackAnalog(cluon::OD4Session * od4, uint16_t pin, uint32_t rawVal)
{
		//Currently using old BBB cape, which has 12bit ADC 0-1.8V, while STM32 has 12bit ADC 0-3.3V, thus need to re-scale the raw value
		rawVal = 3.3/1.8*rawVal;
		std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
		cluon::data::TimeStamp sampleTime = cluon::time::convert(tp);
		int16_t senderStamp = (int16_t) pin + m_senderStampOffsetAnalog;
        
		float value = 0.0;
		//Convert raw readings(0-4019) to actual measurements (mm, bar)		
		if(pin == m_analogPinSteerPosition){
    	value = (float)rawVal/((float) m_analogConvSteerPosition)-((float) m_analogOffsetSteerPosition);
		}else if(pin == m_analogPinEbsLine){
    	value = (float)rawVal/((float) m_analogConvEbsLine)-((float) m_analogOffsetEbsLine);
			value = lowPassFilter(value, m_prevEbsLine, 0.9f);
			m_prevEbsLine = value;
		}else if(pin == m_analogPinServiceTank){
    	value = (float)rawVal/((float) m_analogConvServiceTank)-((float) m_analogOffsetServiceTank);
			value = lowPassFilter(value, m_prevServiceTank, 0.9f);
			m_prevServiceTank = value;
		}else if(pin == m_analogPinEbsActuator){
    	value = (float)rawVal/((float) m_analogConvEbsActuator)-((float) m_analogOffsetEbsActuator);
			value = lowPassFilter(value, m_prevEbsActuator, 0.9f);
			m_prevEbsActuator = value;
		}else if(pin == m_analogPinPressureReg){
    	value = (float)rawVal/((float) m_analogConvPressureReg)-((float) m_analogOffsetPressureReg);
			value = lowPassFilter(value, m_prevPressureReg, 0.95f);
			m_prevPressureReg = value;
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
		// return converted value for displaying
		return value;	
}

void STM::sendBackDigital(cluon::OD4Session * od4Gpio, uint16_t pin, uint32_t val){
  cluon::data::TimeStamp sampleTime = cluon::time::now();
  opendlv::proxy::SwitchStateReading msg;
  msg.state((bool)val);
  int16_t senderStamp = pin + m_senderStampOffsetGpio;
  od4Gpio->send(msg, sampleTime, senderStamp);
}

bool STM::decodePayload(cluon::OD4Session* od4, cluon::OD4Session* od4Gpio, bool rackPos, bool steerPos, bool ebsLine, bool ebsAct, bool servTank, bool pressReg, bool asms, bool clamped, bool ebsOK)
{

	//std::cout << "m_Payloads.size() == " << m_Payloads.size() << std::endl;
  if(m_Payloads.size() > 0) // Send Analog readings
  {
    for(std::string payload : m_Payloads)
    { 
      //std::cout << "payload == " << payload << std::endl;
      bool newLine = false;   
      char* endptr;
      size_t pos = std::string::npos;
      unsigned int delimiterPos1;
      unsigned int rawVal;
      float value;
      unsigned int pin = -1;
      std::string pinFunc;
      std::map<std::string,int>::iterator it;
      
      // Analog inputs
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
      		if(ebsLine){ 
      		  //std::cout << "ebsLine: " << rawVal << " "; newLine = true;
      		}
      		rawEbsLine = sendBackAnalog(od4, pin, rawVal);

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
      		if(servTank){ 
      		 // std::cout << "servTank: " << rawVal << " "; newLine = true;
      		}
      		rawServiceTank = sendBackAnalog(od4, pin, rawVal);

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
      		if(ebsAct){ 
      		  //std::cout << "ebsAct: " << rawVal << " "; newLine = true;
      		}
      		rawEbsActuator = sendBackAnalog(od4, pin, rawVal);

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
      		if(pressReg){ 
      		  //std::cout << "pressReg: " << rawVal << " "; newLine = true;
      		}
      		rawPressureReg = sendBackAnalog(od4, pin, rawVal);

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
      		if(rackPos){ 
      		  //std::cout << "rackPos: " << rawVal << " "; newLine = true;
      		}
      		rawSteerPositionRack = sendBackAnalog(od4, pin, rawVal);

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
      		if(steerPos){ 
      		  //std::cout << "steerPos: " << rawVal << " "; newLine = true;
      		}
      		rawSteerPosition = sendBackAnalog(od4, pin, rawVal);

				}	
			}
			
			// Digital inputs
			pinFunc = ASMS;
      pos = payload.find(pinFunc);
      if(pos != std::string::npos) {   	
			  pin = m_gpioPinAsms;
				unsigned int delimiterPos1 = pos + pinFunc.length();
      	unsigned int rawVal = strtol(payload.c_str() + delimiterPos1 + 1, &endptr, 10);
      	if(asms){ 
      		//std::cout << "asms: " << rawVal << " "; newLine = true;
      	}
      	sendBackDigital(od4Gpio, pin, rawVal);
      	rawAsms = rawVal;
			}
			
			pinFunc = CLAMPED_SENSOR;
      pos = payload.find(pinFunc);
      if(pos != std::string::npos) {   	
			  pin = m_gpioPinClampSensor;
				unsigned int delimiterPos1 = pos + pinFunc.length();
      	unsigned int rawVal = strtol(payload.c_str() + delimiterPos1 + 1, &endptr, 10);
      	if(clamped){ 
      		//std::cout << "clamped: " << rawVal << " "; newLine = true;
      	}
      	sendBackDigital(od4Gpio, pin, rawVal);
      	rawClamped = rawVal;
			}
			
			pinFunc = EBS_OK;
      pos = payload.find(pinFunc);
      if(pos != std::string::npos) {   	
			  pin = m_gpioPinEbsOk;
				unsigned int delimiterPos1 = pos + pinFunc.length();
      	unsigned int rawVal = strtol(payload.c_str() + delimiterPos1 + 1, &endptr, 10);
      	if(ebsOK){ 
      		//std::cout << "ebsOK: " << rawVal << " "; newLine = true;
      	}
      	sendBackDigital(od4Gpio, pin, rawVal);
      	rawEbsOK = rawVal;
			}
			
			if(rackPos) {std::cout << "rackPos" << ": " << std::setprecision(2) << rawSteerPositionRack << " "; newLine=true;}
  if(steerPos) {std::cout << "steerPos" << ": " << std::setprecision(2) << rawSteerPosition << " "; newLine=true;}
  if(ebsLine) {std::cout << "ebsLine" << ": " << std::setprecision(2) << rawEbsLine << " "; newLine=true;}
  if(ebsAct) {std::cout << "ebsAct" << ": " << std::setprecision(2) << rawEbsActuator << " "; newLine=true;}
  if(servTank) {std::cout << "servTank" << ": " << std::setprecision(2) << rawServiceTank << " "; newLine=true;}
  if(pressReg) {std::cout << "pressReg" << ": " << std::setprecision(2) << rawPressureReg << " "; newLine=true;}
  if(asms) {std::cout << "asms" << ": " << rawAsms << " "; newLine=true;}
  if(clamped) {std::cout << "clamped" << ": " << rawClamped << " "; newLine=true;}
  if(ebsOK) {std::cout << "ebsOK" << ": " << rawEbsOK << " "; newLine=true;}
  if(newLine) std::cout << std::endl;
    }
    m_Payloads.clear();
  return true;
	}
	return false;
}
void STM::extractPayload()
{ //std::cout << "receiveBuffer.length() = " << receiveBuffer.length() << std::endl;
	if(receiveBuffer.length() > 3)
	{
	  char *colonSign = NULL;
		unsigned int lengthOfPayload = strtol(receiveBuffer.c_str(), &colonSign, 10);
		if (*colonSign == 0x3a) {
      //std::cout << "colonSign == " << colonSign << std::endl;
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
				if(start +  lengthOfPayload < receiveBuffer.length()){
				  std::string payload = receiveBuffer.substr(start,lengthOfPayload);
				  //std::cout << payload << std::endl;
				  if(m_Payloads.size() < 100) //Maximum payload storage
					  m_Payloads.push_back(payload);
				}
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
  return m_senderStampOffsetPwm;
}

void STM::viewAnalogRaw(bool rackPos, bool steerPos, bool ebsLine, bool ebsAct, bool servTank, bool pressReg){
  bool newLine = false;
  if(rackPos) {std::cout << "rackPos" << ": " << rawSteerPositionRack << " "; newLine=true;}
  if(steerPos) {std::cout << "steerPos" << ": " << rawSteerPosition << " "; newLine=true;}
  if(ebsLine) {std::cout << "ebsLine" << ": " << rawEbsLine << " "; newLine=true;}
  if(ebsAct) {std::cout << "ebsAct" << ": " << rawEbsActuator << " "; newLine=true;}
  if(servTank) {std::cout << "servTank" << ": " << rawServiceTank << " "; newLine=true;}
  if(pressReg) {std::cout << "pressReg" << ": " << rawPressureReg << " "; newLine=true;}
  if(newLine) std::cout << std::endl;
}

void STM::tearDown() 
{
}
//catch(...){ std::cout << "exception caught"<< std::endl;}

float STM::lowPassFilter(float newValue, float oldValue, float alpha)
{
	return alpha * newValue + (1.0f - alpha) * oldValue;
}