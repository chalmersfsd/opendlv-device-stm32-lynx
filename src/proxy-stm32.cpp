/*
 * Copyright (C) 2019 Nam Vu, Dan Andersson
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

#include <iostream>

Request::Request(std::string a_type, uint32_t a_pin, int32_t a_value)
  : type(a_type)
  , pin(a_pin)
  , value(a_value)
{
}

STM::STM(uint32_t id, bool verbose)
  : m_receiveBuffer()
  , m_senderStampOffsetAnalog(id * 1000 + 200)
  , m_senderStampOffsetGpio(id * 1000)
  , m_senderStampOffsetPwm(id * 1000 + 300)
  , m_verbose(verbose)
{
  m_gpioPins[65] = CLAMP_SET;
  m_gpioPins[45] = COMPRESSOR;
  m_gpioPins[61] = EBS_RELIEF;
  m_gpioPins[44] = EBS_SPEAKER;
  m_gpioPins[66] = FINISHED;
  m_gpioPins[27] = HEART_BEAT;	
  m_gpioPins[46] = RACK_RIGHT;
  m_gpioPins[47] = RACK_LEFT;
  m_gpioPins[69] = SERVICE_BREAK;
  m_gpioPins[68] = REDUNDENCY;
  m_gpioPins[67] = SHUTDOWN;
  m_gpioPins[62] = SPARE;

  m_gpioPins[49] = EBS_OK;
  m_gpioPins[112] = CLAMPED_SENSOR;
  m_gpioPins[115] = ASMS;

  m_pwmPins[40] = STEER_SPEED;
  m_pwmPins[41] = BRAKE_PRESSURE;
  m_pwmPins[0] = ASSI_BLUE;
  m_pwmPins[20] = ASSI_RED;
  m_pwmPins[21] = ASSI_GREEN;

  m_analogPins[STEER_POS] = 0;
  m_analogPins[EBS_LINE] = 1;
  m_analogPins[SERVICE_TANK] = 2;
  m_analogPins[EBS_ACTUATOR] = 3;
  m_analogPins[PRESSURE_RAG] = 5;
  m_analogPins[POSITION_RACK] = 6;
}

STM::~STM()
{
}

void STM::collectRequests(std::string type, uint32_t pin, int32_t value)
{
  uint32_t const maxRequest = 100;

  Request request(type, pin, value);
  if (type == "gpio" && m_gpioRequest.size() < maxRequest) {
    m_gpioRequest.push(request);
    if (m_verbose) {
      std::cout << "Number of GPIO requets in queue: " << m_gpioRequest.size() 
        << std::endl;
    }
  } else if (type == "pwm" && m_pwmRequest.size() < maxRequest) {
    m_pwmRequest.push(request);
    if (m_verbose) {
      std::cout << "Number of PWM requets in queue: " << m_pwmRequest.size() 
        << std::endl;
    }
  }
}

void STM::send(serial::Serial &serial)
{
  while (m_gpioRequest.size() > 0) {
    std::string payload = encodePayload("gpio", m_gpioRequest.front());
    if (!payload.empty()) {
      int32_t result = sendWithAck(serial, payload);
      if (result == -1) {
        std::cout << "[STM32 Proxy]: Error sending " 
          << encodeNetstring(payload) << " : no ACK from STM32" << std::endl;
      }
    }
    m_gpioRequest.pop();
  }

  while (m_pwmRequest.size() > 0) {
    std::string payload = encodePayload("pwm", m_pwmRequest.front());
    if (!payload.empty()) {
      int32_t result = sendWithAck(serial, payload);
      if(result == -1) {
        std::cout << "[STM32 Proxy]: Error sending "
          << encodeNetstring(payload) << " : no ACK from STM32" << std::endl;
      }
    }
    m_pwmRequest.pop();
  }
}

int32_t STM::sendWithAck(serial::Serial &serial, std::string payload)
{
  std::string netstringMsg = encodeNetstring(payload);
  serial.write(netstringMsg);
  uint32_t result = 0;
  uint32_t resendAttempts = 3;
  while (result == 0) {
    // Wait for STM to send back an ACK/NACK message
    if (serial.waitReadable()) {
      std::string resultMsg = serial.read(static_cast<size_t>(128));
      if (resultMsg.find(payload + "|ACK") != std::string::npos) {
        result = 1;
      } else {
        serial.write(netstringMsg);
        resendAttempts--;
      }
    }
    if (resendAttempts == 0) {
      // Error, did not get ack
      result = -1;
    }
  }
  return result;
}

std::string STM::encodeNetstring(const std::string payload)
{
  return std::to_string(payload.length()) + ":" + payload + MSG_END;
}

std::string STM::encodePayload(std::string type, Request const &request)
{
  uint32_t pin = request.pin;
  int32_t value = request.value;
  std::map<int, std::string>::iterator it;

  if (type == "gpio") {
    it = m_gpioPins.find(pin);
    if (it == m_gpioPins.end()) {
      // Error, cannot find requested pin.
      return "";
    } else {
      std::string payload = std::string(SET) + DELIMITER + it->second 
        + DELIMITER + std::to_string(value);
      return payload;
    }
  } else if (type == "pwm") {
    it = m_pwmPins.find(pin);
    if (it == m_pwmPins.end()) {
      // Error, cannot find requested pin.
      return "";
    } else {
      if (value >= 100000) { 
        // steer right, convert to a negative pwm request
        value = value - 100000;
        // this is an offset added in logic-lynx-steering for distinguishing
        // negative numbers (since standard pwm request is non-negative)
        value = -value;
      }
      // Convert old pwm values (0-50000) to duty cycle (0-100.00 percent)
      int32_t dutyCycle = static_cast<int32_t>(
          (static_cast<float>(value) / 50000.0f) * 10000);
      std::string payload = std::string(SET) + DELIMITER + it->second
        + DELIMITER + std::to_string(dutyCycle);
      return payload;
    }
  }
  return "";
}

void STM::sendStatusRequest(serial::Serial &serial)
{
  std::string payload = "get";
  std::string netstringMsg = encodeNetstring(payload);
  serial.write(netstringMsg);
}

float STM::sendAnalog(cluon::OD4Session &od4, uint16_t pin, uint32_t rawInt)
{
  uint16_t const analogPinSteerPosition{0};
  uint16_t const analogPinSteerPositionRack{6};
  uint16_t const analogPinServiceTank{2};
  uint16_t const analogPinPressureReg{5};
  uint16_t const analogPinEbsLine{1};
  uint16_t const analogPinEbsActuator{3};
    
  float const analogConvSteerPosition{80.38f};
  float const analogConvEbsLine{378.5f};
  float const analogConvServiceTank{377.6f};
  float const analogConvEbsActuator{377.9f};
  float const analogConvPressureReg{378.7f};
  float const analogConvSteerPositionRack{80.86f};
              
  float const analogOffsetSteerPosition{27.74f};
  float const analogOffsetEbsLine{0.11f + 1.6f};
  float const analogOffsetServiceTank{0.11f};
  float const analogOffsetEbsActuator{0.11f};
  float const analogOffsetPressureReg{0.0f};
  float const analogOffsetSteerPositionRack{31.06f};

  cluon::data::TimeStamp sampleTime{cluon::time::now()};
  int16_t senderStamp = pin + m_senderStampOffsetAnalog;
  
  // Convert raw readings (0-4019) to actual measurements (mm, bar)		
  // STM32 has 12bit ADC 0-3.3V, thus need to re-scale the raw value
  float rawValue = 3.3f * rawInt / 1.8f;

  float value = 0.0;
  switch (pin) {
    case analogPinSteerPosition:
      {
        value = rawValue / analogConvSteerPosition 
          - analogOffsetSteerPosition;
        break;
      }
    case analogPinEbsLine:
      {
        value = rawValue / analogConvEbsLine 
          - analogOffsetEbsLine;
        value = lowpassFilter(value, m_prevEbsLine, 0.9f);
        m_prevEbsLine = value;
        break;
      }
    case analogPinServiceTank:
      {
        value = rawValue / analogConvServiceTank 
          - analogOffsetServiceTank;
        value = lowpassFilter(value, m_prevServiceTank, 0.9f);
        m_prevServiceTank = value;
        break;
      }
    case analogPinEbsActuator:
      {
        value = rawValue / analogConvEbsActuator 
          - analogOffsetEbsActuator;
        value = lowpassFilter(value, m_prevEbsActuator, 0.9f);
        m_prevEbsActuator = value;
        break;
      }
    case analogPinPressureReg:
      {
        value = rawValue / analogConvPressureReg
          - analogOffsetPressureReg;
        value = lowpassFilter(value, m_prevPressureReg, 0.95f);
        m_prevPressureReg = value;
        break;
      }
    case analogPinSteerPositionRack:
      {
        value = rawValue / analogConvSteerPositionRack
          - analogOffsetSteerPositionRack;
        break;
      }
    default:
      {
      }
  }

  if (pin == analogPinSteerPosition || pin == analogPinSteerPositionRack) {
    opendlv::proxy::GroundSteeringReading msg;
    msg.groundSteering(value);
    od4.send(msg, sampleTime, senderStamp);
  } else if (pin == analogPinEbsLine || pin == analogPinServiceTank 
      || pin == analogPinEbsActuator || pin == analogPinPressureReg) {
    opendlv::proxy::PressureReading msg;
    msg.pressure(value);
    od4.send(msg, sampleTime, senderStamp);
  } else {
    opendlv::proxy::VoltageReading msg;
    msg.voltage(value);
    od4.send(msg, sampleTime, senderStamp);
  }

  return value;	
}

void STM::sendDigital(cluon::OD4Session &od4, uint16_t pin, bool val) {
  cluon::data::TimeStamp sampleTime{cluon::time::now()};
  opendlv::proxy::SwitchStateReading msg;
  msg.state(val);
  int16_t senderStamp = pin + m_senderStampOffsetGpio;
  od4.send(msg, sampleTime, senderStamp);
}

bool STM::decode(cluon::OD4Session &od4, std::string data)
{
  uint32_t bufferSize = 2048;
  if (m_receiveBuffer.length() <= bufferSize) {
    std::stringstream ss;
    ss << m_receiveBuffer << data;
    m_receiveBuffer = ss.str();
  }

  std::vector<std::string> payloads;
  while (m_receiveBuffer.length() > 3) {
    char *colonSign = nullptr;

    uint32_t lengthOfPayload = strtol(m_receiveBuffer.c_str(), &colonSign, 10);
    if (*colonSign == 0x3a) {
      // Found colon sign.
      // First, check if the buffer is as long as it is stated in the netstring.
      // This prevents the case where the colon is near the end of string,
      // which can lead to out of range access later
      if (m_receiveBuffer.length() - (colonSign + 1 - m_receiveBuffer.c_str()) 
          < lengthOfPayload) {
        // Received data is too short. Skip further processing this part.
        break;
      }

      // Now, check if (m_receiveBuffer + 1 + lengthOfPayload) == MSG_END.
      if ((colonSign[1 + lengthOfPayload]) == MSG_END) {
        // Successfully found a complete Netstring.
        int32_t start = colonSign + 1 - m_receiveBuffer.c_str();
        if (start + lengthOfPayload < m_receiveBuffer.length()) {
          std::string payload = m_receiveBuffer.substr(start, lengthOfPayload);
          if(payloads.size() < 100) {
            payloads.push_back(payload);
          } else {
            // Error, dropped payload
          }
        }

        // Remove decoded Netstring from m_receiveBuffer
        char *msgEndPtr =  colonSign + 1 + lengthOfPayload;
        int32_t numberOfCharToRemove = msgEndPtr + 1 - m_receiveBuffer.c_str(); 
        m_receiveBuffer = m_receiveBuffer.erase(0, numberOfCharToRemove);
      } else {
        // The message is corrupted (missing bytes), remove this message from 
        // buffer
        // Shift the bytes in buffer so that buffer starts with payload size
        // (otherwise strtol won't work) 
        size_t endPos = m_receiveBuffer.find(";");
        if (endPos != std::string::npos 
            && endPos + 1 < m_receiveBuffer.length()) {
          m_receiveBuffer = m_receiveBuffer.substr(endPos + 1, 
              m_receiveBuffer.length() - endPos);
        }
      }
    } else {
      // Shift the bytes in buffer so that buffer starts with payload size
      // The message is corrupted (missing bytes), remove this message from 
      // buffer
      size_t endPos = m_receiveBuffer.find(";");
      if(endPos != std::string::npos 
          && endPos + 1 < m_receiveBuffer.length()) {
        m_receiveBuffer = m_receiveBuffer.substr(endPos + 1,
            m_receiveBuffer.length() - endPos);
      }
    }
  }

  if (payloads.empty()) {
    return false;
  }

  for(std::string payload : payloads) { 
    size_t pos = std::string::npos;
    std::string pinFunc;
    std::map<std::string,int>::iterator it;

    // Analog inputs
    pinFunc = EBS_LINE;
    pos = payload.find(pinFunc);
    if (pos != std::string::npos) {   	
      it = m_analogPins.find(pinFunc);
      if (it != m_analogPins.end()) {
        uint32_t pin = it->second;
        uint32_t delimiterPos1 = pos + pinFunc.length();
        uint32_t rawVal = strtol(payload.c_str() + delimiterPos1 + 1, nullptr,
            10);
        sendAnalog(od4, pin, rawVal);
      }
    }

    pinFunc = SERVICE_TANK;
    pos = payload.find(pinFunc);
    if (pos != std::string::npos) {   	
      it = m_analogPins.find(pinFunc);
      if (it != m_analogPins.end()) {
        uint32_t pin = it->second;
        uint32_t delimiterPos1 = pos + pinFunc.length();
        uint32_t rawVal = strtol(payload.c_str() + delimiterPos1 + 1, nullptr,
            10);
        sendAnalog(od4, pin, rawVal);
      }	
    }

    pinFunc = EBS_ACTUATOR;
    pos = payload.find(pinFunc);
    if (pos != std::string::npos) {   	
      it = m_analogPins.find(pinFunc);
      if (it != m_analogPins.end()) {
        uint32_t pin = it->second;
        uint32_t delimiterPos1 = pos + pinFunc.length();
        uint32_t rawVal = strtol(payload.c_str() + delimiterPos1 + 1, nullptr,
            10);
        sendAnalog(od4, pin, rawVal);
      }	
    }

    pinFunc = PRESSURE_RAG;
    pos = payload.find(pinFunc);
    if (pos != std::string::npos) {
      it = m_analogPins.find(pinFunc);
      if (it != m_analogPins.end()) {
        uint32_t pin = it->second;
        uint32_t delimiterPos1 = pos + pinFunc.length();
        uint32_t rawVal = strtol(payload.c_str() + delimiterPos1 + 1, nullptr,
            10);
        sendAnalog(od4, pin, rawVal);
      }	
    }

    pinFunc = POSITION_RACK;
    pos = payload.find(pinFunc);
    if (pos != std::string::npos) {
      it = m_analogPins.find(pinFunc);
      if (it != m_analogPins.end()) {
        uint32_t pin = it->second;
        uint32_t delimiterPos1 = pos + pinFunc.length();
        uint32_t rawVal = strtol(payload.c_str() + delimiterPos1 + 1, nullptr,
            10);
        sendAnalog(od4, pin, rawVal);
      }	
    }

    pinFunc = STEER_POS;
    pos = payload.find(pinFunc);
    if (pos != std::string::npos) {
      it = m_analogPins.find(pinFunc);
      if (it != m_analogPins.end()) {
        uint32_t pin = it->second;
        uint32_t delimiterPos1 = pos + pinFunc.length();
        uint32_t rawVal = strtol(payload.c_str() + delimiterPos1 + 1, nullptr,
            10);
        sendAnalog(od4, pin, rawVal);
      }	
    }

    // Digital inputs
    uint16_t const gpioPinAsms{115};
    uint16_t const gpioPinEbsOk{49};
    uint16_t const gpioPinClampSensor{112};

    pinFunc = ASMS;
    pos = payload.find(pinFunc);
    if (pos != std::string::npos) {
      uint32_t pin = gpioPinAsms;
      uint32_t delimiterPos1 = pos + pinFunc.length();
      uint32_t rawVal = strtol(payload.c_str() + delimiterPos1 + 1, nullptr,
          10);
      sendDigital(od4, pin, rawVal);
    }

    pinFunc = CLAMPED_SENSOR;
    pos = payload.find(pinFunc);
    if (pos != std::string::npos) {
      uint32_t pin = gpioPinClampSensor;
      uint32_t delimiterPos1 = pos + pinFunc.length();
      uint32_t rawVal = strtol(payload.c_str() + delimiterPos1 + 1, nullptr,
          10);
      sendDigital(od4, pin, rawVal);
    }

    pinFunc = EBS_OK;
    pos = payload.find(pinFunc);
    if (pos != std::string::npos) {
      uint32_t pin = gpioPinEbsOk;
      uint32_t delimiterPos1 = pos + pinFunc.length();
      uint32_t rawVal = strtol(payload.c_str() + delimiterPos1 + 1, nullptr,
          10);
      sendDigital(od4, pin, rawVal);
    }
  }
  return true;
}

uint32_t STM::getSenderStampOffsetGpio(){
  return m_senderStampOffsetGpio;
}

uint32_t STM::getSenderStampOffsetPwm(){
  return m_senderStampOffsetPwm;
}

float STM::lowpassFilter(float newValue, float oldValue, float alpha)
{
  return alpha * newValue + (1.0f - alpha) * oldValue;
}
