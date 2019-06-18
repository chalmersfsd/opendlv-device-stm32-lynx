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
#include "opendlv-standard-message-set.hpp"

#include <cstdio>
#include <iostream>
#include <string>
#include <sstream>
#include <unistd.h>

#include <thread>
#include <cmath>
#include <ctime>
#include <chrono>

#include "proxy-stm32.hpp"
using std::string;
using std::stringstream;
using std::cout;
using std::cerr;
using std::endl;

int main(int argc, char **argv) {
    int32_t retCode{0};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ((0 == commandlineArguments.count("freq")) || (0 == commandlineArguments.count("cid"))) {
    	std::cerr << argv[0] << " : Module handling communication between Lynx and STM32F4" << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDaVINCI session> [--id=<Identifier in case of multiple STM32F4 units>] [--verbose]" << std::endl;
        std::cerr << "Example: " << argv[0] << " ADD EXAMPLE HERE" << std::endl;
        retCode = 1;
    } else {
    const uint32_t ID{(commandlineArguments["id"].size() != 0) ? static_cast<uint32_t>(std::stoi(commandlineArguments["id"])) : 0};
    const bool VERBOSE{commandlineArguments.count("verbose") != 0};
    const float FREQ{std::stof(commandlineArguments["freq"])};
    //Options for displaying analog and digital inputs
    const bool rackPos{commandlineArguments.count("rackPos") != 0};
    const bool steerPos{commandlineArguments.count("steerPos") != 0};
    const bool ebsLine{commandlineArguments.count("ebsLine") != 0};
    const bool ebsAct{commandlineArguments.count("ebsAct") != 0};
    const bool servTank{commandlineArguments.count("servTank") != 0};
    const bool pressReg{commandlineArguments.count("pressReg") != 0};
    const bool asms{commandlineArguments.count("asms") != 0};
    const bool clamped{commandlineArguments.count("clamped") != 0};
    const bool ebsOK{commandlineArguments.count("ebsOK") != 0};
    
    const string deviceName{commandlineArguments["device"]};
    std::cout << "Micro-Service ID:" << ID << std::endl;
    std::cout << "FREQ: " << FREQ << std::endl;
    std::cout << "VERBOSE: " << VERBOSE << std::endl;
    //Interface to a running OpenDaVINCI session.
    cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
    cluon::OD4Session od4Gpio{static_cast<uint16_t>(std::stoi(commandlineArguments["cidGpio"]))};
    cluon::OD4Session od4Pwm{static_cast<uint16_t>(std::stoi(commandlineArguments["cidpwm"]))};
    
    //Interface with STM32F4 Discovery board
    STM stm32(VERBOSE, ID);  
        
    auto triggerSendStatusRequestThread{[&stm32, &FREQ]()
    {
      using namespace std::literals::chrono_literals;
      std::chrono::system_clock::time_point threadTime = std::chrono::system_clock::now();
      while (true) {
        std::this_thread::sleep_until(std::chrono::duration<double>(1/FREQ)+threadTime);
        threadTime = std::chrono::system_clock::now();
        
        stm32.readOK = true;
      }
    }};
    std::thread statusThread(triggerSendStatusRequestThread);
    
    //Creare serial port
    string port(deviceName);
    const uint32_t BAUDRATE{38400};
    const uint32_t TIMEOUT{1}; //5 ms timeout
    const uint16_t BUFFER_SIZE{2048};
    /*
    uint8_t *data = new uint8_t[BUFFER_SIZE];
    size_t size{0};
    */
    serial::Serial myPort(port, BAUDRATE, serial::Timeout::simpleTimeout(TIMEOUT));
    if(VERBOSE){
      cout << "Serial port: " << port << " created\n";
      std::cout << "Device name:" << deviceName << std::endl;
    }
    // flush input & output buffers
    myPort.flush();

     /* --- Collect gpio request --- */
    auto onSwitchStateRequest{[&od4Gpio, &stm32, &myPort](cluon::data::Envelope &&envelope)
    {
      auto  gpioState = cluon::extractMessage<opendlv::proxy::SwitchStateRequest>(std::move(envelope));
      int16_t pin = envelope.senderStamp()-stm32.getSenderStampOffsetGpio();
      bool value = gpioState.state();
			stm32.collectRequests("gpio", pin, value);
    }};
    od4Gpio.dataTrigger(opendlv::proxy::SwitchStateRequest::ID(), onSwitchStateRequest);

    /* --- Collect pwm request --- */
	  auto onPulseWidthModulationRequest{[&od4Gpio, &stm32, &myPort](cluon::data::Envelope &&envelope)
    {
      auto pwmState = cluon::extractMessage<opendlv::proxy::PulseWidthModulationRequest>(std::move(envelope));
      int16_t pin = envelope.senderStamp()-stm32.getSenderStampOffsetPwm();
      uint32_t value = pwmState.dutyCycleNs();
			stm32.collectRequests("pwm", pin, value);
			//std::cout << "Received PWM request: " << pin << ", value: " << value << std::endl;
    }};
    od4Pwm.dataTrigger(opendlv::proxy::PulseWidthModulationRequest::ID(), onPulseWidthModulationRequest);
    int toRead=0;
    while(myPort.isOpen()) {
      
      if(stm32.readOK){ //if no bytes in buffer and triggered at read frequency, then write get status request to stm
        stm32.SendStatusRequestToSTM(&myPort);
        stm32.readOK = false; 
        toRead+=1;
      }
      // wait until stm responds with measurements
      if(toRead>0){ //each readings does not need to be checked everytime in the loop.
        if(myPort.waitReadable()){// and it gives the priority to the readings because sending mutiple requests will block the bus.
          std::string data = myPort.read((size_t)256);
          stm32.GetBytesFromSTM(data);
          stm32.extractPayload();
          if(stm32.decodePayload(&od4, &od4Gpio, rackPos, steerPos, ebsLine, ebsAct, servTank, pressReg, asms, clamped, ebsOK)){
            toRead -= 1;
          }
        }
      }     
      else{
        // Send gpio/pwm requests with ACK checks
       if(!myPort.available()){
          stm32.send(&myPort);
       }
      }
      // flush input & output buffers
      myPort.flush();
    }
	cout << "Exit service\n";
  return retCode;
  }
}
