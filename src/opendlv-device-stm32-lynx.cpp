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
    const string deviceName{commandlineArguments["device"]};
    std::cout << "Micro-Service ID:" << ID << std::endl;
    std::cout << "FREQ: " << FREQ << std::endl;
    std::cout << "VERBOSE: " << VERBOSE << std::endl;
    //Interface to a running OpenDaVINCI session.
    cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
    cluon::OD4Session od4Gpio{static_cast<uint16_t>(std::stoi(commandlineArguments["cidGpio"]))};
    
    //Interface with STM32F4 Discovery board
    STM stm32(VERBOSE, ID);  
    //Creare serial port
    string port(deviceName);
    serial::Port myPort(port, 38400);
    if(VERBOSE){
      cout << "Serial port: " << port << " created\n";
      std::cout << "Device name:" << deviceName << std::endl;
    }

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
	
    /* --- Sending request to STM32 --- */
    auto atFrequency{[&od4, &stm32]() -> bool
    {
      
    }};
    od4.timeTrigger(FREQ, atFrequency);
    
    /* --- DECODE BYTES FROM STM32F4 --- */
    auto readingCallback = [&od4, &stm32](const string data){
      stm32.decode(od4, data);
    };
    myPort.read(readingCallback);
	
    while(1) {
    //Sleep as this service is data driven
        usleep(50);
        stm32.send(&myPort);
    }

  }
	cout << "Exit service\n";
  return retCode;
}
