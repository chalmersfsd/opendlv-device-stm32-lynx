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

#include <iostream>
#include <string>

#include <thread>

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include "proxy-stm32.hpp"

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if ((0 == commandlineArguments.count("freq")) 
      || (0 == commandlineArguments.count("cid"))) {
    std::cerr << argv[0] << " : Module handling communication between Lynx and "
     << "STM32F4" << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> "
      << "[--id=<Identifier in case of multiple STM32F4 units>] [--verbose]" 
      << std::endl;
    std::cerr << "Example: " << argv[0] << " ADD EXAMPLE HERE" << std::endl;
    retCode = 1;
  } else {
    uint32_t const ID{(commandlineArguments["id"].size() != 0) ?
      static_cast<uint32_t>(std::stoi(commandlineArguments["id"])) : 0};
    bool const VERBOSE{commandlineArguments.count("verbose") != 0};
    float const FREQ{std::stof(commandlineArguments["freq"])};
    uint32_t const BAUDRATE{38400};
    
    std::string const deviceName{commandlineArguments["device"]};
    int64_t const periodTimeUs = static_cast<int64_t>(1000000.0 / FREQ);

    cluon::OD4Session od4{
      static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

    STM stm(ID, VERBOSE);  
    std::mutex stmMutex;

    std::cout << "Microservice ID:" << ID << std::endl;
    std::cout << "Frequency: " << FREQ << std::endl;
    std::cout << "Serial port (" << deviceName << ") created." << std::endl;

    auto onSwitchStateRequest{[&stm, &stmMutex](
        cluon::data::Envelope &&envelope)
      {
        auto  gpioState = 
          cluon::extractMessage<opendlv::proxy::SwitchStateRequest>(
              std::move(envelope));
        {
          std::lock_guard<std::mutex> lock(stmMutex);
          int16_t pin = envelope.senderStamp() - stm.getSenderStampOffsetGpio();
          bool value = gpioState.state();
          stm.collectRequests("gpio", pin, value);
        }
      }};
    od4.dataTrigger(opendlv::proxy::SwitchStateRequest::ID(),
        onSwitchStateRequest);

    auto onPulseWidthModulationRequest{[&stm, &stmMutex](
        cluon::data::Envelope &&envelope)
      {
        auto pwmState = 
          cluon::extractMessage<opendlv::proxy::PulseWidthModulationRequest>(
              std::move(envelope));
        {
          std::lock_guard<std::mutex> lock(stmMutex);
          int16_t pin = envelope.senderStamp() - stm.getSenderStampOffsetPwm();
          uint32_t value = pwmState.dutyCycleNs();
          stm.collectRequests("pwm", pin, value);
        }
      }};
    od4.dataTrigger(opendlv::proxy::PulseWidthModulationRequest::ID(),
        onPulseWidthModulationRequest);

    serial::Serial serial(deviceName, BAUDRATE);

    while (serial.isOpen()) {
    
      cluon::data::TimeStamp lastStatusRequest{cluon::time::now()};
      int64_t lastStatusRequestUs{
        cluon::time::toMicroseconds(lastStatusRequest)};
      {
        std::lock_guard<std::mutex> lock(stmMutex);
        stm.send(serial);
        stm.sendStatusRequest(serial);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      {
        std::string data = serial.read(static_cast<size_t>(2048));
        std::lock_guard<std::mutex> lock(stmMutex);
        stm.addData(data);
        stm.decode(od4);
      }
      
      serial.flush();
      
      int64_t const leftInTimeSliceUs{
        periodTimeUs - (cluon::time::toMicroseconds(cluon::time::now()) - 
        lastStatusRequestUs)};
        
      std::this_thread::sleep_for(std::chrono::microseconds(leftInTimeSliceUs));
      if (leftInTimeSliceUs < 0) {
        std::cout << "Warning: violated time slice by " 
          << leftInTimeSliceUs << " microseconds." << std::endl;
      }
    }

    return retCode;
  }
}
