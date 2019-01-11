# opendlv-device-stm32f4-lynx

### How it works
- This microservice collects GPIO/PWM write requests sent from other microservices, and converts and sends these requests as netstring to STM32F4 Discovery board. It also receives measurements from STM32 (Analog/digital) and send back to other services.

### Dependencies
- STM32F4 running a serial-over-usb connection, and is connected to host machine, and appears as /dev/ttyACM0
- STM32 firmware can be found at https://github.com/chalmersfsd/stm32-service-node/tree/nam-development

### Build
AMD64:
docker build -f Dockerfile.amd64 -t opendlv-device-stm32-lynx-alpine:v0.0.3 .

### OD4Session message in and out
- recieve:
  - opendlv::proxy::SwitchStateRequest
  - opendlv::proxy::PulseWidthModulationRequest
- send:
  - opendlv::proxy::PressureReading
  - opendlv::proxy::GroundSteeringReading
  - opendlv::proxy::VoltageReading


### Command line arguments
| parameter | comments |
| ----- | ----- |
| cid | OpenDaVINCI v4 session identifier [1..254] for low-level 0D4 messages|
| cidGpio | OpenDaVINCI v4 session identifier [1..254] for receiving GPIO requests |
| cidpwm  | OpenDaVINCI v4 session identifier [1..254] for receiving PWM requests |
| freq | general frequency (not used for now) |
| id | Identifier in case of multiple STM32 used |
| device | absolute path to name of STM32 as it appears in host machine when being connected |
- example usage:

    opendlv-device-stm32-lynx --cid=219 --cidGpio=220 --cidpwm=222 --freq=30 -id=1 --device=/dev/ttyACM0 --verbose


### To-do
- Add digital input from STM32
- Re-calibrate sensor measurements
- Adjust indentation for better codes
