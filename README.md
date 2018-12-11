# opendlv-device-stm32f4-lynx

### How it works
- This microservice collects GPIO/PWM write requests sent from other microservices, and converts and sends these requests as netstring to STM32F4 Discovery board
- Currently, assume that STM32 is constantly sending "a.1.<analog measurement (4-byte int)>\n", this would be intepreted as "Pin 1 with analog measurements of 0-4096", which as intepreted as a Ebs line pressure by current state machine
- The request messages sent to this microservice are stored in vectors, and would be converted and sent to STM32 sequentially (to be done later)

### Dependencies
- STM32F4 running a serial-over-usb connection, and is connected to host machine, and appears as /dev/ttyACM0, otherwise the microservice would exit
- STM32 is constantly sending "a.1.<analog measurement (4-byte string)>\n"

### Build
AMD64:
docker build -f Dockerfile.amd64 -t opendlv-device-stm32-lynx-alpine:v0.0.3 .

### OD4Session message in and out
- recieve:
  - opendlv::proxy::SwitchStateRequest
- send:
  - opendlv::proxy::PressureReading


### Command line arguments
| parameter | comments |
| ----- | ----- |
| cid | OpenDaVINCI v4 session identifier [1..254] for low-level 0D4 messages|
| cidGpio | OpenDaVINCI v4 session identifier [1..254] for receiving GPIO requests |
| freq | general frequency (not used for now) |
| id | Identifier in case of multiple STM32 used |
| device | absolute path to name of STM32 as it appears in host machine when being connected |
- example usage:

    opendlv-device-stm32-lynx --cid=219 --cidGpio=220 --freq=30 -id=1 --device=/dev/ttyACM0 --verbose


### To-do
- Reimplement the send-receive operation using netstring and with protocol agreed with Max
- Include more signals
