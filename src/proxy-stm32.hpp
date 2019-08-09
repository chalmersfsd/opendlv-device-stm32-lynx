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

#ifndef STM32
#define STM32

/*===========================================================================*/
/* Docker-STM32 protocol related stuff.                                      */
/*===========================================================================*/
#define DELIMITER  '|'
#define MSG_END  ';'
#define SET  "set"

/* --- status_msgs --- */
#define ASMS  "asms"
#define CLAMPED_SENSOR  "clamped_sensor"
#define EBS_ACTUATOR  "ebs_actuator"
#define EBS_LINE  "ebs_line"
#define EBS_OK  "ebs_ok"
#define POSITION_RACK  "position_rack"
#define PRESSURE_RAG  "pressure_rag"
#define SERVICE_TANK  "service_tank"
#define STEER_POS  "steer_pos"

/* --- setter_msgs --- */
#define CLAMP_SET  "clamp_set"
#define COMPRESSOR  "compressor"
#define EBS_RELIEF  "ebs_relief"
#define EBS_SPEAKER  "ebs_speaker"
#define FINISHED  "finished"
#define HEART_BEAT  "heart_beat"
#define SERVICE_BREAK  "service_break"
#define SHUTDOWN  "shutdown"
#define RACK_LEFT  "rack_left"
#define RACK_RIGHT "rack_right"
#define REDUNDENCY "redundency"
#define SPARE "spare"
#define AUTO_MODE "auto_mode"

#define STEER_SPEED "steer_speed"
#define BRAKE_PRESSURE "brake_pressure"
#define ASSI_BLUE "assi_blue"
#define ASSI_RED "assi_red"
#define ASSI_GREEN "assi_green"

#include <string>
#include <vector>
#include <map>
#include <queue>

#include <signal.h>
#include <serialport.hpp>

#include "opendlv-standard-message-set.hpp"

struct Request{
  std::string type; // "gpio"/"pwm"
  uint32_t pin; // pin for STM32
  int32_t value;

  Request(std::string, uint32_t, int32_t);
};

class STM {
  public:
    STM(uint32_t, bool);
    ~STM();

    void addData(std::string const &);
    void collectRequests(std::string, uint32_t, int32_t);
    bool decode(cluon::OD4Session &);
    uint32_t getSenderStampOffsetGpio();
    uint32_t getSenderStampOffsetPwm();
    void send(serial::Serial &);
    void sendStatusRequest(serial::Serial &);

  private:
    std::string encodeNetstring(const std::string);
    std::string encodePayload(std::string, Request const &);
    float lowpassFilter(float, float, float);
    float sendAnalog(cluon::OD4Session &, uint16_t, uint32_t);
    void sendDigital(cluon::OD4Session &, uint16_t, bool);
    int32_t sendWithAck(serial::Serial &, std::string);

    std::map<int32_t, std::string> m_gpioPins;
    std::map<int32_t, std::string> m_pwmPins;
    std::map<std::string, int32_t> m_analogPins;

    std::queue<Request> m_gpioRequest;
    std::queue<Request> m_pwmRequest;

    std::string m_receiveBuffer;

    uint32_t m_senderStampOffsetAnalog;
    uint32_t m_senderStampOffsetGpio;
    uint32_t m_senderStampOffsetPwm;

    float m_prevEbsLine{0.0f};
    float m_prevEbsActuator{0.0f};
    float m_prevServiceTank{0.0f};
    float m_prevPressureReg{0.0f};

    bool m_verbose;
};

#endif
