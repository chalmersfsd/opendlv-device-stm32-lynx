/**
 * Copyright (C) 2017 Chalmers Revere
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
 * USA.
 */

#ifndef REQUEST_HPP
#define REQUEST_HPP
#include <string>
class Request{
  public:
    Request(std::string type, unsigned int pin, int value);
    ~Request() = default;
		//time stamp
		std::string m_type; // type of request: "gpio"/"pwm"
		unsigned int m_pin; // pin identifier for STM32
		int m_value; // request value
};

#endif

