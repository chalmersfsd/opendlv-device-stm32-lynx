/*!
 * \file serial/serial.h
 * \author  William Woodall <wjwwood@gmail.com>
 * \author  John Harrison   <ash.gti@gmail.com>
 * \version 0.1
 *
 * \section LICENSE
 *
 * The MIT License
 *
 * Copyright (c) 2012 William Woodall
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * \section DESCRIPTION
 *
 * This provides a cross platform interface for interacting with Serial Ports.
 */

#ifndef SERIAL_H
#define SERIAL_H

#include <limits>
#include <vector>
#include <string>
#include <cstring>
#include <sstream>
#include <exception>
#include <stdexcept>

// This header is from the v8 google project:
// http://code.google.com/p/v8/source/browse/trunk/include/v8stdint.h

// Copyright 2012 the V8 project authors. All rights reserved.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//     * Neither the name of Google Inc. nor the names of its
//       contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Load definitions of standard types.

#ifndef V8STDINT_H_
#define V8STDINT_H_

#include <stddef.h>
#include <stdio.h>

#if defined(_WIN32) && !defined(__MINGW32__)

typedef signed char int8_t;
typedef unsigned char uint8_t;
typedef short int16_t;  // NOLINT
typedef unsigned short uint16_t;  // NOLINT
typedef int int32_t;
typedef unsigned int uint32_t;
typedef __int64 int64_t;
typedef unsigned __int64 uint64_t;
// intptr_t and friends are defined in crtdefs.h through stdio.h.

#else

#include <stdint.h>

#endif

#endif  // V8STDINT_H_

#define THROW(exceptionClass, message) throw exceptionClass(__FILE__, \
__LINE__, (message) )

namespace serial {

/*!
 * Enumeration defines the possible bytesizes for the serial port.
 */
typedef enum {
  fivebits = 5,
  sixbits = 6,
  sevenbits = 7,
  eightbits = 8
} bytesize_t;

/*!
 * Enumeration defines the possible parity types for the serial port.
 */
typedef enum {
  parity_none = 0,
  parity_odd = 1,
  parity_even = 2,
  parity_mark = 3,
  parity_space = 4
} parity_t;

/*!
 * Enumeration defines the possible stopbit types for the serial port.
 */
typedef enum {
  stopbits_one = 1,
  stopbits_two = 2,
  stopbits_one_point_five
} stopbits_t;

/*!
 * Enumeration defines the possible flowcontrol types for the serial port.
 */
typedef enum {
  flowcontrol_none = 0,
  flowcontrol_software,
  flowcontrol_hardware
} flowcontrol_t;

/*!
 * Structure for setting the timeout of the serial port, times are
 * in milliseconds.
 *
 * In order to disable the interbyte timeout, set it to Timeout::max().
 */
struct Timeout {
#ifdef max
# undef max
#endif
  static uint32_t max() {return std::numeric_limits<uint32_t>::max();}
  /*!
   * Convenience function to generate Timeout structs using a
   * single absolute timeout.
   *
   * \param timeout A long that defines the time in milliseconds until a
   * timeout occurs after a call to read or write is made.
   *
   * \return Timeout struct that represents this simple timeout provided.
   */
  static Timeout simpleTimeout(uint32_t timeout) {
    return Timeout(max(), timeout, 0, timeout, 0);
  }

  /*! Number of milliseconds between bytes received to timeout on. */
  uint32_t inter_byte_timeout;
  /*! A constant number of milliseconds to wait after calling read. */
  uint32_t read_timeout_constant;
  /*! A multiplier against the number of requested bytes to wait after
   *  calling read.
   */
  uint32_t read_timeout_multiplier;
  /*! A constant number of milliseconds to wait after calling write. */
  uint32_t write_timeout_constant;
  /*! A multiplier against the number of requested bytes to wait after
   *  calling write.
   */
  uint32_t write_timeout_multiplier;

  explicit Timeout (uint32_t inter_byte_timeout_=0,
                    uint32_t read_timeout_constant_=0,
                    uint32_t read_timeout_multiplier_=0,
                    uint32_t write_timeout_constant_=0,
                    uint32_t write_timeout_multiplier_=0)
  : inter_byte_timeout(inter_byte_timeout_),
    read_timeout_constant(read_timeout_constant_),
    read_timeout_multiplier(read_timeout_multiplier_),
    write_timeout_constant(write_timeout_constant_),
    write_timeout_multiplier(write_timeout_multiplier_)
  {}
};

/*!
 * Class that provides a portable serial port interface.
 */
class Serial {
public:
  /*!
   * Creates a Serial object and opens the port if a port is specified,
   * otherwise it remains closed until serial::Serial::open is called.
   *
   * \param port A std::string containing the address of the serial port,
   *        which would be something like 'COM1' on Windows and '/dev/ttyS0'
   *        on Linux.
   *
   * \param baudrate An unsigned 32-bit integer that represents the baudrate
   *
   * \param timeout A serial::Timeout struct that defines the timeout
   * conditions for the serial port. \see serial::Timeout
   *
   * \param bytesize Size of each byte in the serial transmission of data,
   * default is eightbits, possible values are: fivebits, sixbits, sevenbits,
   * eightbits
   *
   * \param parity Method of parity, default is parity_none, possible values
   * are: parity_none, parity_odd, parity_even
   *
   * \param stopbits Number of stop bits used, default is stopbits_one,
   * possible values are: stopbits_one, stopbits_one_point_five, stopbits_two
   *
   * \param flowcontrol Type of flowcontrol used, default is
   * flowcontrol_none, possible values are: flowcontrol_none,
   * flowcontrol_software, flowcontrol_hardware
   *
   * \throw serial::PortNotOpenedException
   * \throw serial::IOException
   * \throw std::invalid_argument
   */
  Serial (const std::string &port = "",
          uint32_t baudrate = 9600,
          Timeout timeout = Timeout(),
          bytesize_t bytesize = eightbits,
          parity_t parity = parity_none,
          stopbits_t stopbits = stopbits_one,
          flowcontrol_t flowcontrol = flowcontrol_none);

  /*! Destructor */
  virtual ~Serial ();

  /*!
   * Opens the serial port as long as the port is set and the port isn't
   * already open.
   *
   * If the port is provided to the constructor then an explicit call to open
   * is not needed.
   *
   * \see Serial::Serial
   *
   * \throw std::invalid_argument
   * \throw serial::SerialException
   * \throw serial::IOException
   */
  void
  open ();

  /*! Gets the open status of the serial port.
   *
   * \return Returns true if the port is open, false otherwise.
   */
  bool
  isOpen () const;

  /*! Closes the serial port. */
  void
  close ();

  /*! Return the number of characters in the buffer. */
  size_t
  available ();

  /*! Block until there is serial data to read or read_timeout_constant
   * number of milliseconds have elapsed. The return value is true when
   * the function exits with the port in a readable state, false otherwise
   * (due to timeout or select interruption). */
  bool
  waitReadable ();

  /*! Block for a period of time corresponding to the transmission time of
   * count characters at present serial settings. This may be used in con-
   * junction with waitReadable to read larger blocks of data from the
   * port. */
  void
  waitByteTimes (size_t count);

  /*! Read a given amount of bytes from the serial port into a given buffer.
   *
   * The read function will return in one of three cases:
   *  * The number of requested bytes was read.
   *    * In this case the number of bytes requested will match the size_t
   *      returned by read.
   *  * A timeout occurred, in this case the number of bytes read will not
   *    match the amount requested, but no exception will be thrown.  One of
   *    two possible timeouts occurred:
   *    * The inter byte timeout expired, this means that number of
   *      milliseconds elapsed between receiving bytes from the serial port
   *      exceeded the inter byte timeout.
   *    * The total timeout expired, which is calculated by multiplying the
   *      read timeout multiplier by the number of requested bytes and then
   *      added to the read timeout constant.  If that total number of
   *      milliseconds elapses after the initial call to read a timeout will
   *      occur.
   *  * An exception occurred, in this case an actual exception will be thrown.
   *
   * \param buffer An uint8_t array of at least the requested size.
   * \param size A size_t defining how many bytes to be read.
   *
   * \return A size_t representing the number of bytes read as a result of the
   *         call to read.
   *
   * \throw serial::PortNotOpenedException
   * \throw serial::SerialException
   */
  size_t
  read (uint8_t *buffer, size_t size);

  /*! Read a given amount of bytes from the serial port into a give buffer.
   *
   * \param buffer A reference to a std::vector of uint8_t.
   * \param size A size_t defining how many bytes to be read.
   *
   * \return A size_t representing the number of bytes read as a result of the
   *         call to read.
   *
   * \throw serial::PortNotOpenedException
   * \throw serial::SerialException
   */
  size_t
  read (std::vector<uint8_t> &buffer, size_t size = 1);

  /*! Read a given amount of bytes from the serial port into a give buffer.
   *
   * \param buffer A reference to a std::string.
   * \param size A size_t defining how many bytes to be read.
   *
   * \return A size_t representing the number of bytes read as a result of the
   *         call to read.
   *
   * \throw serial::PortNotOpenedException
   * \throw serial::SerialException
   */
  size_t
  read (std::string &buffer, size_t size = 1);

  /*! Read a given amount of bytes from the serial port and return a string
   *  containing the data.
   *
   * \param size A size_t defining how many bytes to be read.
   *
   * \return A std::string containing the data read from the port.
   *
   * \throw serial::PortNotOpenedException
   * \throw serial::SerialException
   */
  std::string
  read (size_t size = 1);

  /*! Reads in a line or until a given delimiter has been processed.
   *
   * Reads from the serial port until a single line has been read.
   *
   * \param buffer A std::string reference used to store the data.
   * \param size A maximum length of a line, defaults to 65536 (2^16)
   * \param eol A string to match against for the EOL.
   *
   * \return A size_t representing the number of bytes read.
   *
   * \throw serial::PortNotOpenedException
   * \throw serial::SerialException
   */
  size_t
  readline (std::string &buffer, size_t size = 65536, std::string eol = "\n");

  /*! Reads in a line or until a given delimiter has been processed.
   *
   * Reads from the serial port until a single line has been read.
   *
   * \param size A maximum length of a line, defaults to 65536 (2^16)
   * \param eol A string to match against for the EOL.
   *
   * \return A std::string containing the line.
   *
   * \throw serial::PortNotOpenedException
   * \throw serial::SerialException
   */
  std::string
  readline (size_t size = 65536, std::string eol = "\n");

  /*! Reads in multiple lines until the serial port times out.
   *
   * This requires a timeout > 0 before it can be run. It will read until a
   * timeout occurs and return a list of strings.
   *
   * \param size A maximum length of combined lines, defaults to 65536 (2^16)
   *
   * \param eol A string to match against for the EOL.
   *
   * \return A vector<string> containing the lines.
   *
   * \throw serial::PortNotOpenedException
   * \throw serial::SerialException
   */
  std::vector<std::string>
  readlines (size_t size = 65536, std::string eol = "\n");

  /*! Write a string to the serial port.
   *
   * \param data A const reference containing the data to be written
   * to the serial port.
   *
   * \param size A size_t that indicates how many bytes should be written from
   * the given data buffer.
   *
   * \return A size_t representing the number of bytes actually written to
   * the serial port.
   *
   * \throw serial::PortNotOpenedException
   * \throw serial::SerialException
   * \throw serial::IOException
   */
  size_t
  write (const uint8_t *data, size_t size);

  /*! Write a string to the serial port.
   *
   * \param data A const reference containing the data to be written
   * to the serial port.
   *
   * \return A size_t representing the number of bytes actually written to
   * the serial port.
   *
   * \throw serial::PortNotOpenedException
   * \throw serial::SerialException
   * \throw serial::IOException
   */
  size_t
  write (const std::vector<uint8_t> &data);

  /*! Write a string to the serial port.
   *
   * \param data A const reference containing the data to be written
   * to the serial port.
   *
   * \return A size_t representing the number of bytes actually written to
   * the serial port.
   *
   * \throw serial::PortNotOpenedException
   * \throw serial::SerialException
   * \throw serial::IOException
   */
  size_t
  write (const std::string &data);

  /*! Sets the serial port identifier.
   *
   * \param port A const std::string reference containing the address of the
   * serial port, which would be something like 'COM1' on Windows and
   * '/dev/ttyS0' on Linux.
   *
   * \throw std::invalid_argument
   */
  void
  setPort (const std::string &port);

  /*! Gets the serial port identifier.
   *
   * \see Serial::setPort
   *
   * \throw std::invalid_argument
   */
  std::string
  getPort () const;

  /*! Sets the timeout for reads and writes using the Timeout struct.
   *
   * There are two timeout conditions described here:
   *  * The inter byte timeout:
   *    * The inter_byte_timeout component of serial::Timeout defines the
   *      maximum amount of time, in milliseconds, between receiving bytes on
   *      the serial port that can pass before a timeout occurs.  Setting this
   *      to zero will prevent inter byte timeouts from occurring.
   *  * Total time timeout:
   *    * The constant and multiplier component of this timeout condition,
   *      for both read and write, are defined in serial::Timeout.  This
   *      timeout occurs if the total time since the read or write call was
   *      made exceeds the specified time in milliseconds.
   *    * The limit is defined by multiplying the multiplier component by the
   *      number of requested bytes and adding that product to the constant
   *      component.  In this way if you want a read call, for example, to
   *      timeout after exactly one second regardless of the number of bytes
   *      you asked for then set the read_timeout_constant component of
   *      serial::Timeout to 1000 and the read_timeout_multiplier to zero.
   *      This timeout condition can be used in conjunction with the inter
   *      byte timeout condition with out any problems, timeout will simply
   *      occur when one of the two timeout conditions is met.  This allows
   *      users to have maximum control over the trade-off between
   *      responsiveness and efficiency.
   *
   * Read and write functions will return in one of three cases.  When the
   * reading or writing is complete, when a timeout occurs, or when an
   * exception occurs.
   *
   * A timeout of 0 enables non-blocking mode.
   *
   * \param timeout A serial::Timeout struct containing the inter byte
   * timeout, and the read and write timeout constants and multipliers.
   *
   * \see serial::Timeout
   */
  void
  setTimeout (Timeout &timeout);

  /*! Sets the timeout for reads and writes. */
  void
  setTimeout (uint32_t inter_byte_timeout, uint32_t read_timeout_constant,
              uint32_t read_timeout_multiplier, uint32_t write_timeout_constant,
              uint32_t write_timeout_multiplier)
  {
    Timeout timeout(inter_byte_timeout, read_timeout_constant,
                    read_timeout_multiplier, write_timeout_constant,
                    write_timeout_multiplier);
    return setTimeout(timeout);
  }

  /*! Gets the timeout for reads in seconds.
   *
   * \return A Timeout struct containing the inter_byte_timeout, and read
   * and write timeout constants and multipliers.
   *
   * \see Serial::setTimeout
   */
  Timeout
  getTimeout () const;

  /*! Sets the baudrate for the serial port.
   *
   * Possible baudrates depends on the system but some safe baudrates include:
   * 110, 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 56000,
   * 57600, 115200
   * Some other baudrates that are supported by some comports:
   * 128000, 153600, 230400, 256000, 460800, 500000, 921600
   *
   * \param baudrate An integer that sets the baud rate for the serial port.
   *
   * \throw std::invalid_argument
   */
  void
  setBaudrate (uint32_t baudrate);

  /*! Gets the baudrate for the serial port.
   *
   * \return An integer that sets the baud rate for the serial port.
   *
   * \see Serial::setBaudrate
   *
   * \throw std::invalid_argument
   */
  uint32_t
  getBaudrate () const;

  /*! Sets the bytesize for the serial port.
   *
   * \param bytesize Size of each byte in the serial transmission of data,
   * default is eightbits, possible values are: fivebits, sixbits, sevenbits,
   * eightbits
   *
   * \throw std::invalid_argument
   */
  void
  setBytesize (bytesize_t bytesize);

  /*! Gets the bytesize for the serial port.
   *
   * \see Serial::setBytesize
   *
   * \throw std::invalid_argument
   */
  bytesize_t
  getBytesize () const;

  /*! Sets the parity for the serial port.
   *
   * \param parity Method of parity, default is parity_none, possible values
   * are: parity_none, parity_odd, parity_even
   *
   * \throw std::invalid_argument
   */
  void
  setParity (parity_t parity);

  /*! Gets the parity for the serial port.
   *
   * \see Serial::setParity
   *
   * \throw std::invalid_argument
   */
  parity_t
  getParity () const;

  /*! Sets the stopbits for the serial port.
   *
   * \param stopbits Number of stop bits used, default is stopbits_one,
   * possible values are: stopbits_one, stopbits_one_point_five, stopbits_two
   *
   * \throw std::invalid_argument
   */
  void
  setStopbits (stopbits_t stopbits);

  /*! Gets the stopbits for the serial port.
   *
   * \see Serial::setStopbits
   *
   * \throw std::invalid_argument
   */
  stopbits_t
  getStopbits () const;

  /*! Sets the flow control for the serial port.
   *
   * \param flowcontrol Type of flowcontrol used, default is flowcontrol_none,
   * possible values are: flowcontrol_none, flowcontrol_software,
   * flowcontrol_hardware
   *
   * \throw std::invalid_argument
   */
  void
  setFlowcontrol (flowcontrol_t flowcontrol);

  /*! Gets the flow control for the serial port.
   *
   * \see Serial::setFlowcontrol
   *
   * \throw std::invalid_argument
   */
  flowcontrol_t
  getFlowcontrol () const;

  /*! Flush the input and output buffers */
  void
  flush ();

  /*! Flush only the input buffer */
  void
  flushInput ();

  /*! Flush only the output buffer */
  void
  flushOutput ();

  /*! Sends the RS-232 break signal.  See tcsendbreak(3). */
  void
  sendBreak (int duration);

  /*! Set the break condition to a given level.  Defaults to true. */
  void
  setBreak (bool level = true);

  /*! Set the RTS handshaking line to the given level.  Defaults to true. */
  void
  setRTS (bool level = true);

  /*! Set the DTR handshaking line to the given level.  Defaults to true. */
  void
  setDTR (bool level = true);

  /*!
   * Blocks until CTS, DSR, RI, CD changes or something interrupts it.
   *
   * Can throw an exception if an error occurs while waiting.
   * You can check the status of CTS, DSR, RI, and CD once this returns.
   * Uses TIOCMIWAIT via ioctl if available (mostly only on Linux) with a
   * resolution of less than +-1ms and as good as +-0.2ms.  Otherwise a
   * polling method is used which can give +-2ms.
   *
   * \return Returns true if one of the lines changed, false if something else
   * occurred.
   *
   * \throw SerialException
   */
  bool
  waitForChange ();

  /*! Returns the current status of the CTS line. */
  bool
  getCTS ();

  /*! Returns the current status of the DSR line. */
  bool
  getDSR ();

  /*! Returns the current status of the RI line. */
  bool
  getRI ();

  /*! Returns the current status of the CD line. */
  bool
  getCD ();

private:
  // Disable copy constructors
  Serial(const Serial&);
  Serial& operator=(const Serial&);

  // Pimpl idiom, d_pointer
  class SerialImpl;
  SerialImpl *pimpl_;

  // Scoped Lock Classes
  class ScopedReadLock;
  class ScopedWriteLock;

  // Read common function
  size_t
  read_ (uint8_t *buffer, size_t size);
  // Write common function
  size_t
  write_ (const uint8_t *data, size_t length);

};

class SerialException : public std::exception
{
  // Disable copy constructors
  SerialException& operator=(const SerialException&);
  std::string e_what_;
public:
  SerialException (const char *description) {
      std::stringstream ss;
      ss << "SerialException " << description << " failed.";
      e_what_ = ss.str();
  }
  SerialException (const SerialException& other) : e_what_(other.e_what_) {}
  virtual ~SerialException() throw() {}
  virtual const char* what () const throw () {
    return e_what_.c_str();
  }
};

class IOException : public std::exception
{
  // Disable copy constructors
  IOException& operator=(const IOException&);
  std::string file_;
  int line_;
  std::string e_what_;
  int errno_;
public:
  explicit IOException (std::string file, int line, int errnum)
    : file_(file), line_(line), errno_(errnum) {
      std::stringstream ss;
#if defined(_WIN32) && !defined(__MINGW32__)
      char error_str [1024];
      strerror_s(error_str, 1024, errnum);
#else
      char * error_str = strerror(errnum);
#endif
      ss << "IO Exception (" << errno_ << "): " << error_str;
      ss << ", file " << file_ << ", line " << line_ << ".";
      e_what_ = ss.str();
  }
  explicit IOException (std::string file, int line, const char * description)
    : file_(file), line_(line), errno_(0) {
      std::stringstream ss;
      ss << "IO Exception: " << description;
      ss << ", file " << file_ << ", line " << line_ << ".";
      e_what_ = ss.str();
  }
  virtual ~IOException() throw() {}
  IOException (const IOException& other) : line_(other.line_), e_what_(other.e_what_), errno_(other.errno_) {}

  int getErrorNumber () const { return errno_; }

  virtual const char* what () const throw () {
    return e_what_.c_str();
  }
};

class PortNotOpenedException : public std::exception
{
  // Disable copy constructors
  const PortNotOpenedException& operator=(PortNotOpenedException);
  std::string e_what_;
public:
  PortNotOpenedException (const char * description)  {
      std::stringstream ss;
      ss << "PortNotOpenedException " << description << " failed.";
      e_what_ = ss.str();
  }
  PortNotOpenedException (const PortNotOpenedException& other) : e_what_(other.e_what_) {}
  virtual ~PortNotOpenedException() throw() {}
  virtual const char* what () const throw () {
    return e_what_.c_str();
  }
};

/*!
 * Structure that describes a serial device.
 */
struct PortInfo {

  /*! Address of the serial port (this can be passed to the constructor of Serial). */
  std::string port;

  /*! Human readable description of serial device if available. */
  std::string description;

  /*! Hardware ID (e.g. VID:PID of USB serial devices) or "n/a" if not available. */
  std::string hardware_id;

};

/* Lists the serial ports available on the system
 *
 * Returns a vector of available serial ports, each represented
 * by a serial::PortInfo data structure:
 *
 * \return vector of serial::PortInfo.
 */
std::vector<PortInfo>
list_ports();

} // namespace serial

#endif

////////////////////////////////////////////////////////////////////////////////

/*!
 * \file serial/impl/unix.h
 * \author  William Woodall <wjwwood@gmail.com>
 * \author  John Harrison <ash@greaterthaninfinity.com>
 * \version 0.1
 *
 * \section LICENSE
 *
 * The MIT License
 *
 * Copyright (c) 2012 William Woodall, John Harrison
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * \section DESCRIPTION
 *
 * This provides a unix based pimpl for the Serial class. This implementation is
 * based off termios.h and uses select for multiplexing the IO ports.
 *
 */

#if !defined(_WIN32)

#ifndef SERIAL_IMPL_UNIX_H
#define SERIAL_IMPL_UNIX_H

//#include "serial/serial.h"

#include <pthread.h>

namespace serial {

using std::size_t;
using std::string;
using std::invalid_argument;

using serial::SerialException;
using serial::IOException;

class MillisecondTimer {
public:
  MillisecondTimer(const uint32_t millis);         
  int64_t remaining();

private:
  static timespec timespec_now();
  timespec expiry;
};

class serial::Serial::SerialImpl {
public:
  SerialImpl (const string &port,
              unsigned long baudrate,
              bytesize_t bytesize,
              parity_t parity,
              stopbits_t stopbits,
              flowcontrol_t flowcontrol);

  virtual ~SerialImpl ();

  void
  open ();

  void
  close ();

  bool
  isOpen () const;

  size_t
  available ();

  bool
  waitReadable (uint32_t timeout);

  void
  waitByteTimes (size_t count);

  size_t
  read (uint8_t *buf, size_t size = 1);

  size_t
  write (const uint8_t *data, size_t length);

  void
  flush ();

  void
  flushInput ();

  void
  flushOutput ();

  void
  sendBreak (int duration);

  void
  setBreak (bool level);

  void
  setRTS (bool level);

  void
  setDTR (bool level);

  bool
  waitForChange ();

  bool
  getCTS ();

  bool
  getDSR ();

  bool
  getRI ();

  bool
  getCD ();

  void
  setPort (const string &port);

  string
  getPort () const;

  void
  setTimeout (Timeout &timeout);

  Timeout
  getTimeout () const;

  void
  setBaudrate (unsigned long baudrate);

  unsigned long
  getBaudrate () const;

  void
  setBytesize (bytesize_t bytesize);

  bytesize_t
  getBytesize () const;

  void
  setParity (parity_t parity);

  parity_t
  getParity () const;

  void
  setStopbits (stopbits_t stopbits);

  stopbits_t
  getStopbits () const;

  void
  setFlowcontrol (flowcontrol_t flowcontrol);

  flowcontrol_t
  getFlowcontrol () const;

  void
  readLock ();

  void
  readUnlock ();

  void
  writeLock ();

  void
  writeUnlock ();

protected:
  void reconfigurePort ();

private:
  string port_;               // Path to the file descriptor
  int fd_;                    // The current file descriptor

  bool is_open_;
  bool xonxoff_;
  bool rtscts_;

  Timeout timeout_;           // Timeout for read operations
  unsigned long baudrate_;    // Baudrate
  uint32_t byte_time_ns_;     // Nanoseconds to transmit/receive a single byte

  parity_t parity_;           // Parity
  bytesize_t bytesize_;       // Size of the bytes
  stopbits_t stopbits_;       // Stop Bits
  flowcontrol_t flowcontrol_; // Flow Control

  // Mutex used to lock the read functions
  pthread_mutex_t read_mutex;
  // Mutex used to lock the write functions
  pthread_mutex_t write_mutex;
};

}

#endif // SERIAL_IMPL_UNIX_H

#endif // !defined(_WIN32)

////////////////////////////////////////////////////////////////////////////////

/* Copyright 2012 William Woodall and John Harrison */
#include <algorithm>

#if !defined(_WIN32) && !defined(__OpenBSD__) && !defined(__FreeBSD__)
# include <alloca.h>
#endif

#if defined (__MINGW32__)
# define alloca __builtin_alloca
#endif

//#include "serial/serial.h"

//#ifdef _WIN32
//#include "serial/impl/win.h"
//#else
//#include "serial/impl/unix.h"
//#endif

using std::invalid_argument;
using std::min;
using std::numeric_limits;
using std::vector;
using std::size_t;
using std::string;

using serial::Serial;
using serial::SerialException;
using serial::IOException;
using serial::bytesize_t;
using serial::parity_t;
using serial::stopbits_t;
using serial::flowcontrol_t;

class Serial::ScopedReadLock {
public:
  ScopedReadLock(SerialImpl *pimpl) : pimpl_(pimpl) {
    this->pimpl_->readLock();
  }
  ~ScopedReadLock() {
    this->pimpl_->readUnlock();
  }
private:
  // Disable copy constructors
  ScopedReadLock(const ScopedReadLock&);
  const ScopedReadLock& operator=(ScopedReadLock);

  SerialImpl *pimpl_;
};

class Serial::ScopedWriteLock {
public:
  ScopedWriteLock(SerialImpl *pimpl) : pimpl_(pimpl) {
    this->pimpl_->writeLock();
  }
  ~ScopedWriteLock() {
    this->pimpl_->writeUnlock();
  }
private:
  // Disable copy constructors
  ScopedWriteLock(const ScopedWriteLock&);
  const ScopedWriteLock& operator=(ScopedWriteLock);
  SerialImpl *pimpl_;
};

inline Serial::Serial (const string &port, uint32_t baudrate, serial::Timeout timeout,
                bytesize_t bytesize, parity_t parity, stopbits_t stopbits,
                flowcontrol_t flowcontrol)
 : pimpl_(new SerialImpl (port, baudrate, bytesize, parity,
                                           stopbits, flowcontrol))
{
  pimpl_->setTimeout(timeout);
}

inline Serial::~Serial ()
{
  delete pimpl_;
}

inline void
Serial::open ()
{
  pimpl_->open ();
}

inline void
Serial::close ()
{
  pimpl_->close ();
}

inline bool
Serial::isOpen () const
{
  return pimpl_->isOpen ();
}

inline size_t
Serial::available ()
{
  return pimpl_->available ();
}

inline bool
Serial::waitReadable ()
{
  serial::Timeout timeout(pimpl_->getTimeout ());
  return pimpl_->waitReadable(timeout.read_timeout_constant);
}

inline void
Serial::waitByteTimes (size_t count)
{
  pimpl_->waitByteTimes(count);
}

inline size_t
Serial::read_ (uint8_t *buffer, size_t size)
{
  return this->pimpl_->read (buffer, size);
}

inline size_t
Serial::read (uint8_t *buffer, size_t size)
{
  ScopedReadLock lock(this->pimpl_);
  return this->pimpl_->read (buffer, size);
}

inline size_t
Serial::read (std::vector<uint8_t> &buffer, size_t size)
{
  ScopedReadLock lock(this->pimpl_);
  uint8_t *buffer_ = new uint8_t[size];
  size_t bytes_read = this->pimpl_->read (buffer_, size);
  buffer.insert (buffer.end (), buffer_, buffer_+bytes_read);
  delete[] buffer_;
  return bytes_read;
}

inline size_t
Serial::read (std::string &buffer, size_t size)
{
  ScopedReadLock lock(this->pimpl_);
  uint8_t *buffer_ = new uint8_t[size];
  size_t bytes_read = this->pimpl_->read (buffer_, size);
  buffer.append (reinterpret_cast<const char*>(buffer_), bytes_read);
  delete[] buffer_;
  return bytes_read;
}

inline string
Serial::read (size_t size)
{
  std::string buffer;
  this->read (buffer, size);
  return buffer;
}

inline size_t
Serial::readline (string &buffer, size_t size, string eol)
{
  ScopedReadLock lock(this->pimpl_);
  size_t eol_len = eol.length ();
  uint8_t *buffer_ = static_cast<uint8_t*>
                              (alloca (size * sizeof (uint8_t)));
  size_t read_so_far = 0;
  while (true)
  {
    size_t bytes_read = this->read_ (buffer_ + read_so_far, 1);
    read_so_far += bytes_read;
    if (bytes_read == 0) {
      break; // Timeout occured on reading 1 byte
    }
    if (string (reinterpret_cast<const char*>
         (buffer_ + read_so_far - eol_len), eol_len) == eol) {
      break; // EOL found
    }
    if (read_so_far == size) {
      break; // Reached the maximum read length
    }
  }
  buffer.append(reinterpret_cast<const char*> (buffer_), read_so_far);
  return read_so_far;
}

inline string
Serial::readline (size_t size, string eol)
{
  std::string buffer;
  this->readline (buffer, size, eol);
  return buffer;
}

inline vector<string>
Serial::readlines (size_t size, string eol)
{
  ScopedReadLock lock(this->pimpl_);
  std::vector<std::string> lines;
  size_t eol_len = eol.length ();
  uint8_t *buffer_ = static_cast<uint8_t*>
    (alloca (size * sizeof (uint8_t)));
  size_t read_so_far = 0;
  size_t start_of_line = 0;
  while (read_so_far < size) {
    size_t bytes_read = this->read_ (buffer_+read_so_far, 1);
    read_so_far += bytes_read;
    if (bytes_read == 0) {
      if (start_of_line != read_so_far) {
        lines.push_back (
          string (reinterpret_cast<const char*> (buffer_ + start_of_line),
            read_so_far - start_of_line));
      }
      break; // Timeout occured on reading 1 byte
    }
    if (string (reinterpret_cast<const char*>
         (buffer_ + read_so_far - eol_len), eol_len) == eol) {
      // EOL found
      lines.push_back(
        string(reinterpret_cast<const char*> (buffer_ + start_of_line),
          read_so_far - start_of_line));
      start_of_line = read_so_far;
    }
    if (read_so_far == size) {
      if (start_of_line != read_so_far) {
        lines.push_back(
          string(reinterpret_cast<const char*> (buffer_ + start_of_line),
            read_so_far - start_of_line));
      }
      break; // Reached the maximum read length
    }
  }
  return lines;
}

inline size_t
Serial::write (const string &data)
{
  ScopedWriteLock lock(this->pimpl_);
  return this->write_ (reinterpret_cast<const uint8_t*>(data.c_str()),
                       data.length());
}

inline size_t
Serial::write (const std::vector<uint8_t> &data)
{
  ScopedWriteLock lock(this->pimpl_);
  return this->write_ (&data[0], data.size());
}

inline size_t
Serial::write (const uint8_t *data, size_t size)
{
  ScopedWriteLock lock(this->pimpl_);
  return this->write_(data, size);
}

inline size_t
Serial::write_ (const uint8_t *data, size_t length)
{
  return pimpl_->write (data, length);
}

inline void
Serial::setPort (const string &port)
{
  ScopedReadLock rlock(this->pimpl_);
  ScopedWriteLock wlock(this->pimpl_);
  bool was_open = pimpl_->isOpen ();
  if (was_open) close();
  pimpl_->setPort (port);
  if (was_open) open ();
}

inline string
Serial::getPort () const
{
  return pimpl_->getPort ();
}

inline void
Serial::setTimeout (serial::Timeout &timeout)
{
  pimpl_->setTimeout (timeout);
}

inline serial::Timeout
Serial::getTimeout () const {
  return pimpl_->getTimeout ();
}

inline void
Serial::setBaudrate (uint32_t baudrate)
{
  pimpl_->setBaudrate (baudrate);
}

inline uint32_t
Serial::getBaudrate () const
{
  return uint32_t(pimpl_->getBaudrate ());
}

inline void
Serial::setBytesize (bytesize_t bytesize)
{
  pimpl_->setBytesize (bytesize);
}

inline bytesize_t
Serial::getBytesize () const
{
  return pimpl_->getBytesize ();
}

inline void
Serial::setParity (parity_t parity)
{
  pimpl_->setParity (parity);
}

inline parity_t
Serial::getParity () const
{
  return pimpl_->getParity ();
}

inline void
Serial::setStopbits (stopbits_t stopbits)
{
  pimpl_->setStopbits (stopbits);
}

inline stopbits_t
Serial::getStopbits () const
{
  return pimpl_->getStopbits ();
}

inline void
Serial::setFlowcontrol (flowcontrol_t flowcontrol)
{
  pimpl_->setFlowcontrol (flowcontrol);
}

inline flowcontrol_t
Serial::getFlowcontrol () const
{
  return pimpl_->getFlowcontrol ();
}

inline void Serial::flush ()
{
  ScopedReadLock rlock(this->pimpl_);
  ScopedWriteLock wlock(this->pimpl_);
  pimpl_->flush ();
}

inline void Serial::flushInput ()
{
  ScopedReadLock lock(this->pimpl_);
  pimpl_->flushInput ();
}

inline void Serial::flushOutput ()
{
  ScopedWriteLock lock(this->pimpl_);
  pimpl_->flushOutput ();
}

inline void Serial::sendBreak (int duration)
{
  pimpl_->sendBreak (duration);
}

inline void Serial::setBreak (bool level)
{
  pimpl_->setBreak (level);
}

inline void Serial::setRTS (bool level)
{
  pimpl_->setRTS (level);
}

inline void Serial::setDTR (bool level)
{
  pimpl_->setDTR (level);
}

inline bool Serial::waitForChange()
{
  return pimpl_->waitForChange();
}

inline bool Serial::getCTS ()
{
  return pimpl_->getCTS ();
}

inline bool Serial::getDSR ()
{
  return pimpl_->getDSR ();
}

inline bool Serial::getRI ()
{
  return pimpl_->getRI ();
}

inline bool Serial::getCD ()
{
  return pimpl_->getCD ();
}

/* Copyright 2012 William Woodall and John Harrison
 *
 * Additional Contributors: Christopher Baker @bakercp
 */

#if !defined(_WIN32)

#include <stdio.h>
#include <string.h>
#include <sstream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/signal.h>
#include <errno.h>
#include <paths.h>
#include <sysexits.h>
#include <termios.h>
#include <sys/param.h>
#include <pthread.h>

#if defined(__linux__)
# include <linux/serial.h>
#endif

#include <sys/select.h>
#include <sys/time.h>
#include <time.h>
#ifdef __MACH__
#include <AvailabilityMacros.h>
#include <mach/clock.h>
#include <mach/mach.h>
#endif

//#include "serial/impl/unix.h"

#ifndef TIOCINQ
#ifdef FIONREAD
#define TIOCINQ FIONREAD
#else
#define TIOCINQ 0x541B
#endif
#endif

#if defined(MAC_OS_X_VERSION_10_3) && (MAC_OS_X_VERSION_MIN_REQUIRED >= MAC_OS_X_VERSION_10_3)
#include <IOKit/serial/ioss.h>
#endif

using std::string;
using std::stringstream;
using std::invalid_argument;
using serial::MillisecondTimer;
using serial::Serial;
using serial::SerialException;
using serial::PortNotOpenedException;
using serial::IOException;


inline MillisecondTimer::MillisecondTimer (const uint32_t millis)
  : expiry(timespec_now())
{
  int64_t tv_nsec = expiry.tv_nsec + (millis * 1e6);
  if (tv_nsec >= 1e9) {
    int64_t sec_diff = tv_nsec / static_cast<int> (1e9);
    expiry.tv_nsec = tv_nsec % static_cast<int>(1e9);
    expiry.tv_sec += sec_diff;
  } else {
    expiry.tv_nsec = tv_nsec;
  }
}

inline int64_t
MillisecondTimer::remaining ()
{
  timespec now(timespec_now());
  int64_t millis = (expiry.tv_sec - now.tv_sec) * 1e3;
  millis += (expiry.tv_nsec - now.tv_nsec) / 1e6;
  return millis;
}

inline timespec
MillisecondTimer::timespec_now ()
{
  timespec time;
# ifdef __MACH__ // OS X does not have clock_gettime, use clock_get_time
  clock_serv_t cclock;
  mach_timespec_t mts;
  host_get_clock_service(mach_host_self(), SYSTEM_CLOCK, &cclock);
  clock_get_time(cclock, &mts);
  mach_port_deallocate(mach_task_self(), cclock);
  time.tv_sec = mts.tv_sec;
  time.tv_nsec = mts.tv_nsec;
# else
  clock_gettime(CLOCK_MONOTONIC, &time);
# endif
  return time;
}

inline timespec
timespec_from_ms (const uint32_t millis)
{
  timespec time;
  time.tv_sec = millis / 1e3;
  time.tv_nsec = (millis - (time.tv_sec * 1e3)) * 1e6;
  return time;
}

inline Serial::SerialImpl::SerialImpl (const string &port, unsigned long baudrate,
                                bytesize_t bytesize,
                                parity_t parity, stopbits_t stopbits,
                                flowcontrol_t flowcontrol)
  : port_ (port), fd_ (-1), is_open_ (false), xonxoff_ (false), rtscts_ (false),
    baudrate_ (baudrate), parity_ (parity),
    bytesize_ (bytesize), stopbits_ (stopbits), flowcontrol_ (flowcontrol)
{
  pthread_mutex_init(&this->read_mutex, NULL);
  pthread_mutex_init(&this->write_mutex, NULL);
  if (port_.empty () == false)
    open ();
}

inline Serial::SerialImpl::~SerialImpl ()
{
  close();
  pthread_mutex_destroy(&this->read_mutex);
  pthread_mutex_destroy(&this->write_mutex);
}

inline void
Serial::SerialImpl::open ()
{
  if (port_.empty ()) {
    throw invalid_argument ("Empty port is invalid.");
  }
  if (is_open_ == true) {
    throw SerialException ("Serial port already open.");
  }

  fd_ = ::open (port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (fd_ == -1) {
    switch (errno) {
    case EINTR:
      // Recurse because this is a recoverable error.
      open ();
      return;
    case ENFILE:
    case EMFILE:
      THROW (IOException, "Too many file handles open.");
    default:
      THROW (IOException, errno);
    }
  }

  reconfigurePort();
  is_open_ = true;
}

inline void
Serial::SerialImpl::reconfigurePort ()
{
  if (fd_ == -1) {
    // Can only operate on a valid file descriptor
    THROW (IOException, "Invalid file descriptor, is the serial port open?");
  }

  struct termios options; // The options for the file descriptor

  if (tcgetattr(fd_, &options) == -1) {
    THROW (IOException, "::tcgetattr");
  }

  // set up raw mode / no echo / binary
  options.c_cflag |= (tcflag_t)  (CLOCAL | CREAD);
  options.c_lflag &= (tcflag_t) ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL |
                                       ISIG | IEXTEN); //|ECHOPRT

  options.c_oflag &= (tcflag_t) ~(OPOST);
  options.c_iflag &= (tcflag_t) ~(INLCR | IGNCR | ICRNL | IGNBRK);
#ifdef IUCLC
  options.c_iflag &= (tcflag_t) ~IUCLC;
#endif
#ifdef PARMRK
  options.c_iflag &= (tcflag_t) ~PARMRK;
#endif

  // setup baud rate
  bool custom_baud = false;
  speed_t baud;
  switch (baudrate_) {
#ifdef B0
  case 0: baud = B0; break;
#endif
#ifdef B50
  case 50: baud = B50; break;
#endif
#ifdef B75
  case 75: baud = B75; break;
#endif
#ifdef B110
  case 110: baud = B110; break;
#endif
#ifdef B134
  case 134: baud = B134; break;
#endif
#ifdef B150
  case 150: baud = B150; break;
#endif
#ifdef B200
  case 200: baud = B200; break;
#endif
#ifdef B300
  case 300: baud = B300; break;
#endif
#ifdef B600
  case 600: baud = B600; break;
#endif
#ifdef B1200
  case 1200: baud = B1200; break;
#endif
#ifdef B1800
  case 1800: baud = B1800; break;
#endif
#ifdef B2400
  case 2400: baud = B2400; break;
#endif
#ifdef B4800
  case 4800: baud = B4800; break;
#endif
#ifdef B7200
  case 7200: baud = B7200; break;
#endif
#ifdef B9600
  case 9600: baud = B9600; break;
#endif
#ifdef B14400
  case 14400: baud = B14400; break;
#endif
#ifdef B19200
  case 19200: baud = B19200; break;
#endif
#ifdef B28800
  case 28800: baud = B28800; break;
#endif
#ifdef B57600
  case 57600: baud = B57600; break;
#endif
#ifdef B76800
  case 76800: baud = B76800; break;
#endif
#ifdef B38400
  case 38400: baud = B38400; break;
#endif
#ifdef B115200
  case 115200: baud = B115200; break;
#endif
#ifdef B128000
  case 128000: baud = B128000; break;
#endif
#ifdef B153600
  case 153600: baud = B153600; break;
#endif
#ifdef B230400
  case 230400: baud = B230400; break;
#endif
#ifdef B256000
  case 256000: baud = B256000; break;
#endif
#ifdef B460800
  case 460800: baud = B460800; break;
#endif
#ifdef B500000
  case 500000: baud = B500000; break;
#endif
#ifdef B576000
  case 576000: baud = B576000; break;
#endif
#ifdef B921600
  case 921600: baud = B921600; break;
#endif
#ifdef B1000000
  case 1000000: baud = B1000000; break;
#endif
#ifdef B1152000
  case 1152000: baud = B1152000; break;
#endif
#ifdef B1500000
  case 1500000: baud = B1500000; break;
#endif
#ifdef B2000000
  case 2000000: baud = B2000000; break;
#endif
#ifdef B2500000
  case 2500000: baud = B2500000; break;
#endif
#ifdef B3000000
  case 3000000: baud = B3000000; break;
#endif
#ifdef B3500000
  case 3500000: baud = B3500000; break;
#endif
#ifdef B4000000
  case 4000000: baud = B4000000; break;
#endif
  default:
    custom_baud = true;
    // OS X support
#if defined(MAC_OS_X_VERSION_10_4) && (MAC_OS_X_VERSION_MIN_REQUIRED >= MAC_OS_X_VERSION_10_4)
    // Starting with Tiger, the IOSSIOSPEED ioctl can be used to set arbitrary baud rates
    // other than those specified by POSIX. The driver for the underlying serial hardware
    // ultimately determines which baud rates can be used. This ioctl sets both the input
    // and output speed.
    speed_t new_baud = static_cast<speed_t> (baudrate_);
    if (-1 == ioctl (fd_, IOSSIOSPEED, &new_baud, 1)) {
      THROW (IOException, errno);
    }
    // Linux Support
#elif defined(__linux__) && defined (TIOCSSERIAL)
    struct serial_struct ser;

    if (-1 == ioctl (fd_, TIOCGSERIAL, &ser)) {
      THROW (IOException, errno);
    }

    // set custom divisor
    ser.custom_divisor = ser.baud_base / static_cast<int> (baudrate_);
    // update flags
    ser.flags &= ~ASYNC_SPD_MASK;
    ser.flags |= ASYNC_SPD_CUST;

    if (-1 == ioctl (fd_, TIOCSSERIAL, &ser)) {
      THROW (IOException, errno);
    }
#else
    throw invalid_argument ("OS does not currently support custom bauds");
#endif
  }
  if (custom_baud == false) {
#ifdef _BSD_SOURCE
    ::cfsetspeed(&options, baud);
#else
    ::cfsetispeed(&options, baud);
    ::cfsetospeed(&options, baud);
#endif
  }

  // setup char len
  options.c_cflag &= (tcflag_t) ~CSIZE;
  if (bytesize_ == eightbits)
    options.c_cflag |= CS8;
  else if (bytesize_ == sevenbits)
    options.c_cflag |= CS7;
  else if (bytesize_ == sixbits)
    options.c_cflag |= CS6;
  else if (bytesize_ == fivebits)
    options.c_cflag |= CS5;
  else
    throw invalid_argument ("invalid char len");
  // setup stopbits
  if (stopbits_ == stopbits_one)
    options.c_cflag &= (tcflag_t) ~(CSTOPB);
  else if (stopbits_ == stopbits_one_point_five)
    // ONE POINT FIVE same as TWO.. there is no POSIX support for 1.5
    options.c_cflag |=  (CSTOPB);
  else if (stopbits_ == stopbits_two)
    options.c_cflag |=  (CSTOPB);
  else
    throw invalid_argument ("invalid stop bit");
  // setup parity
  options.c_iflag &= (tcflag_t) ~(INPCK | ISTRIP);
  if (parity_ == parity_none) {
    options.c_cflag &= (tcflag_t) ~(PARENB | PARODD);
  } else if (parity_ == parity_even) {
    options.c_cflag &= (tcflag_t) ~(PARODD);
    options.c_cflag |=  (PARENB);
  } else if (parity_ == parity_odd) {
    options.c_cflag |=  (PARENB | PARODD);
  }
#ifdef CMSPAR
  else if (parity_ == parity_mark) {
    options.c_cflag |=  (PARENB | CMSPAR | PARODD);
  }
  else if (parity_ == parity_space) {
    options.c_cflag |=  (PARENB | CMSPAR);
    options.c_cflag &= (tcflag_t) ~(PARODD);
  }
#else
  // CMSPAR is not defined on OSX. So do not support mark or space parity.
  else if (parity_ == parity_mark || parity_ == parity_space) {
    throw invalid_argument ("OS does not support mark or space parity");
  }
#endif  // ifdef CMSPAR
  else {
    throw invalid_argument ("invalid parity");
  }
  // setup flow control
  if (flowcontrol_ == flowcontrol_none) {
    xonxoff_ = false;
    rtscts_ = false;
  }
  if (flowcontrol_ == flowcontrol_software) {
    xonxoff_ = true;
    rtscts_ = false;
  }
  if (flowcontrol_ == flowcontrol_hardware) {
    xonxoff_ = false;
    rtscts_ = true;
  }
  // xonxoff
#ifdef IXANY
  if (xonxoff_)
    options.c_iflag |=  (IXON | IXOFF); //|IXANY)
  else
    options.c_iflag &= (tcflag_t) ~(IXON | IXOFF | IXANY);
#else
  if (xonxoff_)
    options.c_iflag |=  (IXON | IXOFF);
  else
    options.c_iflag &= (tcflag_t) ~(IXON | IXOFF);
#endif
  // rtscts
#ifdef CRTSCTS
  if (rtscts_)
    options.c_cflag |=  (CRTSCTS);
  else
    options.c_cflag &= (unsigned long) ~(CRTSCTS);
#elif defined CNEW_RTSCTS
  if (rtscts_)
    options.c_cflag |=  (CNEW_RTSCTS);
  else
    options.c_cflag &= (unsigned long) ~(CNEW_RTSCTS);
#else
#error "OS Support seems wrong."
#endif

  // http://www.unixwiz.net/techtips/termios-vmin-vtime.html
  // this basically sets the read call up to be a polling read,
  // but we are using select to ensure there is data available
  // to read before each call, so we should never needlessly poll
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;

  // activate settings
  ::tcsetattr (fd_, TCSANOW, &options);

  // Update byte_time_ based on the new settings.
  uint32_t bit_time_ns = 1e9 / baudrate_;
  byte_time_ns_ = bit_time_ns * (1 + bytesize_ + parity_ + stopbits_);

  // Compensate for the stopbits_one_point_five enum being equal to int 3,
  // and not 1.5.
  if (stopbits_ == stopbits_one_point_five) {
    byte_time_ns_ += ((1.5 - stopbits_one_point_five) * bit_time_ns);
  }
}

inline void
Serial::SerialImpl::close ()
{
  if (is_open_ == true) {
    if (fd_ != -1) {
      int ret;
      ret = ::close (fd_);
      if (ret == 0) {
        fd_ = -1;
      } else {
        THROW (IOException, errno);
      }
    }
    is_open_ = false;
  }
}

inline bool
Serial::SerialImpl::isOpen () const
{
  return is_open_;
}

inline size_t
Serial::SerialImpl::available ()
{
  if (!is_open_) {
    return 0;
  }
  int count = 0;
  if (-1 == ioctl (fd_, TIOCINQ, &count)) {
      THROW (IOException, errno);
  } else {
      return static_cast<size_t> (count);
  }
}

inline bool
Serial::SerialImpl::waitReadable (uint32_t timeout)
{
  // Setup a select call to block for serial data or a timeout
  fd_set readfds;
  FD_ZERO (&readfds);
  FD_SET (fd_, &readfds);
  timespec timeout_ts (timespec_from_ms (timeout));
  int r = pselect (fd_ + 1, &readfds, NULL, NULL, &timeout_ts, NULL);

  if (r < 0) {
    // Select was interrupted
    if (errno == EINTR) {
      return false;
    }
    // Otherwise there was some error
    THROW (IOException, errno);
  }
  // Timeout occurred
  if (r == 0) {
    return false;
  }
  // This shouldn't happen, if r > 0 our fd has to be in the list!
  if (!FD_ISSET (fd_, &readfds)) {
    THROW (IOException, "select reports ready to read, but our fd isn't"
           " in the list, this shouldn't happen!");
  }
  // Data available to read.
  return true;
}

inline void
Serial::SerialImpl::waitByteTimes (size_t count)
{
  timespec wait_time = { 0, static_cast<long>(byte_time_ns_ * count)};
  pselect (0, NULL, NULL, NULL, &wait_time, NULL);
}

inline size_t
Serial::SerialImpl::read (uint8_t *buf, size_t size)
{
  // If the port is not open, throw
  if (!is_open_) {
    throw PortNotOpenedException ("Serial::read");
  }
  size_t bytes_read = 0;

  // Calculate total timeout in milliseconds t_c + (t_m * N)
  long total_timeout_ms = timeout_.read_timeout_constant;
  total_timeout_ms += timeout_.read_timeout_multiplier * static_cast<long> (size);
  MillisecondTimer total_timeout(total_timeout_ms);

  // Pre-fill buffer with available bytes
  {
    ssize_t bytes_read_now = ::read (fd_, buf, size);
    if (bytes_read_now > 0) {
      bytes_read = bytes_read_now;
    }
  }

  while (bytes_read < size) {
    int64_t timeout_remaining_ms = total_timeout.remaining();
    if (timeout_remaining_ms <= 0) {
      // Timed out
      break;
    }
    // Timeout for the next select is whichever is less of the remaining
    // total read timeout and the inter-byte timeout.
    uint32_t timeout = std::min(static_cast<uint32_t> (timeout_remaining_ms),
                                timeout_.inter_byte_timeout);
    // Wait for the device to be readable, and then attempt to read.
    if (waitReadable(timeout)) {
      // If it's a fixed-length multi-byte read, insert a wait here so that
      // we can attempt to grab the whole thing in a single IO call. Skip
      // this wait if a non-max inter_byte_timeout is specified.
      if (size > 1 && timeout_.inter_byte_timeout == Timeout::max()) {
        size_t bytes_available = available();
        if (bytes_available + bytes_read < size) {
          waitByteTimes(size - (bytes_available + bytes_read));
        }
      }
      // This should be non-blocking returning only what is available now
      //  Then returning so that select can block again.
      ssize_t bytes_read_now =
        ::read (fd_, buf + bytes_read, size - bytes_read);
      // read should always return some data as select reported it was
      // ready to read when we get to this point.
      if (bytes_read_now < 1) {
        // Disconnected devices, at least on Linux, show the
        // behavior that they are always ready to read immediately
        // but reading returns nothing.
        throw SerialException ("device reports readiness to read but "
                               "returned no data (device disconnected?)");
      }
      // Update bytes_read
      bytes_read += static_cast<size_t> (bytes_read_now);
      // If bytes_read == size then we have read everything we need
      if (bytes_read == size) {
        break;
      }
      // If bytes_read < size then we have more to read
      if (bytes_read < size) {
        continue;
      }
      // If bytes_read > size then we have over read, which shouldn't happen
      if (bytes_read > size) {
        throw SerialException ("read over read, too many bytes where "
                               "read, this shouldn't happen, might be "
                               "a logical error!");
      }
    }
  }
  return bytes_read;
}

inline size_t
Serial::SerialImpl::write (const uint8_t *data, size_t length)
{
  if (is_open_ == false) {
    throw PortNotOpenedException ("Serial::write");
  }
  fd_set writefds;
  size_t bytes_written = 0;

  // Calculate total timeout in milliseconds t_c + (t_m * N)
  long total_timeout_ms = timeout_.write_timeout_constant;
  total_timeout_ms += timeout_.write_timeout_multiplier * static_cast<long> (length);
  MillisecondTimer total_timeout(total_timeout_ms);

  bool first_iteration = true;
  while (bytes_written < length) {
    int64_t timeout_remaining_ms = total_timeout.remaining();
    // Only consider the timeout if it's not the first iteration of the loop
    // otherwise a timeout of 0 won't be allowed through
    if (!first_iteration && (timeout_remaining_ms <= 0)) {
      // Timed out
      break;
    }
    first_iteration = false;

    timespec timeout(timespec_from_ms(timeout_remaining_ms));

    FD_ZERO (&writefds);
    FD_SET (fd_, &writefds);

    // Do the select
    int r = pselect (fd_ + 1, NULL, &writefds, NULL, &timeout, NULL);

    // Figure out what happened by looking at select's response 'r'
    /** Error **/
    if (r < 0) {
      // Select was interrupted, try again
      if (errno == EINTR) {
        continue;
      }
      // Otherwise there was some error
      THROW (IOException, errno);
    }
    /** Timeout **/
    if (r == 0) {
      break;
    }
    /** Port ready to write **/
    if (r > 0) {
      // Make sure our file descriptor is in the ready to write list
      if (FD_ISSET (fd_, &writefds)) {
        // This will write some
        ssize_t bytes_written_now =
          ::write (fd_, data + bytes_written, length - bytes_written);
        // write should always return some data as select reported it was
        // ready to write when we get to this point.
        if (bytes_written_now < 1) {
          // Disconnected devices, at least on Linux, show the
          // behavior that they are always ready to write immediately
          // but writing returns nothing.
          throw SerialException ("device reports readiness to write but "
                                 "returned no data (device disconnected?)");
        }
        // Update bytes_written
        bytes_written += static_cast<size_t> (bytes_written_now);
        // If bytes_written == size then we have written everything we need to
        if (bytes_written == length) {
          break;
        }
        // If bytes_written < size then we have more to write
        if (bytes_written < length) {
          continue;
        }
        // If bytes_written > size then we have over written, which shouldn't happen
        if (bytes_written > length) {
          throw SerialException ("write over wrote, too many bytes where "
                                 "written, this shouldn't happen, might be "
                                 "a logical error!");
        }
      }
      // This shouldn't happen, if r > 0 our fd has to be in the list!
      THROW (IOException, "select reports ready to write, but our fd isn't"
                          " in the list, this shouldn't happen!");
    }
  }
  return bytes_written;
}

inline void
Serial::SerialImpl::setPort (const string &port)
{
  port_ = port;
}

inline string
Serial::SerialImpl::getPort () const
{
  return port_;
}

inline void
Serial::SerialImpl::setTimeout (serial::Timeout &timeout)
{
  timeout_ = timeout;
}

inline serial::Timeout
Serial::SerialImpl::getTimeout () const
{
  return timeout_;
}

inline void
Serial::SerialImpl::setBaudrate (unsigned long baudrate)
{
  baudrate_ = baudrate;
  if (is_open_)
    reconfigurePort ();
}

inline unsigned long
Serial::SerialImpl::getBaudrate () const
{
  return baudrate_;
}

inline void
Serial::SerialImpl::setBytesize (serial::bytesize_t bytesize)
{
  bytesize_ = bytesize;
  if (is_open_)
    reconfigurePort ();
}

inline serial::bytesize_t
Serial::SerialImpl::getBytesize () const
{
  return bytesize_;
}

inline void
Serial::SerialImpl::setParity (serial::parity_t parity)
{
  parity_ = parity;
  if (is_open_)
    reconfigurePort ();
}

inline serial::parity_t
Serial::SerialImpl::getParity () const
{
  return parity_;
}

inline void
Serial::SerialImpl::setStopbits (serial::stopbits_t stopbits)
{
  stopbits_ = stopbits;
  if (is_open_)
    reconfigurePort ();
}

inline serial::stopbits_t
Serial::SerialImpl::getStopbits () const
{
  return stopbits_;
}

inline void
Serial::SerialImpl::setFlowcontrol (serial::flowcontrol_t flowcontrol)
{
  flowcontrol_ = flowcontrol;
  if (is_open_)
    reconfigurePort ();
}

inline serial::flowcontrol_t
Serial::SerialImpl::getFlowcontrol () const
{
  return flowcontrol_;
}

inline void
Serial::SerialImpl::flush ()
{
  if (is_open_ == false) {
    throw PortNotOpenedException ("Serial::flush");
  }
  tcdrain (fd_);
}

inline void
Serial::SerialImpl::flushInput ()
{
  if (is_open_ == false) {
    throw PortNotOpenedException ("Serial::flushInput");
  }
  tcflush (fd_, TCIFLUSH);
}

inline void
Serial::SerialImpl::flushOutput ()
{
  if (is_open_ == false) {
    throw PortNotOpenedException ("Serial::flushOutput");
  }
  tcflush (fd_, TCOFLUSH);
}

inline void
Serial::SerialImpl::sendBreak (int duration)
{
  if (is_open_ == false) {
    throw PortNotOpenedException ("Serial::sendBreak");
  }
  tcsendbreak (fd_, static_cast<int> (duration / 4));
}

inline void
Serial::SerialImpl::setBreak (bool level)
{
  if (is_open_ == false) {
    throw PortNotOpenedException ("Serial::setBreak");
  }

  if (level) {
    if (-1 == ioctl (fd_, TIOCSBRK))
    {
        stringstream ss;
        ss << "setBreak failed on a call to ioctl(TIOCSBRK): " << errno << " " << strerror(errno);
        throw(SerialException(ss.str().c_str()));
    }
  } else {
    if (-1 == ioctl (fd_, TIOCCBRK))
    {
        stringstream ss;
        ss << "setBreak failed on a call to ioctl(TIOCCBRK): " << errno << " " << strerror(errno);
        throw(SerialException(ss.str().c_str()));
    }
  }
}

inline void
Serial::SerialImpl::setRTS (bool level)
{
  if (is_open_ == false) {
    throw PortNotOpenedException ("Serial::setRTS");
  }

  int command = TIOCM_RTS;

  if (level) {
    if (-1 == ioctl (fd_, TIOCMBIS, &command))
    {
      stringstream ss;
      ss << "setRTS failed on a call to ioctl(TIOCMBIS): " << errno << " " << strerror(errno);
      throw(SerialException(ss.str().c_str()));
    }
  } else {
    if (-1 == ioctl (fd_, TIOCMBIC, &command))
    {
      stringstream ss;
      ss << "setRTS failed on a call to ioctl(TIOCMBIC): " << errno << " " << strerror(errno);
      throw(SerialException(ss.str().c_str()));
    }
  }
}

inline void
Serial::SerialImpl::setDTR (bool level)
{
  if (is_open_ == false) {
    throw PortNotOpenedException ("Serial::setDTR");
  }

  int command = TIOCM_DTR;

  if (level) {
    if (-1 == ioctl (fd_, TIOCMBIS, &command))
    {
      stringstream ss;
      ss << "setDTR failed on a call to ioctl(TIOCMBIS): " << errno << " " << strerror(errno);
      throw(SerialException(ss.str().c_str()));
    }
  } else {
    if (-1 == ioctl (fd_, TIOCMBIC, &command))
    {
      stringstream ss;
      ss << "setDTR failed on a call to ioctl(TIOCMBIC): " << errno << " " << strerror(errno);
      throw(SerialException(ss.str().c_str()));
    }
  }
}

inline bool
Serial::SerialImpl::waitForChange ()
{
#ifndef TIOCMIWAIT

while (is_open_ == true) {

    int status;

    if (-1 == ioctl (fd_, TIOCMGET, &status))
    {
        stringstream ss;
        ss << "waitForChange failed on a call to ioctl(TIOCMGET): " << errno << " " << strerror(errno);
        throw(SerialException(ss.str().c_str()));
    }
    else
    {
        if (0 != (status & TIOCM_CTS)
         || 0 != (status & TIOCM_DSR)
         || 0 != (status & TIOCM_RI)
         || 0 != (status & TIOCM_CD))
        {
          return true;
        }
    }

    usleep(1000);
  }

  return false;
#else
  int command = (TIOCM_CD|TIOCM_DSR|TIOCM_RI|TIOCM_CTS);

  if (-1 == ioctl (fd_, TIOCMIWAIT, &command)) {
    stringstream ss;
    ss << "waitForDSR failed on a call to ioctl(TIOCMIWAIT): "
       << errno << " " << strerror(errno);
    throw(SerialException(ss.str().c_str()));
  }
  return true;
#endif
}

inline bool
Serial::SerialImpl::getCTS ()
{
  if (is_open_ == false) {
    throw PortNotOpenedException ("Serial::getCTS");
  }

  int status;

  if (-1 == ioctl (fd_, TIOCMGET, &status))
  {
    stringstream ss;
    ss << "getCTS failed on a call to ioctl(TIOCMGET): " << errno << " " << strerror(errno);
    throw(SerialException(ss.str().c_str()));
  }
  else
  {
    return 0 != (status & TIOCM_CTS);
  }
}

inline bool
Serial::SerialImpl::getDSR ()
{
  if (is_open_ == false) {
    throw PortNotOpenedException ("Serial::getDSR");
  }

  int status;

  if (-1 == ioctl (fd_, TIOCMGET, &status))
  {
      stringstream ss;
      ss << "getDSR failed on a call to ioctl(TIOCMGET): " << errno << " " << strerror(errno);
      throw(SerialException(ss.str().c_str()));
  }
  else
  {
      return 0 != (status & TIOCM_DSR);
  }
}

inline bool
Serial::SerialImpl::getRI ()
{
  if (is_open_ == false) {
    throw PortNotOpenedException ("Serial::getRI");
  }

  int status;

  if (-1 == ioctl (fd_, TIOCMGET, &status))
  {
    stringstream ss;
    ss << "getRI failed on a call to ioctl(TIOCMGET): " << errno << " " << strerror(errno);
    throw(SerialException(ss.str().c_str()));
  }
  else
  {
    return 0 != (status & TIOCM_RI);
  }
}

inline bool
Serial::SerialImpl::getCD ()
{
  if (is_open_ == false) {
    throw PortNotOpenedException ("Serial::getCD");
  }

  int status;

  if (-1 == ioctl (fd_, TIOCMGET, &status))
  {
    stringstream ss;
    ss << "getCD failed on a call to ioctl(TIOCMGET): " << errno << " " << strerror(errno);
    throw(SerialException(ss.str().c_str()));
  }
  else
  {
    return 0 != (status & TIOCM_CD);
  }
}

inline void
Serial::SerialImpl::readLock ()
{
  int result = pthread_mutex_lock(&this->read_mutex);
  if (result) {
    THROW (IOException, result);
  }
}

inline void
Serial::SerialImpl::readUnlock ()
{
  int result = pthread_mutex_unlock(&this->read_mutex);
  if (result) {
    THROW (IOException, result);
  }
}

inline void
Serial::SerialImpl::writeLock ()
{
  int result = pthread_mutex_lock(&this->write_mutex);
  if (result) {
    THROW (IOException, result);
  }
}

inline void
Serial::SerialImpl::writeUnlock ()
{
  int result = pthread_mutex_unlock(&this->write_mutex);
  if (result) {
    THROW (IOException, result);
  }
}

#endif // !defined(_WIN32)

////////////////////////////////////////////////////////////////////////////////

#if defined(__linux__)

/*
 * Copyright (c) 2014 Craig Lilley <cralilley@gmail.com>
 * This software is made available under the terms of the MIT licence.
 * A copy of the licence can be obtained from:
 * http://opensource.org/licenses/MIT
 */

#include <vector>
#include <string>
#include <sstream>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdarg>
#include <cstdlib>

#include <glob.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

//#include "serial/serial.h"

using serial::PortInfo;
using std::istringstream;
using std::ifstream;
using std::getline;
using std::vector;
using std::string;
using std::cout;
using std::endl;

static vector<string> glob(const vector<string>& patterns);
static string basename(const string& path);
static string dirname(const string& path);
static bool path_exists(const string& path);
static string realpath(const string& path);
static string usb_sysfs_friendly_name(const string& sys_usb_path);
static vector<string> get_sysfs_info(const string& device_path);
static string read_line(const string& file);
static string usb_sysfs_hw_string(const string& sysfs_path);
static string format(const char* format, ...);

inline vector<string>
glob(const vector<string>& patterns)
{
    vector<string> paths_found;

	if(patterns.size() == 0)
	    return paths_found;

    glob_t glob_results;

    int glob_retval = glob(patterns[0].c_str(), 0, NULL, &glob_results);

    vector<string>::const_iterator iter = patterns.begin();

    while(++iter != patterns.end())
    {
        glob_retval = glob(iter->c_str(), GLOB_APPEND, NULL, &glob_results);
    }

    for(int path_index = 0; path_index < glob_results.gl_pathc; path_index++)
    {
        paths_found.push_back(glob_results.gl_pathv[path_index]);
    }

    globfree(&glob_results);

    return paths_found;
}

inline string
basename(const string& path)
{
    size_t pos = path.rfind("/");

    if(pos == std::string::npos)
        return path;

    return string(path, pos+1, string::npos);
}

inline string
dirname(const string& path)
{
    size_t pos = path.rfind("/");

    if(pos == std::string::npos)
        return path;
    else if(pos == 0)
        return "/";

    return string(path, 0, pos);
}

inline bool
path_exists(const string& path)
{
    struct stat sb;

    if( stat(path.c_str(), &sb ) == 0 )
        return true;

    return false;
}

inline string
realpath(const string& path)
{
    char* real_path = realpath(path.c_str(), NULL);

    string result;

    if(real_path != NULL)
    {
        result = real_path;

        free(real_path);
    }

    return result;
}

inline string
usb_sysfs_friendly_name(const string& sys_usb_path)
{
    unsigned int device_number = 0;

    istringstream( read_line(sys_usb_path + "/devnum") ) >> device_number;

    string manufacturer = read_line( sys_usb_path + "/manufacturer" );

    string product = read_line( sys_usb_path + "/product" );

    string serial = read_line( sys_usb_path + "/serial" );

    if( manufacturer.empty() && product.empty() && serial.empty() )
        return "";

    return format("%s %s %s", manufacturer.c_str(), product.c_str(), serial.c_str() );
}

inline vector<string>
get_sysfs_info(const string& device_path)
{
    string device_name = basename( device_path );

    string friendly_name;

    string hardware_id;

    string sys_device_path = format( "/sys/class/tty/%s/device", device_name.c_str() );

    if( device_name.compare(0,6,"ttyUSB") == 0 )
    {
        sys_device_path = dirname( dirname( realpath( sys_device_path ) ) );

        if( path_exists( sys_device_path ) )
        {
            friendly_name = usb_sysfs_friendly_name( sys_device_path );

            hardware_id = usb_sysfs_hw_string( sys_device_path );
        }
    }
    else if( device_name.compare(0,6,"ttyACM") == 0 )
    {
        sys_device_path = dirname( realpath( sys_device_path ) );

        if( path_exists( sys_device_path ) )
        {
            friendly_name = usb_sysfs_friendly_name( sys_device_path );

            hardware_id = usb_sysfs_hw_string( sys_device_path );
        }
    }
    else
    {
        // Try to read ID string of PCI device

        string sys_id_path = sys_device_path + "/id";

        if( path_exists( sys_id_path ) )
            hardware_id = read_line( sys_id_path );
    }

    if( friendly_name.empty() )
        friendly_name = device_name;

    if( hardware_id.empty() )
        hardware_id = "n/a";

    vector<string> result;
    result.push_back(friendly_name);
    result.push_back(hardware_id);

    return result;
}

inline string
read_line(const string& file)
{
    ifstream ifs(file.c_str(), ifstream::in);

    string line;

    if(ifs)
    {
        getline(ifs, line);
    }

    return line;
}

inline string
format(const char* format, ...)
{
    va_list ap;

    size_t buffer_size_bytes = 256;

    string result;

    char* buffer = (char*)malloc(buffer_size_bytes);

    if( buffer == NULL )
        return result;

    bool done = false;

    unsigned int loop_count = 0;

    while(!done)
    {
        va_start(ap, format);

        int return_value = vsnprintf(buffer, buffer_size_bytes, format, ap);

        if( return_value < 0 )
        {
            done = true;
        }
        else if( return_value >= buffer_size_bytes )
        {
            // Realloc and try again.

            buffer_size_bytes = return_value + 1;

            char* new_buffer_ptr = (char*)realloc(buffer, buffer_size_bytes);

            if( new_buffer_ptr == NULL )
            {
                done = true;
            }
            else
            {
                buffer = new_buffer_ptr;
            }
        }
        else
        {
            result = buffer;
            done = true;
        }

        va_end(ap);

        if( ++loop_count > 5 )
            done = true;
    }

    free(buffer);

    return result;
}

inline string
usb_sysfs_hw_string(const string& sysfs_path)
{
    string serial_number = read_line( sysfs_path + "/serial" );

    if( serial_number.length() > 0 )
    {
        serial_number = format( "SNR=%s", serial_number.c_str() );
    }

    string vid = read_line( sysfs_path + "/idVendor" );

    string pid = read_line( sysfs_path + "/idProduct" );

    return format("USB VID:PID=%s:%s %s", vid.c_str(), pid.c_str(), serial_number.c_str() );
}

inline vector<PortInfo>
serial::list_ports()
{
    vector<PortInfo> results;

    vector<string> search_globs;
    search_globs.push_back("/dev/ttyACM*");
    search_globs.push_back("/dev/ttyS*");
    search_globs.push_back("/dev/ttyUSB*");
    search_globs.push_back("/dev/tty.*");
    search_globs.push_back("/dev/cu.*");

    vector<string> devices_found = glob( search_globs );

    vector<string>::iterator iter = devices_found.begin();

    while( iter != devices_found.end() )
    {
        string device = *iter++;

        vector<string> sysfs_info = get_sysfs_info( device );

        string friendly_name = sysfs_info[0];

        string hardware_id = sysfs_info[1];

        PortInfo device_entry;
        device_entry.port = device;
        device_entry.description = friendly_name;
        device_entry.hardware_id = hardware_id;

        results.push_back( device_entry );

    }

    return results;
}

#endif // defined(__linux__)


/*
 * Copyright (C) 2018 Ola Benderius
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

#ifndef SERIAL_PORT
#define SERIAL_PORT

#include <functional>
#include <memory>
#include <thread>

//#include <libserialport.h>

namespace serial {

  enum class mode {
    Read,
    ReadWrite,
    Write
  };

  class Port {
    public:
      Port(std::string const &, int32_t const, serial::mode m = serial::mode::ReadWrite) noexcept;
      ~Port() noexcept;
      Port(Port const &) = delete;
      Port(Port &&) = delete;
      Port &operator=(Port const &) = delete;
      Port &operator=(Port &&) = delete;

      void read(std::function<void(std::string const)>) noexcept;
      void write(std::string const &) noexcept;

    private:
      void readRunner();

      std::function<void(std::string const)> m_readDeligate;
      std::unique_ptr<serial::Serial> m_port{nullptr};
      std::thread m_readThread;
      bool m_isReading;
  };

  inline Port::Port(std::string const &portname, int32_t const baudrate, serial::mode mode) noexcept:
    m_readDeligate(),
    m_port(),
    m_readThread(),
    m_isReading(false)
    {
       m_port.reset(new serial::Serial(portname, baudrate, serial::Timeout::simpleTimeout(1000)));
       if (!m_port->isOpen()) {
         std::cerr << "Could not find port " << portname << "." << std::endl;
         return;
       }
    }

  inline Port::~Port() noexcept
  {
    bool needsToCloseThead{m_isReading};
    m_isReading = false;
    m_port->close();
    m_port.reset(nullptr);
    if (needsToCloseThead) {
      m_readThread.join();
    }
  }

  inline void Port::read(std::function<void(std::string const)> readDeligate) noexcept
  {
    m_isReading = true;
    m_readDeligate = readDeligate;
    m_readThread = std::thread(&Port::readRunner, this);
  }

  inline void Port::readRunner()
  {
    uint32_t const BUFFER_SIZE = 512;
    char buf[BUFFER_SIZE];
    while (m_isReading) {
      if (m_port->waitReadable()) {
          const uint32_t byteCount = m_port->available();
          m_port->read(reinterpret_cast<uint8_t*>(buf), (byteCount > BUFFER_SIZE) ? BUFFER_SIZE : byteCount);
          std::string data(buf, (byteCount > BUFFER_SIZE) ? BUFFER_SIZE : byteCount);
          m_readDeligate(data);
      }
    }
  }

  inline void Port::write(std::string const &data) noexcept
  {
    m_port->write(data);
  }

}

#endif

