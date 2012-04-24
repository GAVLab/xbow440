/*!
 * \file include/Xbow440.h
 * \author David Hodo <david.hodo@gmail.com>
 * \version 0.1
 *
 * \section LICENSE
 *
 * The BSD License
 *
 * Copyright (c) 2011 David Hodo
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
 * This provides an interface for the Crossbow IMU440 inertial measurement unit.
 * 
 * This library depends on CMake-2.4.6 or later: http://www.cmake.org/
 * This library depends on Serial: https://github.com/wjwwood/serial
 * 
 */


#ifndef XBOW440_H
#define XBOW440_H


#include <string>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <queue>
using namespace std;

// Boost Headers
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#define SERIAL_LISTENER_DEBUG 0

// Serial Headers
#include "serial/serial.h"
using namespace serial;


namespace xbow440{

// crossbow constants

/* buffer size */
#define MAXQUEUE 2000
#define MAX_MSG_SIZE 256

// scale factors for converting raw data
#define S1_ACCEL_SCALE 0.002992752075195     // scale for m/s^2
#define S1_GYRO_SCALE 0.000335558297349984 // scale for rad/s
#define S1_TEMP_SCALE 0.0030517578125
#define S2_ACCEL_SCALE 0.00000004656612873077393
#define S2_GYRO_SCALE 0.000000005120213277435059

union s1union {
    signed short ssdata;
    unsigned short usdata; //packets
    char cdata[2];
};

union s2union {
    signed int sidata;
    unsigned short usdata; //packets
    char cdata[4];
};


//accelerations and turning rates
struct imuData {
    int datatype; //1 for accels and turning rates, 2 for delta v's and delta thetas.
    double receive_time;
    double ax; //1:g's 2:m/s
    double ay; //1:g's 2:m/s
    double az; //1:g's 2:m/s
    double rollrate; //1:rad/s 2:rad
    double pitchrate; //1:rad/s 2:rad
    double yawrate; //1:rad/s 2:rad
    double xtemp; //degC
    double ytemp; //degC
    double ztemp; //degC
    double boardtemp; //degC
    unsigned short counter; //packets
    unsigned short bitstatus;
};


/*!
 * This function type describes the prototype for the logging callbacks.
 * 
 * The function takes a std::string reference and returns nothing.  It is 
 * called from the library when a logging message occurs.  This 
 * allows the library user to hook into this and integrate it with their own 
 * logging system.  It can be set with any of the set<log level>Handler 
 * functions.
 * 
 * \see SerialListener::setInfoHandler, SerialListener::setDebugHandler, 
 * SerialListener::setWarningHandler
 */
typedef boost::function<void(const std::string&)> LoggingCallback;

/*!
 * This function type describes the prototype for the exception callback.
 * 
 * The function takes a std::exception reference and returns nothing.  It is 
 * called from the library when an exception occurs in a library thread.
 * This exposes these exceptions to the user so they can to error handling.
 * 
 * \see SerialListener::setExceptionHandler
 */
typedef boost::function<void(const std::exception&)> ExceptionCallback;



class XBOW440{
public:
	
	XBOW440();

	~XBOW440(){};
	
    /*!
     * Connects to the XBOW440 IMU given a serial port.
     * 
     * \param port Defines which serial port to connect to in serial mode.
     * Examples: Linux - "/dev/ttyS0" Windows - "COM1"
     * 
     * \throws ConnectionFailedException connection attempt failed.
     * \throws UnknownErrorCodeException unknown error code returned.
     */
    void connect(std::string port, int baudrate=115200, long timeout=0);

   /*!
    * Disconnects from the serial port
    */
    void disconnect();

	bool SetOutputRate(unsigned short rate); //!< set rate to 0, 1, 2, 5, 10, 20, 25, 50Hz

    bool ping(int numAttempts=5, long timeout=100);

private:
    // Serial port members
    Serial *serialPort;

    //! Houses latest data; populated in parse()
    imuData imudata;
    s1union s1contents;
    s2union s2contents;

    bool start_reading();
    bool stop_reading();
    void read_thread();
    void resync();

    void BufferIncomingData(unsigned char* msg, unsigned int length); //!< data read from serial port is passed to this method
    void parse(char *data, unsigned short packet_type);
    void compile();
    unsigned short calcCRC(char* data, int length);
    void SoftwareReset(); //!< performs a software reset - needed to make changes to settings
    size_t readSize; // number of bytes to read at a time
    boost::shared_ptr<boost::thread> mReadThread; //!< Boost thread for listening for data from xbow  
    bool bReading;
};

}; // end namespace

#endif
