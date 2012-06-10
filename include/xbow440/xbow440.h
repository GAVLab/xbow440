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

// Serial Headers
#include "serial/serial.h"

namespace xbow440{


union ShortsUnion {
    signed short ssdata;
    unsigned short usdata; //packets
    char cdata[2];
};

union IntsUnion {
    signed int sidata;
    unsigned short usdata; //packets
    char cdata[4];
};


//accelerations and turning rates
struct ImuData {
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
 * @see SerialListener::setInfoHandler, SerialListener::setDebugHandler, 
 * SerialListener::setWarningHandler
 */
typedef boost::function<void(const std::string&)> LoggingCallback;

/*!
 * This function type describes the prototype for the data callback.
 * 
 * The function takes a xbow440::imuData reference and returns nothing.  It is 
 * called from the library when new data arrives and is parsed.
 * 
 * @see XBOW440::setDataHandler
 */
typedef boost::function<void(const xbow440::ImuData&)> DataCallback;


typedef boost::function<double()> GetTimeCallback;



class XBOW440{
public:
	
	XBOW440();

	~XBOW440(){};
	
    /*!
     * Connects to the XBOW440 IMU given a serial port.
     * 
     * @param port Defines which serial port to connect to in serial mode.
     * Examples: Linux - "/dev/ttyS0" Windows - "COM1"
     * 
     * @throws ConnectionFailedException connection attempt failed.
     * @throws UnknownErrorCodeException unknown error code returned.
     */
    bool Connect(std::string port, int baudrate=115200, long timeout=50);

   /*!
    * Disconnects from the serial port
    */
    void Disconnect();


   /*!
    * Pings the IMU to determine if it is properly connected
    * 
    * This method sends a ping to the IMU and waits for a response.
    * 
    * @param num_attempts The number of times to ping the device
    * before giving up
    * @param timeout The time in milliseconds to wait for each reponse
    *
    * @return True if the IMU was found, false if it was not.
    * 
    * @see xbow440::DataCallback
    */
    bool Ping(int num_attempts=5);

   /*!
    * Sets the handler to be called when a new data is received.
    * 
    * This allows you to set a catch all function that will get called 
    * everytime a new data packet is received from the IMU.
    * 
    * \param default_handler A function pointer to the callback to handle 
    * parsed IMU data.
    * 
    * \see xbow440::DataCallback
    */
    void set_data_handler(DataCallback data_handler) {
        this->data_handler_ = data_handler;
    }

    void set_time_handler(GetTimeCallback time_handler) {
        this->time_handler_ = time_handler;
    }

    // NOT IMPLEMENTED YET
    //bool SetOutputRate(unsigned short rate); //!< set rate to 0, 1, 2, 5, 10, 20, 25, 50Hz

private:

   /*!
    * Starts a thread to continuously read from the serial port.
    * 
    * Starts a thread that runs 'ReadSerialPort' which constatly reads
    * from the serial port.  When valid data is received, parse and then
    *  the data callback functions are called.
    * 
    * @see xbow440::DataCallback, xbow440::XBOW440::ReadSerialPort, xbow440::XBOW440::StopReading
    */
    void StartReading();

   /*!
    * Starts the thread that reads from the serial port
    * 
    * @see xbow440::XBOW440::ReadSerialPort, xbow440::XBOW440::StartReading
    */
    void StopReading();

   /*!
    * Method run in a seperate thread that continuously reads from the
    * serial port.  When a complete packet is received, the parse 
    * method is called to process the data
    * 
    * @see xbow440::XBOW440::Parse, xbow440::XBOW440::StartReading, xbow440::XBOW440::StopReading
    */    
    void ReadSerialPort();

   /*!
    * Resynchronizes the serial port so that each read of a set number
    * of bytes returns a complete data packet.
    */
    void Resync();

   /*!
    * Parses a packet of data from the IMU.  Scale factors are 
    * also applied to the data to convert into engineering units.
    */
    void Parse(unsigned char *data, unsigned short packet_type);
   /*!
    * Calculated a cyclic redundancy check (CRC) on the received
    * data to check for errors in data transmission.
    */    
    unsigned short CalculateCRC(char* data, unsigned int length);
    //void SoftwareReset(); //!< performs a software reset - needed to make changes to settings

    //! Serial port object for communicating with sensor
    serial::Serial *serial_port_;
    //! most recently parsed IMU data
    ImuData imu_data_;      
    ShortsUnion s1contents;    //!< union for converting bytes to shorts
    IntsUnion s2contents;    //!< union for converting bytes to ints

   /*!
    * The number of bytes read during each call to read on the serial
    * port.  This value is set intially to 31 (assumes an S2 packet), 
    * but is modified by Resync to match the size of the packets actually
    * being received.
    */  
    size_t read_size_;
    //! shared pointer to Boost thread for listening for data from xbow 
    boost::shared_ptr<boost::thread> read_thread_ptr_;  
    bool reading_status_;  //!< True if the read thread is running, false otherwise.
    DataCallback data_handler_; //!< Function pointer to callback function for parsed data
    GetTimeCallback time_handler_; //!< Function pointer to callback function for timestamping
};

}; // end namespace

#endif
