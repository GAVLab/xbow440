
#include "xbow440/xbow440.h"
using namespace xbow440;

#define WIN32_LEAN_AND_MEAN 
#include "boost/date_time/posix_time/posix_time.hpp"

void DefaultProcessData(const ImuData& data) {
    std::cout << "IMU440 Packet:" << std::endl;
    std::cout << " Timestamp: " << data.receive_time;
    std::cout << " IMU Temperature: " << data.boardtemp << std::endl;
    std::cout << " Gyro Yaw: " << data.yawrate << std::endl;
    std::cout << " Gyro Roll: " << data.rollrate << std::endl;
    std::cout << " Gyro Pitch: " << data.pitchrate << std::endl;
    std::cout << " Accel X: " << data.ax << std::endl;
    std::cout << " Accel Y: " << data.ay << std::endl;
    std::cout << " Accel Z: " << data.az << std::endl;
    std::cout << std::endl;
}

double DefaultGetTime() {
	boost::posix_time::ptime present_time(boost::posix_time::microsec_clock::universal_time());
	boost::posix_time::time_duration duration(present_time.time_of_day());
	return duration.total_seconds();
}

XBOW440::XBOW440()
{	
	serial_port_=NULL;
	data_handler_=DefaultProcessData;
	time_handler_=DefaultGetTime;
	read_size_=31; // initially set to size of the largest packet
	reading_status_=false;
}

bool XBOW440::Connect(std::string port, int baudrate, long timeout) {
	serial_port_ = new serial::Serial(port,baudrate,timeout);

	if (!serial_port_->isOpen()){
		std::cout << "Serial port: " << port << " failed to open." << std::endl;
		delete serial_port_;
		serial_port_ = NULL;
		return false;
	} else {
		std::cout << "Serial port: " << port << " opened successfully." << std::endl;
		std::cout << "Searching for IMU..." << std::endl;
	}


	// look for Xbow by sending ping and waiting for response
	if (!Ping()){
		std::cout << "Xbow440 not found on port: " << port << std::endl;
		delete serial_port_;
		serial_port_ = NULL;
		return false;
	}

	// start reading
	StartReading();
	return true;
}

void XBOW440::Disconnect() {
	StopReading();
	serial_port_->close();
	delete serial_port_;
	serial_port_=NULL;
}

bool XBOW440::Ping(int numAttempts, long timeout) {
	// get the current timeout period
	long origTimeout=serial_port_->getTimeoutMilliseconds();
	// set new timeout
	serial_port_->setTimeoutMilliseconds(timeout);

	char buffer[1000];
	size_t found=string::npos;
	int bytesRead=0;

	// ping the Xbow and wait for a response
	while (numAttempts-->0){
		// send ping
		serial_port_->write("UUPK");
		// wait for response
		bytesRead=serial_port_->read(buffer,1000);
		// convert read data to string
		std::string buffer_string = "";
		buffer_string.append(buffer,bytesRead);

		// see if we got a ping response or a timeout
		found=buffer_string.find("PK");
		if (found!=string::npos) {
			serial_port_->setTimeoutMilliseconds(origTimeout);
			return true;
		}
	}

	// reset to original timeout
	serial_port_->setTimeoutMilliseconds(origTimeout);

	// no reponse found
	return false;
}

void XBOW440::StartReading() {
	// create thread to read from sensor
	reading_status_=true;
	read_thread_ptr_ = boost::shared_ptr<boost::thread > (new boost::thread(boost::bind(&XBOW440::ReadSerialPort, this)));
}

void XBOW440::StopReading() {
	reading_status_=false;
}

void XBOW440::ReadSerialPort() {
	char buffer[31];
	size_t len;
	double time_stamp;

	Resync();

	while (reading_status_) {
		len = serial_port_->read(buffer, read_size_);
		imu_data_.receive_time = time_handler_();

		// check for header and first of packet type
		if ((buffer[0]!='U')&&(buffer[1]!='U')&&(buffer[2]!=0x53)) {
			Resync();
			continue;
		}

		// parse packet
		Parse(buffer+5, buffer[3]);
	}

}

void XBOW440::Resync() {
	static const unsigned int kMaximumMessageSize = 256;
	std::string result="";
	size_t found=string::npos;
	bool synced=false;
	char data[kMaximumMessageSize];

	// make up to 5 attempts to resync
	for (int ii=0; ii<5; ii++){
		std::cout << "Resyncing.  Attempt # " << (ii+1) << " of 5." << std::endl;
		result=serial_port_->read_until("UU");
		found=result.find("UU");
		if (found!=string::npos){
			
			size_t len = serial_port_->read(data,3);
			if (len<3)
				continue;
			size_t dataSize = data[2];
			//read_size_ = dataSize+7;
			len = serial_port_->read(data,dataSize+2);
			if (len==dataSize+2) {
				synced=true;
				break;
			}
		}
	}

	if (synced) {
		std::cout << "Successfully synchronized." << std::endl;
	} else {
		std::cout << "Failed to synchronize. Stopping sensor." << std::endl;
		StopReading();
		Disconnect();
	}
}

/*void XBOW440::SoftwareReset()
{
	if (serial_port_==NULL)
		return;

	if (!serial_port_->isOpen())
		return;

	char msgToSend[7];
	msgToSend[0]=0x55;
	msgToSend[1]=0x55;
	// preamble SR
	msgToSend[2]=0x53;
	msgToSend[3]=0x62;
	// length
	msgToSend[4]=0;
	// crc
	msgToSend[5]=0x7e;
	msgToSend[6]=0x4f;

	// send command to device
	serial_port_->write(msgToSend,7);
} */

void XBOW440::Parse(char *data, unsigned short packet_type) {
    
	// scale factors for converting raw data
	static const double kAccelerometerScaleFactorS1 = 0.002992752075195; // scale for m/s^2
	static const double kGyroscopeScaleFactorS1 = 0.000335558297349984; // scale for rad/s
	static const double kTemperatureScaleFactorS1 = 0.0030517578125;
	static const double kAccelerometerScaleFactorS2 = 0.00000004656612873077393;
	static const double kGyroscopeScaleFactorS2 = 0.000000005120213277435059;

	// TODO: check CRC

    switch (packet_type) {
	case 0x31: //S1 Payload

	    imu_data_.datatype=packet_type;

	    s1contents.cdata[0] = data[1];
	    s1contents.cdata[1] = data[0];
	    imu_data_.ax = s1contents.ssdata*kAccelerometerScaleFactorS1;

	    s1contents.cdata[0] = data[3];
	    s1contents.cdata[1] = data[2];
	    imu_data_.ay = s1contents.ssdata*kAccelerometerScaleFactorS1;

	    s1contents.cdata[0] = data[5];
	    s1contents.cdata[1] = data[4];
	    imu_data_.az = s1contents.ssdata*kAccelerometerScaleFactorS1;

	    s1contents.cdata[0] = data[7];
	    s1contents.cdata[1] = data[6];
	    imu_data_.rollrate = s1contents.ssdata*kGyroscopeScaleFactorS1;

	    s1contents.cdata[0] = data[9];
	    s1contents.cdata[1] = data[8];
	    imu_data_.pitchrate = s1contents.ssdata*kGyroscopeScaleFactorS1;

	    s1contents.cdata[0] = data[11];
	    s1contents.cdata[1] = data[10];
	    imu_data_.yawrate = s1contents.ssdata*kGyroscopeScaleFactorS1;

	    s1contents.cdata[0] = data[13];
	    s1contents.cdata[1] = data[12];
	    imu_data_.xtemp = s1contents.ssdata*kTemperatureScaleFactorS1;

	    s1contents.cdata[0] = data[15];
	    s1contents.cdata[1] = data[14];
	    imu_data_.ytemp = s1contents.ssdata*kTemperatureScaleFactorS1;

	    s1contents.cdata[0] = data[17];
	    s1contents.cdata[1] = data[16];
	    imu_data_.ztemp = s1contents.ssdata*kTemperatureScaleFactorS1;

	    s1contents.cdata[0] = data[19];
	    s1contents.cdata[1] = data[18];
	    imu_data_.boardtemp = s1contents.ssdata*kTemperatureScaleFactorS1;

	    s1contents.cdata[0] = data[21];
	    s1contents.cdata[1] = data[20];
	    imu_data_.counter = s1contents.usdata;

	    s1contents.cdata[0] = data[23];
	    s1contents.cdata[1] = data[22];
	    imu_data_.bitstatus = s1contents.usdata;

	    break;

	case 0x32: //S2 Payload
	    
	    s2contents.cdata[0] = data[3];
	    s2contents.cdata[1] = data[2];
	    s2contents.cdata[2] = data[1];
	    s2contents.cdata[3] = data[0];
	    imu_data_.ax = s2contents.sidata*kAccelerometerScaleFactorS2;

	    s2contents.cdata[0] = data[7];
	    s2contents.cdata[1] = data[6];
	    s2contents.cdata[2] = data[5];
	    s2contents.cdata[3] = data[4];
	    imu_data_.ay = s2contents.sidata*kAccelerometerScaleFactorS2;

	    s2contents.cdata[0] = data[11];
	    s2contents.cdata[1] = data[10];
	    s2contents.cdata[2] = data[9];
	    s2contents.cdata[3] = data[8];
	    imu_data_.az = s2contents.sidata*kAccelerometerScaleFactorS2;

	    s2contents.cdata[0] = data[15];
	    s2contents.cdata[1] = data[14];
	    s2contents.cdata[2] = data[13];
	    s2contents.cdata[3] = data[12];
	    imu_data_.rollrate = s2contents.sidata*kGyroscopeScaleFactorS2;

	    s2contents.cdata[0] = data[19];
	    s2contents.cdata[1] = data[18];
	    s2contents.cdata[2] = data[17];
	    s2contents.cdata[3] = data[16];
	    imu_data_.pitchrate = s2contents.sidata*kGyroscopeScaleFactorS2;

	    s2contents.cdata[0] = data[23];
	    s2contents.cdata[1] = data[22];
	    s2contents.cdata[2] = data[21];
	    s2contents.cdata[3] = data[20];
	    imu_data_.yawrate = s2contents.sidata*kGyroscopeScaleFactorS2;

	    s2contents.cdata[0] = data[25];
	    s2contents.cdata[1] = data[24];
	    s2contents.cdata[2] = '0';
	    s2contents.cdata[3] = '0';
	    imu_data_.counter = s2contents.usdata;

	    s2contents.cdata[0] = data[27];
	    s2contents.cdata[1] = data[26];
	    s2contents.cdata[2] = '0';
	    s2contents.cdata[3] = '0';
	    imu_data_.bitstatus = s2contents.usdata;

	    break;
	default:
		std::cout << "Unsupported packet type." << std::endl;
    }

    // call callback with data
    if (data_handler_!=NULL)
    	data_handler_(imu_data_);
    
}

/*bool XBOW440::SetOutputRate(unsigned short rate) {

	if (serialPort==NULL)
		return false;

	if (!serialPort->isOpen())
		return false;

	//QUEUE_TYPE queueOutRate;
	//InitializeQueue(&queueOutRate);
	char msgToSend[12];
	// preamble 0x5555
	msgToSend[0]=0x55;
	msgToSend[1]=0x55;
	// packet type 0x5746
	msgToSend[2]=0x57;
	msgToSend[3]=0x46;
	// length 1+numFields*4
	msgToSend[4]=5;
	// num fields
	msgToSend[5]=1;
	// field id 0x0001
	msgToSend[6]=0x00;
	msgToSend[7]=0x01;
	// field data
	msgToSend[8]=(rate>>8)&0x00FF;
	msgToSend[9]=rate&0x00FF;
	// crc 2 bytes
	unsigned short crc=calcCRC(msgToSend+2, 8);
	
	msgToSend[10]=((crc<<8)&0x00FF);
	msgToSend[11]=(crc&0x00FF);
	  
	//cout << dec << queueOutRate.count << " : " << queueOutRate.front << endl;
	serialPort->write(msgToSend,12);
	return true;
}*/

/*******************************************************************************
* FUNCTION: calcCRC calculates a 2-byte CRC on serial data using
* CRC-CCITT 16-bit standard maintained by the ITU
* (International Telecommunications Union).
* ARGUMENTS: queue_ptr is pointer to queue holding area to be CRCed
* startIndex is offset into buffer where to begin CRC calculation
* num is offset into buffer where to stop CRC calculation
* RETURNS: 2-byte CRC
*******************************************************************************/
unsigned short XBOW440::CalculateCRC(char* data, int length){
    unsigned int i = 0, j = 0;
    unsigned short crc = 0x1D0F; //non-augmented inital value equivalent to augmented initial value 0xFFFF
	// calculate crc on remainder of packet
    for (i = 0; i < length; i += 1) {
		crc ^= data[i] << 8;
		//cout << hex << (int)peekByte(queue_ptr, startIndex + i) << endl;
		for (j = 0; j < 8; j += 1) {
			if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
			else crc = crc << 1;
		}
    }
    return crc;
}

