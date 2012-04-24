
#include "xbow440.h"
using namespace xbow440;


XBOW440::XBOW440()
{	
	serialPort=NULL;
	readSize=31; // initially set to size of the largest packet
	bReading=false;
}
    
void XBOW440::connect(std::string port, int baudrate, long timeout) {
	serialPort = new Serial(port,baudrate,timeout);

	if (~serialPort->isOpen()){
		std::cout << "Serial port: " << port << " failed to open." << std::endl;
		delete serialPort;
		serialPort=NULL;
		return;
	}

	// look for Xbow by sending ping and waiting for response
	if (!ping()){
		std::cout << "Xbow440 not found on port: " << port << std::endl;
		delete serialPort;
		serialPort=NULL;
		return;
	}

	// start reading
	start_reading();
}

bool XBOW440::ping(int numAttempts, long timeout) {
	// get the current timeout period
	long origTimeout=serialPort->getTimeoutMilliseconds();
	// set new timeout
	serialPort->setTimeoutMilliseconds(timeout);

	std::string result="";
	size_t found=string::npos;

	// ping the Xbow and wait for a response
	while (numAttempts-->0){
		// send ping
		serialPort->write("UUPK");
		// wait for response
		result=serialPort->read_until("PK");

		// see if we got a ping response or a timeout
		found=result.find("PK");
		if (found!=string::npos) {
			serialPort->setTimeoutMilliseconds(origTimeout);
			return true;
		}
	}

	// reset to original timeout
	serialPort->setTimeoutMilliseconds(origTimeout);

	// no reponse found
	return false;
}

bool XBOW440::start_reading() {
	// create thread to read from sensor
	bReading=true;
	mReadThread = boost::shared_ptr<boost::thread > (new boost::thread(boost::bind(&XBOW440::read_thread, this)));
	return true;
}

bool XBOW440::stop_reading() {
	bReading=false;
	return true;
}

void XBOW440::read_thread() {
	char buffer[31];
	size_t len;

	resync();

	while (bReading) {
		len = serialPort->read(buffer, readSize);

		// check for header and first of packet type
		if ((buffer[0]!='U')&&(buffer[1]!='U')&&(buffer[2]!=0x53)) {
			resync();
			continue;
		}

		// parse packet
		parse(buffer+6, buffer[3]);
		

	}

}

void XBOW440::resync() {
	std::string result="";
	size_t found=string::npos;
	bool synced=false;
	char data[MAX_MSG_SIZE];

	// make up to 5 attempts to resync
	for (int ii=0; ii<5; ii++){
		std::cout << "Resyncing.  Attempt # " << (ii+1) << " of 5." << std::endl;
		result=serialPort->read_until("PK");
		found=result.find("PK");
		if (found!=string::npos){
			
			size_t len = serialPort->read(data,3);
			if (len<3)
				continue;
			size_t dataSize = data[2];
			readSize = dataSize+7;
			len = serialPort->read(data,dataSize+2);
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
		stop_reading();
		disconnect();
	}
}

void XBOW440::SoftwareReset()
{
	if (serialPort==NULL)
		return;

	if (!serialPort->isOpen())
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
	serialPort->write(msgToSend,7);
}

void XBOW440::parse(char *data, unsigned short packet_type) {
    
    switch (packet_type) {
	case 0x31: //S1 Payload

	    imudata.datatype=packet_type;

	    s1contents.cdata[0] = data[1];
	    s1contents.cdata[1] = data[0];
	    imudata.ax = s1contents.ssdata*S1_ACCEL_SCALE;

	    s1contents.cdata[0] = data[3];
	    s1contents.cdata[1] = data[2];
	    imudata.ay = s1contents.ssdata*S1_ACCEL_SCALE;

	    s1contents.cdata[0] = data[5];
	    s1contents.cdata[1] = data[4];
	    imudata.az = s1contents.ssdata*S1_ACCEL_SCALE;

	    s1contents.cdata[0] = data[7];
	    s1contents.cdata[1] = data[6];
	    imudata.rollrate = s1contents.ssdata*S1_GYRO_SCALE;

	    s1contents.cdata[0] = data[9];
	    s1contents.cdata[1] = data[8];
	    imudata.pitchrate = s1contents.ssdata*S1_GYRO_SCALE;

	    s1contents.cdata[0] = data[11];
	    s1contents.cdata[1] = data[10];
	    imudata.yawrate = s1contents.ssdata*S1_GYRO_SCALE;

	    s1contents.cdata[0] = data[13];
	    s1contents.cdata[1] = data[12];
	    imudata.xtemp = s1contents.ssdata*S1_TEMP_SCALE;

	    s1contents.cdata[0] = data[15];
	    s1contents.cdata[1] = data[14];
	    imudata.ytemp = s1contents.ssdata*S1_TEMP_SCALE;

	    s1contents.cdata[0] = data[17];
	    s1contents.cdata[1] = data[16];
	    imudata.ztemp = s1contents.ssdata*S1_TEMP_SCALE;

	    s1contents.cdata[0] = data[19];
	    s1contents.cdata[1] = data[18];
	    imudata.boardtemp = s1contents.ssdata*S1_TEMP_SCALE;

	    s1contents.cdata[0] = data[21];
	    s1contents.cdata[1] = data[20];
	    imudata.counter = s1contents.usdata;

	    s1contents.cdata[0] = data[23];
	    s1contents.cdata[1] = data[22];
	    imudata.bitstatus = s1contents.usdata;

	    break;

	case 0x32: //S2 Payload
	    
	    s2contents.cdata[0] = data[3];
	    s2contents.cdata[1] = data[2];
	    s2contents.cdata[2] = data[1];
	    s2contents.cdata[3] = data[0];
	    imudata.ax = s2contents.sidata*S2_ACCEL_SCALE;

	    s2contents.cdata[0] = data[7];
	    s2contents.cdata[1] = data[6];
	    s2contents.cdata[2] = data[5];
	    s2contents.cdata[3] = data[4];
	    imudata.ay = s2contents.sidata*S2_ACCEL_SCALE;

	    s2contents.cdata[0] = data[11];
	    s2contents.cdata[1] = data[10];
	    s2contents.cdata[2] = data[9];
	    s2contents.cdata[3] = data[8];
	    imudata.az = s2contents.sidata*S2_ACCEL_SCALE;

	    s2contents.cdata[0] = data[15];
	    s2contents.cdata[1] = data[14];
	    s2contents.cdata[2] = data[13];
	    s2contents.cdata[3] = data[12];
	    imudata.rollrate = s2contents.sidata*S2_GYRO_SCALE;

	    s2contents.cdata[0] = data[19];
	    s2contents.cdata[1] = data[18];
	    s2contents.cdata[2] = data[17];
	    s2contents.cdata[3] = data[16];
	    imudata.pitchrate = s2contents.sidata*S2_GYRO_SCALE;

	    s2contents.cdata[0] = data[23];
	    s2contents.cdata[1] = data[22];
	    s2contents.cdata[2] = data[21];
	    s2contents.cdata[3] = data[20];
	    imudata.yawrate = s2contents.sidata*S2_GYRO_SCALE;

	    s2contents.cdata[0] = data[25];
	    s2contents.cdata[1] = data[24];
	    s2contents.cdata[2] = '0';
	    s2contents.cdata[3] = '0';
	    imudata.counter = s2contents.usdata;

	    s2contents.cdata[0] = data[27];
	    s2contents.cdata[1] = data[26];
	    s2contents.cdata[2] = '0';
	    s2contents.cdata[3] = '0';
	    imudata.bitstatus = s2contents.usdata;

	    break;
	default:
		std::cout << "Unsupported packet type." << std::endl;
    }

    // call callback with data
    
}

bool XBOW440::SetOutputRate(unsigned short rate) {

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
}

/*******************************************************************************
* FUNCTION: calcCRC calculates a 2-byte CRC on serial data using
* CRC-CCITT 16-bit standard maintained by the ITU
* (International Telecommunications Union).
* ARGUMENTS: queue_ptr is pointer to queue holding area to be CRCed
* startIndex is offset into buffer where to begin CRC calculation
* num is offset into buffer where to stop CRC calculation
* RETURNS: 2-byte CRC
*******************************************************************************/
unsigned short XBOW440::calcCRC(char* data, int length){
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

