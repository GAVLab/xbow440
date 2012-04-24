#include <string>
#include <iostream>

#include "mdc2250/mdc2250.h"
using namespace mdc2250;
using namespace std;

int main(int argc, char **argv)
{
    if(argc < 2) {
        std::cerr << "Usage: mdc2250_example <serial port address>" << std::endl;
        return 0;
    }
    std::string port(argv[1]);

    std::cout << "WARNING: This example moves the ATRV. \n Do you wish to continue (Y or N)?" << std::endl;
    char reply;
    std::cin >> reply;

    MDC2250 myMDC;
    bool result = myMDC.connect(port);

    if (result) {
        cout << "Successfully connected." << endl;
    }
    else {
        cout << "Failed to connect." << endl;
        return -1;
    }

    // set up encoder
    myMDC.setEncoderPPR(1,500);
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    myMDC.setEncoderPPR(2,500);

    // set up to read BA, FF, S, CR
    myMDC.sendCommand("\r# C_?BA_?FF_?S_?C_# 200\r");
    myMDC.startContinuousReading();

    if (reply=='Y') {

        // run motor
        for (int ii=0; ii<20; ii++)
        {
            //std::cout << ii << std::endl;
            myMDC.multiMotorCmd(400,400);
            boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        }
        for (int ii=0; ii<20; ii++)
        {
            //std::cout << ii << std::endl;
            myMDC.multiMotorCmd(100,100);
            boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        }
        for (int ii=0; ii<20; ii++)
        {
            //std::cout << ii << std::endl;
            myMDC.multiMotorCmd(400,400);
            boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        }

        myMDC.multiMotorCmd(0,0);
    } else {
        std::cout << "Running without motion." << std::endl;
    }

    while(1);

    myMDC.disconnect();

    return 0;
}
