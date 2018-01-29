#include <algorithm>
#include <iostream>

#include "md25.h"

char * getCmdOption(char ** begin, char ** end, const std::string & option) {

    char ** itr = std::find(begin, end, option);

    if (itr != end && ++itr != end) {
        return * itr;
    }

    return NULL;

}

bool cmdOptionExists(char** begin, char** end, const std::string& option) {
    return std::find(begin, end, option) != end;
}

int main(int argc, char * argv[]) {

    // Set default parameters
    std::string port = "/dev/ttyUSB0";
    uint32_t baudrate = 38400;
    uint32_t timeout = 1000;

    // Get port from command line
    char * _port = getCmdOption(argv, argv + argc, "-p");

    if (_port) {
        port = _port;
    }

    // Get baudrate from command line
    char * _baudarate = getCmdOption(argv, argv + argc, "-b");

    if (_baudarate) {
        baudrate = std::atoi(_baudarate);
    }

    // Get timeout from command line
    char * _timeout = getCmdOption(argv, argv + argc, "-t");

    if (_timeout) {
        timeout = std::atoi(_timeout);
    }

    // Create MD25
    MD25 md25(port, baudrate, timeout);

    // Log
    std::cout << "Cooneccting to port " << port << " with baudrate " << baudrate << " and timeout " << timeout << " ..." << std::endl;

    uint8_t result;

    // Init MD25
    result == md25.init();

    if (result == MD25_RESPONSE_OK) {

        if (cmdOptionExists(argv, argv + argc, "getversion")) {

            uint8_t version;

            if (md25.getVersion(& version) == MD25_RESPONSE_OK) {

                // Log
                std::cout << "Version: " << unsigned(version) << std::endl;

            } else {

                // Log
                std::cerr << "Error getting version" << std::endl;

                return -1;

            }

        }

        if (cmdOptionExists(argv, argv + argc, "getspeed")) {

            uint8_t speed1, speed2;

            if (md25.getSpeed1(& speed1) == MD25_RESPONSE_OK &&
                    md25.getSpeed2(& speed2) == MD25_RESPONSE_OK) {

                // Log
                std::cout << "Speed1: " << unsigned(speed1) << ", speed2: " << unsigned(speed2) << std::endl;

            } else {

                // Log
                std::cerr << "Error getting speeds" << std::endl;

                return -1;

            }

        }

        if (cmdOptionExists(argv, argv + argc, "getencoders")) {

            uint32_t encoder1, encoder2;

            if (md25.getEncoders(& encoder1, & encoder2) == MD25_RESPONSE_OK) {

                // Log
                std::cout << "Encoder1: " << unsigned(encoder1) << ", encoder2: " << unsigned(encoder2) << std::endl;

            } else {

                // Log
                std::cerr << "Error getting encoders" << std::endl;

                return -1;

            }

        }

        char * speed1 = getCmdOption(argv, argv + argc, "setspeed1");

        if (speed1) {

            if (md25.setSpeed1(atoi(speed1)) != MD25_RESPONSE_OK) {

                // Log
                std::cerr << "Error setting speed1" << std::endl;

                return -1;

            }

        }

        char * speed2 = getCmdOption(argv, argv + argc, "setspeed2");

        if (speed2) {

            if (md25.setSpeed2(atoi(speed2)) != MD25_RESPONSE_OK) {

                // Log
                std::cerr << "Error setting speed2" << std::endl;

                return -1;

            }

        }

        char * mode = getCmdOption(argv, argv + argc, "setmode");

        if (mode) {

            if (md25.setMode(atoi(mode)) != MD25_RESPONSE_OK) {

                // Log
                std::cerr << "Error setting mode" << std::endl;

                return -1;

            }

        }

        char * acceleration = getCmdOption(argv, argv + argc, "setacceleration");

        if (acceleration) {

            if (md25.setAcceleration(atoi(acceleration)) != MD25_RESPONSE_OK) {

                // Log
                std::cerr << "Error setting acceleration" << std::endl;

                return -1;

            }

        }

        if (cmdOptionExists(argv, argv + argc, "disableregulator")) {

            if (md25.disableRegulator() != MD25_RESPONSE_OK) {

                // Log
                std::cerr << "Error disabling regulator" << std::endl;

                return -1;

            }

        }

        if (cmdOptionExists(argv, argv + argc, "enableregulator")) {

            if (md25.enabledRegulator() != MD25_RESPONSE_OK) {

                // Log
                std::cerr << "Error enabling regulator" << std::endl;

                return -1;

            }

        }

        if (cmdOptionExists(argv, argv + argc, "disabletimeout")) {

            if (md25.disableTimeout() != MD25_RESPONSE_OK) {

                // Log
                std::cerr << "Error disabling timeout" << std::endl;

                return -1;

            }

        }

        if (cmdOptionExists(argv, argv + argc, "enabletimeout")) {

            if (md25.enabledTimeout() != MD25_RESPONSE_OK) {

                // Log
                std::cerr << "Error enabling timeout" << std::endl;

                return -1;

            }

        }



        // Dispose md25
        md25.dispose();

        return 0;

    } else {

        // Log
        std::cerr << "Cannot connect to MD25" << std::endl;

        return -1;

    }

}

