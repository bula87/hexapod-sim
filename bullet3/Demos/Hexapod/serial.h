#include <string>   /* ERROR Number Definitions           */
#include <iostream>

class serial
{
public:
    serial(std::string port);
    ~serial();
    bool openSerial(std::string port);
    bool isPortOpen();
    bool setupSerial();
    bool closeSerial();
    std::string readSerial();
private:
    int serialFD_;
    std::string port_;
};
