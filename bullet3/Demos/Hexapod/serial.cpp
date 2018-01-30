#include <stdio.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	   */ 
#include <errno.h>   /* ERROR Number Definitions           */
#include "serial.h"

serial::serial(std::string port):serialFD_(-1),port_("")
{
  openSerial(port);
  if(-1 == serialFD_)
  {
    std::cout << "Can not open serial port (" << port << ")" << std::endl;
  }
  else
  {
    std::cout << "Serial port opened successfully (" << port << ")" << std::endl;
    if(setupSerial())
    {
      std::cout << "BaudRate = 9600 " << std::endl
                << "StopBits = 1 " << std::endl
                << "Parity   = none" << std::endl;
    }
    else
    {
      std::cout << "Can not configure serial port (" << port << ")" << std::endl;
    }
  }
}

serial::~serial()
{
  closeSerial();
}

bool serial::openSerial(std::string port)
{
  bool result = false;
  if(-1 == serialFD_)
  {
    serialFD_ = open(port.c_str(),O_RDWR | O_NOCTTY);	/* Open Serial port */
    /* O_RDWR   - Read/Write access to serial port       */
    /* O_NOCTTY - No terminal will control the process   */
    /* Open in blocking mode,read will wait              */
    if(-1 == serialFD_)
    {
      result = true;
      port_ = port;
    }
  }
  else
  {
    std::cout << "Serial port is already open (" << port << ")" << std::endl;
  }
  return result;
}

bool serial::isPortOpen()
{
  bool result = false;
  if(-1 != serialFD_)
  {
    result = true;
  }
  return result;
}

bool serial::closeSerial()
{
  bool result = false;
  if(-1 != serialFD_)
  {
    if(0 == close(serialFD_))
    {
      result = true;
      port_ = "";
    }
  }
  return result;
}

bool serial::setupSerial()
{
  bool result = false;
  if(-1 != serialFD_)
  {
    /*---------- Setting the Attributes of the serial port using termios structure --------- */

    struct termios SerialPortSettings;	// Create the structure
    tcgetattr(serialFD_, &SerialPortSettings);	// Get the current attributes of the Serial port

    /* Setting the Baud rate */
    cfsetispeed(&SerialPortSettings,B9600); // Set Read  Speed as 115200
    cfsetospeed(&SerialPortSettings,B9600); // Set Write Speed as 115200

    /* 8N1 Mode */
    SerialPortSettings.c_cflag &= ~PARENB;   // Disables the Parity Enable bit(PARENB),So No Parity
    SerialPortSettings.c_cflag &= ~CSTOPB;   // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit
    SerialPortSettings.c_cflag &= ~CSIZE;	 // Clears the mask for setting the data size
    SerialPortSettings.c_cflag |=  CS8;      // Set the data bits = 8

    SerialPortSettings.c_cflag &= ~CRTSCTS;       // No Hardware flow Control
    SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Enable receiver,Ignore Modem Control lines 


    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          // Disable XON/XOFF flow control both i/p and o/p
    SerialPortSettings.c_iflag &= ICANON;  // Non Cannonical mode

    if((tcsetattr(serialFD_,TCSANOW,&SerialPortSettings)) == 0) // Set the attributes to the termios structure
      result = true;
  }
  else
  {
     std::cout << "There is no open serial port..." << std::endl;
  }
    
  return result;
}

std::string serial::readSerial()
{
  tcflush(serialFD_, TCIFLUSH);   /* Discards old data in the rx buffer            */

  char read_buffer[1024] = {0};   /* Buffer to store the data received              */
  int  bytes_read = 0;    /* Number of bytes read by the read() system call */
  int pos = 0;

  while( pos < sizeof(read_buffer) ) {
    bytes_read = read(serialFD_, read_buffer+pos, 1);
    if (bytes_read == -1)
    {
        printf("Error reading from serial port\n");
        break;
    }
    else if (bytes_read == 0)
    {
        printf("No more data\n");
        break;
    }
    else
    {
        if( read_buffer[pos] == '\n' ) break;
    }
    pos++;
  }

  std::cout << "Received string: [" << read_buffer << "]" << std::endl;
  
  std::string result(read_buffer);
  return result;
}


