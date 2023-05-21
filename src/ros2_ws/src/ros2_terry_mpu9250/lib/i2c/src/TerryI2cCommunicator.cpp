#include "TerryI2cCommunicator.h"

extern "C" {
#include <errno.h>
#include <fcntl.h>
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
#include <string.h>
#include <unistd.h>
}

#include <iostream>
#include <string>

TerryI2cCommunicator::TerryI2cCommunicator(int bus_number)
{
  const std::string filename_ = "/dev/i2c-" + std::to_string(bus_number);
  std::cout << filename_ << std::endl;
  file_ = open(filename_.c_str(), O_RDWR);
  if (file_ < 0) {
    std::cerr << "Failed to open file descriptor! Check your bus number! Errno: "
              << strerror(errno);
    exit(1);
  }
}

TerryI2cCommunicator::~TerryI2cCommunicator() { close(file_); }

int TerryI2cCommunicator::read(unsigned char address)
{
  int result = i2c_smbus_read_byte_data(file_, address);
  if (result < 0) reportError(errno);
  return result;
}

int TerryI2cCommunicator::write(unsigned char address, unsigned char value)
{
  int result = i2c_smbus_write_byte_data(file_, address, value);
  if (result < 0) reportError(errno);
  return result;
}

int TerryI2cCommunicator::read_block(unsigned char address, char length, unsigned char *buffer)
{
  int readLength = i2c_smbus_read_i2c_block_data(file_, address, length, buffer);
  if (readLength < 0) reportError(errno);
  return readLength;
}

char TerryI2cCommunicator::getFile() { return file_; }

void TerryI2cCommunicator::reportError(int error)
{
  std::cerr << "Error! Errno: " << strerror(error) << std::endl;
}
