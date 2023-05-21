#ifndef __TERRYI2CCOMMUNICATOR_H__
#define __TERRYI2CCOMMUNICATOR_H__

#include "I2cCommunicator.h"

class TerryI2cCommunicator final : public I2cCommunicator {
 public:
  TerryI2cCommunicator(int bus_number = 1);
  ~TerryI2cCommunicator();
  int read(unsigned char address) final;
  int write(unsigned char address, unsigned char value) final;
  int read_block(unsigned char address, char length, unsigned char *buffer) final;
  char getFile() final;

 private:
  void reportError(int error);
  int file_;
};

#endif  // __TERRYI2CCOMMUNICATOR_H__