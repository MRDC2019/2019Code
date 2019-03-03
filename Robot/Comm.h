#ifndef COMM_H
#define COMM_H

#include <stdint.h>
#include <Arduino.h>
#include "RobotIO.h"
#include "Config.h"

class Comm {
private:
  RobotIn *_in_struct;
  RobotOut *_out_struct;
  unsigned char read_buf[128];
  unsigned char outBuf[8];
  int bufferIndex;
  long failures;
  long resetStart;
  
public:
  Comm(RobotIn *in_struct, RobotOut *out_struct) : _in_struct(in_struct), 
                                                   _out_struct(out_struct){}
  // time out is measured in frames
  void begin(long baud_rate);

  /**
   * Basically, write out to PC through serial,
   * and read from serial, update the internal servo values.
   */
  void write();
  bool read();
  
  void setOutBuf();
  long getFailures() { return failures; }
  ~Comm();

private:
  void _set_timer(uint32_t rate);
  void _rm_timer();
  uint8_t _crc8(uint8_t *data, int len);
};

#endif /* COMM_H */
