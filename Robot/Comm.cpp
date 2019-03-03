#include "Comm.h"
#include <util/crc16.h>

void Comm::begin(long baud_rate) {
  Serial2.begin(baud_rate);
  failures = 0;
  resetStart = 0;
  bufferIndex = 0;
}

bool Comm::read(){
  //Serial.println(Serial2.available());
  if (Serial2.available() < READ_LEN-bufferIndex){
    failures++;
    return false;
  }
  
  if(Serial2.readBytes(read_buf + bufferIndex, READ_LEN-bufferIndex) < READ_LEN-bufferIndex){
    bufferIndex = 0;
    failures++;
    return false;
  }
  bufferIndex = 0;
  
  // if the start byte is not first we have a problem
  while(read_buf[0] != 0xdd || read_buf[READ_LEN-1] != _crc8(&read_buf[1], READ_LEN-2)){
    // attempt to recover
    int i;
    for(i=1; i<READ_LEN; i++)
      if(read_buf[i] == 0xdd)
        break;
    if(i >= READ_LEN){
      // recovery failed, attempt another read in case we have more bytes in the buffer
      return Comm::read();
    }else{
      // found possible start byte, attempt to read rest of message
      for(int j=i; j<READ_LEN; j++){
        read_buf[j-i] = read_buf[j];
      }
      if(Serial2.available() >= i){
        // rest of message available
        if(Serial2.readBytes(&read_buf[READ_LEN-i], i) < i){
          //Serial.println("didn't read rest of READ_LEN");
          failures++;
          return false;
        }
      }else{
        // wait for rest of message
        // we have to check next function call because this function cannot be blocking
        bufferIndex = READ_LEN - i;
        //Serial.println("wait for rest of message");
        failures++;
        return false;
      }
    }
  }

  _out_struct->driveFL   = read_buf[1];     //TODO: potential race condition. No synchronization primitives
  _out_struct->driveBL   = read_buf[2];
  _out_struct->driveFR   = read_buf[3];
  _out_struct->driveBR   = read_buf[4];
  _out_struct->waist     = read_buf[5];
  _out_struct->shoulder  = read_buf[6];
  _out_struct->elbow     = read_buf[7];
  _out_struct->wrist     = read_buf[8];
  _out_struct->vacuum    = read_buf[9];
  
  failures = 0;
  return true;
}

void Comm::write(){
  setOutBuf();
  if(failures == 0){
    while(Serial2.available() > 0) {
      char t = Serial2.read();
    }
  }
  Serial2.write(outBuf, 8);
}

void Comm::setOutBuf(){
  outBuf[0] = 0xdd;
  uint16_t *tmp = (uint16_t *)(outBuf + 1);
  *tmp = _in_struct->waist;
  *(tmp+1) = _in_struct->shoulder;
  *(tmp+2) = _in_struct->elbow;
  outBuf[7] = _crc8(&outBuf[1], 6);
}

Comm::~Comm() {
}

uint8_t Comm::_crc8(uint8_t *data, int len) {
  uint8_t crc = 0x00;
  for (int i = 0; i < len; i++) {
    crc = _crc8_ccitt_update(crc, data[i]);
  }
  return crc;
}

