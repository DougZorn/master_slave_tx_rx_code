#include <SPI.h>
#ifndef READ_WRITE_H
#define READ_WRITE_H

void WriteReg(char addr, char value){
  digitalWrite(SS,LOW);
  
  while (digitalRead(MISO) == HIGH) {
    };
    
  SPI.transfer(addr);
  delayMicroseconds(200);
  SPI.transfer(value);
  digitalWrite(SS,HIGH);
}

void WriteReg_burst(char addr, char value[], byte count)
{
  digitalWrite(SS,LOW);  
  while (digitalRead(MISO) == HIGH) {
  };
  SPI.transfer(addr);  
  delay(1);
  for(byte i = 0; i<count; i++)
  {
    SPI.transfer(value[i]);
    delayMicroseconds(200);
  }
  digitalWrite(SS,HIGH);  
}

char ReadReg(char addr){
  addr = addr + 0x80;
  digitalWrite(SS,LOW);
  while (digitalRead(MISO) == HIGH) {
    };
  char x = SPI.transfer(addr);
  delay(10);
  char y = SPI.transfer(0);
  digitalWrite(SS,HIGH);
  return y;  
}
#endif
