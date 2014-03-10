#include <SPI.h>
#ifndef READ_WRITE_H
#define READ_WRITE_H

char WriteReg(char addr, char value) //see page 22 of cc2500 data sheet for timing
{
  char x;
  digitalWrite(SS,LOW);
  delayMicroseconds(150);
  while (digitalRead(MISO) == HIGH)
  {
  };    
  x = SPI.transfer(addr);
  delayMicroseconds(1);
  SPI.transfer(value);
  delayMicroseconds(1);
  digitalWrite(SS,HIGH);
  return x;
}

char WriteReg_burst(char addr, char value[], byte count)
{
  char x;
  digitalWrite(SS,LOW);
  delayMicroseconds(150);  
  while (digitalRead(MISO) == HIGH) {
  };
  x = SPI.transfer(addr);
  delayMicroseconds(1);    
  for(byte i = 0; i<count; i++)
  {
    SPI.transfer(value[i]);
  }
  delayMicroseconds(1);
  digitalWrite(SS,HIGH);  
  return x;
}

char ReadReg(char addr){
  addr = addr + 0x80;
  delayMicroseconds(150);
  digitalWrite(SS,LOW);
  while (digitalRead(MISO) == HIGH) {
    };
  char x = SPI.transfer(addr);
  delayMicroseconds(1); 
  char y = SPI.transfer(0);
  delayMicroseconds(150);
  digitalWrite(SS,HIGH);
  return y;  
}

char SendStrobe(char strobe){
  char x;
  delayMicroseconds(150);
  digitalWrite(SS,LOW);  
  while (digitalRead(MISO) == HIGH) {
  };  
  x = SPI.transfer(strobe);
  delayMicroseconds(1); 
  digitalWrite(SS,HIGH);    
  return x;
}
#endif

