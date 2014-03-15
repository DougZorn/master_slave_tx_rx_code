#include <SPI.h>
#ifndef READ_WRITE_H
#define READ_WRITE_H

void WriteReg(char addr, char value) //see page 22 of cc2500 data sheet for timing
{
  digitalWrite(SS,LOW);
  delayMicroseconds(150);
  while (digitalRead(MISO) == HIGH)
  {
  };    
  SPI.transfer(addr);
  delayMicroseconds(1);
  SPI.transfer(value);
  delayMicroseconds(1);
  digitalWrite(SS,HIGH);
}

void WriteTX_burst(char addr, char value[], byte count)
{  
  addr = addr + 0x40;
  digitalWrite(SS,LOW);
  delayMicroseconds(150);  
  while (digitalRead(MISO) == HIGH) {
  };
  SPI.transfer(addr);
  delayMicroseconds(1);    
  for(byte i = 0; i<count; i++)
  {
    SPI.transfer(value[i]);
  }
  delayMicroseconds(1);
  digitalWrite(SS,HIGH);  
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

/*
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
*/

char ReadOnly_Reg(char addr){
  addr = addr + 0xC0;
  digitalWrite(SS,LOW);
  while (digitalRead(MISO) == HIGH) {
    };
  char x = SPI.transfer(addr);
  delayMicroseconds(1);
  char y = SPI.transfer(0);
  digitalWrite(SS,HIGH);
  return y;  
}


/*
char SendStrobe(char strobe){
  digitalWrite(SS,LOW);
  
  while (digitalRead(MISO) == HIGH) {
  };
    
  char result =  SPI.transfer(strobe);
  //while (!digitalRead(MISO)) {}; //This doesn't work because MISO asserts before the transfer is complete. Although you won't MISO to assert and deassert before you transfer something again in burst mode.
  //while (digitalRead(MISO)) {};
  digitalWrite(SS,HIGH);
  delay(10);
  return result;
}
*/

void SendStrobe(char strobe){
  delayMicroseconds(150);
  digitalWrite(SS,LOW);  
  while (digitalRead(MISO) == HIGH) {
  };  
  SPI.transfer(strobe);
  delayMicroseconds(1); 
  digitalWrite(SS,HIGH);    
}

#endif
