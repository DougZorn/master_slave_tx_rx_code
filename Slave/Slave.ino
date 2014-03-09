#include "cc2500_REG_V2.h"
#include "cc2500_VAL_V2.h"
#include "cc2500init_V2.h"
#include "read_write.h"

#include <SPI.h>

#define CC2500_IDLE    0x36      // Exit RX / TX, turn
#define CC2500_TX      0x35      // Enable TX. If in RX state, only enable TX if CCA passes
#define CC2500_RX      0x34      // Enable RX. Perform calibration if enabled
#define CC2500_FTX     0x3B      // Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states
#define CC2500_FRX     0x3A      // Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states
#define CC2500_SWOR    0x38
#define CC2500_TXFIFO  0x3F
#define CC2500_RXFIFO  0x3F

#define CC2500_TXFIFO_BURST  0x7F
#define CC2500_RXFIFO_BURST  0xFF

#define CC2500_SRES    0x30       // reset strobe 

#define myName         0x00
#define broadCast      0xFF
#define finish         1
const int GDO2_PIN = 2;


#define TX_TIMEOUT 10 // in milliseconds
long previousTXTimeoutMillis = 0;
long previousMillis = 0; 

long sendInterval = 800; // in milliseconds

char TP[] = {5, 0x05, 'H','E','L','L','O'};

void setup(){
  Serial.begin(9600);
  
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  
  // Setup 
  pinMode(SS,OUTPUT);
  SPI.begin();
  digitalWrite(SS,HIGH);
  Serial.println("Initializing Wireless..");
  SendStrobe(CC2500_SRES);
  //Serial.println(ReadReg(0x3A),HEX);  
  //Serial.println(ReadReg(0x3B),HEX);
  init_CC2500_V2();  
  Read_Config_Regs(); 
}

void loop(){  
  //unsigned long currentMillis = millis();
  //if(currentMillis - previousMillis > sendInterval) {
  //  previousMillis = currentMillis;   
  //sendPacket(7, TP);
  //Serial.println(ReadReg(0x3A),HEX);  
  //Serial.println(ReadReg(0x3B),HEX);
  Serial.println("In main loop waiting to listen");
  listenForPacket();   
}

void sendPacket(byte count, char TP[]){
  WriteReg(REG_IOCFG1,0x06);
  // Make sure that the radio is in IDLE state before flushing the FIFO
  SendStrobe(CC2500_IDLE);
  // Flush TX FIFO
  SendStrobe(CC2500_FTX); 
  SendStrobe(CC2500_IDLE);

  WriteReg_burst(CC2500_TXFIFO_BURST,TP,count);
  SendStrobe(CC2500_TX);
  Serial.println("test");
  
  previousTXTimeoutMillis = millis();
  while (!digitalRead(MISO)) {
  }  
  previousTXTimeoutMillis = millis();
  while (digitalRead(MISO)) {
  }  
  Serial.println("Finished sending");
  SendStrobe(CC2500_IDLE);
}

void listenForPacket() {
  WriteReg(REG_IOCFG2,0x07);
  SendStrobe(CC2500_RX);  
  delayMicroseconds(100);
  //unsigned long currentMillis = millis();  
  while (true){
    //Serial.println(ReadReg(0x3A),HEX);  
    //Serial.println(ReadReg(0x3B),HEX);
    if (digitalRead(GDO2_PIN)){
      Serial.println("Packet Received!");
      break;
    }
  }  
    
    // Make sure that the radio is in IDLE state before flushing the FIFO
    // (Unless RXOFF_MODE has been changed, the radio should be in IDLE state at this point) 
    SendStrobe(CC2500_IDLE);
    // Flush RX FIFO
    SendStrobe(CC2500_FRX);
}

char SendStrobe(char strobe){
  digitalWrite(SS,LOW);
  
  while (digitalRead(MISO) == HIGH) {
  };
    
  char result =  SPI.transfer(strobe);
  digitalWrite(SS,HIGH);
  delay(10);
  return result;
}

void Read_Config_Regs(void){ 
  Serial.println(ReadReg(REG_IOCFG2),HEX);
  delayMicroseconds(1);
  Serial.println(ReadReg(REG_IOCFG1),HEX);
  delayMicroseconds(1);
  Serial.println(ReadReg(REG_IOCFG0),HEX);
  delayMicroseconds(1);
  Serial.println(ReadReg(0x3E),HEX);
  delay(100);
}
