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

char TP[] = {5, 0x05, 'H','E','L','L','O'}; //packet length(only includes data), device adress, data 

void setup()
{
  Serial.begin(9600);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  //SPI_MODE0
  //SPI_MODE1
  //SPI_MODE2
  //SPI_MODE3  
  SPI.setDataMode(SPI_MODE0);
  
  // Setup 
  pinMode(SS,OUTPUT);
  SPI.begin();
  digitalWrite(SS,HIGH);
  Serial.println("Initializing Wireless..");
  SendStrobe(CC2500_SRES);
  init_CC2500_V2();  
  Read_Config_Regs(); 
}

void loop()
{  
  //sendPacket(7, TP);
  listenForPacket();
  delay(500);
} 

void sendPacket(byte count, char TP[]){
  WriteReg(REG_IOCFG1,0x06);
  // Make sure that the radio is in IDLE state before flushing the FIFO
  SendStrobe(CC2500_IDLE);
  // Flush TX FIFO
  SendStrobe(CC2500_FTX); 
  SendStrobe(CC2500_IDLE);

  WriteReg_burst(CC2500_TXFIFO_BURST,TP,count);
  
  //Serial.println(ReadReg(0x3A),HEX);  
  //Serial.println(ReadReg(0x3B),HEX);
    
  SendStrobe(CC2500_TX);
  

  
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
//CC2500_IDLE   0x36 Exit RX / TX, turn
//CC2500_TX     0x35 Enable TX. If in RX state, only enable TX if CCA passes
//CC2500_RX     0x34 Enable RX. Perform calibration if enabled
//CC2500_FTX    0x3B Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states
//CC2500_FRX    0x3A Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states
//CC2500_SWOR   0x38
//CC2500_TXFIFO 0x3F
//CC2500_RXFIFO 0x3F

  
  
  Serial.println("START LISTEN");
  WriteReg(REG_IOCFG2,0x06);  
  Serial.println("IOCFG1 written");
  SendStrobe(CC2500_IDLE); //36
  Serial.println("IDLE top written");  
  SendStrobe(CC2500_FRX); //3A
  Serial.println("FRX written");  
  SendStrobe(CC2500_RX);  //34
  Serial.println("RX written");  
  char z;
  //while(digitalRead(2))
  //{
  //  Serial.println("entered loop");
  //  Serial.println(ReadReg(CC2500_RXFIFO),HEX);   
  //  //z = ReadReg(CC2500_RXFIFO);
  //} 
  
  while(!digitalRead(2))
  {
  }
  int PacketLength = ReadReg(CC2500_RXFIFO);
  unsigned char recvPacket[PacketLength];
  while(digitalRead(2)){
  }  
    
  Serial.println("GDO2 unasserted and loop exited");   
  
  for(int i = 0; i < PacketLength;i++)
  {
    recvPacket[ReadReg(CC2500_RXFIFO)];
  }
  
  for(int i = 0; i < PacketLength;i++)
  {
    Serial.println(recvPacket[i],HEX);
  }
  
  
  SendStrobe(CC2500_IDLE);  
  Serial.println("IDLE bottom");
  SendStrobe(CC2500_FRX); // this is causing the freeze?
  Serial.println("END LISTEN");
  //delay(20);   
}

void SendStrobe(char strobe){
  digitalWrite(SS,LOW);
  
  while (digitalRead(MISO) == HIGH) {
  };    
  SPI.transfer(strobe);
  digitalWrite(SS,HIGH);
  delay(10);  
}

void Read_Config_Regs(void){ 
  Serial.println(ReadReg(REG_IOCFG2),HEX);
  delayMicroseconds(1);
  Serial.println(ReadReg(REG_IOCFG1),HEX);
  delayMicroseconds(1);
  Serial.println(ReadReg(REG_IOCFG0),HEX);
  //delayMicroseconds(1);
  //Serial.println(ReadReg(0x3E),HEX);
  //delay(100);
}

