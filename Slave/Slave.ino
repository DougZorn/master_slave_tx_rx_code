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
#define CC2500_RXFIFO  0xBF

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

char TP[] = {7, 0x05, 'H','E','L','L','O','!'}; //packet length(only includes data), device adress, data 

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
  
  //Serial.println("START LISTEN");
  WriteReg(REG_IOCFG2,0x06);  
  //Serial.println("IOCFG1 written");
  //SendStrobe(CC2500_IDLE); //36
  //Serial.println("IDLE top written");  
  //SendStrobe(CC2500_FRX); //3A   ERRATA NOTES
  //SendStrobe(CC2500_FRX);
  //Serial.println("FRX written");  
  SendStrobe(CC2500_RX);  //34


     
  //Serial.println("RX written");   
  //I get to this point and the GDO_2 never deassertes but it should
  ReadReg(0xFB);
  ReadReg(0xFB);
  ReadReg(0xFB);
  while(!digitalRead(MISO))   // with GDO_2 = 6 asserted when received and deasserted when end of packet. !!!!!!Wne should never empty (read REG the FIFO) the RX FIFO before the last byte of the packet is received!!!!!!!!!!!!!!!!!!!
  {
     
  }
  while(digitalRead(MISO)) 
  {
     
  }  
  
  Serial.println("PACKET Received");  
  Serial.println(ReadReg(0xFB),HEX);
  if(ReadReg(0xFB)==0)//number of bytes in RX FIFO 
  {
    Serial.println("CRC FAILED");     
  }
  else
  {
    Serial.println("CRC PASSED");
    for(int i = 0; i < 8; i++)
    {
      Serial.write(ReadReg(CC2500_RXFIFO)); 
      Serial.println(" ");
    }    
  }  
  SendStrobe(CC2500_IDLE);  
  //Serial.println("IDLE bottom");
  SendStrobe(CC2500_FRX);
  //Serial.println("END LISTEN");  
}

void Read_Config_Regs(void){ 
  Serial.println(ReadReg(REG_IOCFG2),HEX);
  delayMicroseconds(1);
  Serial.println(ReadReg(REG_IOCFG1),HEX);
  delayMicroseconds(1);
  Serial.println(ReadReg(REG_IOCFG0),HEX);
 
}

