#include "cc2500_REG_V2.h"
#include "cc2500_VAL_V2.h"
#include "cc2500init_V2.h"
#include "read_write.h"

#include <SPI.h>
//These are all single byte burst address for write and read. ReadReg modifies the address for single and burst reads. WriteReg single and burst make similar modifications to the address. See page 60 of the datasheet

#define CC2500_RX      0x34      // Enable RX. Perform calibration if enabled
#define CC2500_TX      0x35      // Enable TX. If in RX state, only enable TX if CCA passes
#define CC2500_IDLE    0x36      // Exit RX / TX, turn
#define CC2500_FRX     0x3A      // Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states
#define CC2500_FTX     0x3B      // Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states
#define CC2500_SWOR    0x38
#define CC2500_TXFIFO  0x3F
#define CC2500_RXFIFO  0x3F

//#define CC2500_TXFIFO_BURST  0x7F
//#define CC2500_RXFIFO_BURST  0xFF

#define CC2500_SRES    0x30       // reset strobe 

#define myName         0x00
#define broadCast      0xFF
#define finish         1
const int GDO2_PIN = 2;

#define TX_TIMEOUT 10 // in milliseconds
long previousTXTimeoutMillis = 0;
long previousMillis = 0; 

long sendInterval = 800; // in milliseconds

char TP[] = {7, 0x05, 'H','E','L','L','O','!'}; //packet length (includes address and data), device adress 
void setup(){
  Serial.begin(9600);  
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);  
  // Setup 
  pinMode(SS,OUTPUT);
  SPI.begin();
  digitalWrite(SS,HIGH);
  Serial.println(15&ReadReg(0xF5),DEC); 
  Serial.println("Initializing Wireless..");
  SendStrobe(CC2500_SRES);
  init_CC2500_V2();  
  Read_Config_Regs();
  pinMode(9, OUTPUT);
  digitalWrite(9,HIGH);     
}
void loop()
{    
  sendPacket(8, TP); // 8 is the whole array    
  //delay(100);  
  //delay(1);
} 

void sendPacket(byte count, char TP[]){
  char state;
  //WriteReg(REG_IOCFG1,0x06);
  //Make sure that the radio is in IDLE state before flushing the FIFO  
  //Serial.println(15&ReadReg(0xF5),DEC); 
  //Flush TX FIFO  
  //SendStrobe(CC2500_RX);
  SendStrobe(CC2500_IDLE); 
  SendStrobe(CC2500_FTX);
  //SendStrobe(CC2500_IDLE); 
  Serial.println("flushed"); 
  //SendStrobe(CC2500_IDLE); //Do I need to go to idle?
  WriteTX_burst(CC2500_TXFIFO,TP,count);
  
  Serial.println("written");   
  SendStrobe(CC2500_TX); //do not add code between the strobe and while loops otherwise it will miss the conditions
  //ReadReg(REG_IOCFG1);
  //Serial.println("strobed"); 
  //previousTXTimeoutMillis = millis();
  while (!digitalRead(MISO)) {  
  }    
  //previousTXTimeoutMillis = millis();
  while (digitalRead(MISO)) {    
  }    
  Serial.println("Finished sending");
     SendStrobe(CC2500_IDLE); 
}

void listenForPacket() {
  SendStrobe(CC2500_RX);
  WriteReg(REG_IOCFG1,0x01);
  delay(20);
  unsigned long currentMillis = millis();
  while (digitalRead(MISO)) {      
    char PacketLength = ReadReg(CC2500_RXFIFO);
    char recvPacket[PacketLength];
    if(PacketLength == 7) {
      Serial.println("Packet Received!");
      Serial.print("Packet Length: ");
      Serial.println(PacketLength, DEC);
      Serial.print("Data: ");
      for(int i = 1; i < PacketLength; i++){
        recvPacket[i] = ReadReg(CC2500_RXFIFO);
        Serial.print(recvPacket[i], DEC);
        Serial.print(" ");
      }
      Serial.println(" ");
      byte rssi = ReadReg(CC2500_RXFIFO);
      byte lqi = ReadReg(CC2500_RXFIFO);
      Serial.print("RSSI: ");
      Serial.println(rssi);
      Serial.print("LQI: ");
      Serial.println(lqi);
    }
    
    // Make sure that the radio is in IDLE state before flushing the FIFO
    // (Unless RXOFF_MODE has been changed, the radio should be in IDLE state at this point) 
    SendStrobe(CC2500_IDLE);
    // Flush RX FIFO
    SendStrobe(CC2500_FRX);
  } 
}

void Read_Config_Regs(void)
{ 
  Serial.println(ReadReg(REG_IOCFG2),HEX);
  delayMicroseconds(1);
  Serial.println(ReadReg(REG_IOCFG1),HEX);
  delayMicroseconds(1);
  Serial.println(ReadReg(REG_IOCFG0),HEX);
}
