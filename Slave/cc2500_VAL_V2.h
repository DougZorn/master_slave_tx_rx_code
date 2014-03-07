 /*Sync word qualifier mode = 16/16 sync word bits detected 
 CRC autoflush = false 
 Channel spacing = 199.951172 
 Data format = Normal mode 
 Data rate = 2.39897 
 RX filter BW = 203.125000 
 Preamble count = 4 
 Whitening = true 
 Address config = Address check and 0 (0x00) broadcast 
 Carrier frequency = 2433.599762 
 Device address = 255 
 TX power = 1 
 Manchester enable = false 
 CRC enable = true 
 Deviation = 38.085938 
 Packet length mode = Variable packet length mode. Packet length configured by the first byte after sync word 
 Packet length = 255 
 Modulation format = 2-FSK 
 Base frequency = 2432.999908 
 Modulated = true 
 Channel number = 3 
 PA table
*/

 
#define PA_TABLE {0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,}
#define VAL_IOCFG2            0x0B    //GDO2Output Pin Configuration 
#define VAL_IOCFG1            0x06    //GDO1Output Pin Configuration 
#define VAL_IOCFG0            0x0C    //GDO0Output Pin Configuration 
#define VAL_FIFOTHR           0x07    //RX FIFO and TX FIFO Thresholds
#define VAL_SYNC1             0xD3    //Sync Word, High Byte 
#define VAL_SYNC0             0x91    //Sync Word, Low Byte 
#define VAL_PKTLEN            0xFF    //Packet Length 
#define VAL_PKTCTRL1          0xC6    //Packet Automation Control
#define VAL_PKTCTRL0          0x46    //Packet Automation Control
#define VAL_ADDR              0x255   //Device Address 
#define VAL_CHANNR            0x03    //Channel Number 
#define VAL_FSCTRL1           0x08    //Frequency Synthesizer Control 
#define VAL_FSCTRL0           0x00    //Frequency Synthesizer Control 
#define VAL_FREQ2             0x5D    //Frequency Control Word, High Byte 
#define VAL_FREQ1             0x93    //Frequency Control Word, Middle Byte 
#define VAL_FREQ0             0xB1    //Frequency Control Word, Low Byte 
#define VAL_MDMCFG4           0x86    //Modem Configuration 
#define VAL_MDMCFG3           0x83    //Modem Configuration 
#define VAL_MDMCFG2           0x02    //Modem Configuration
#define VAL_MDMCFG1           0x22    //Modem Configuration
#define VAL_MDMCFG0           0xF8    //Modem Configuration 
#define VAL_DEVIATN           0x44    //Modem Deviation Setting 
#define VAL_MCSM2             0x07    //Main Radio Control State Machine Configuration 
#define VAL_MCSM1             0x30    //Main Radio Control State Machine Configuration
#define VAL_MCSM0             0x18    //Main Radio Control State Machine Configuration 
#define VAL_FOCCFG            0x16    //Frequency Offset Compensation Configuration
#define VAL_BSCFG             0x6C    //Bit Synchronization Configuration
#define VAL_AGCCTRL2          0x03    //AGC Control
#define VAL_AGCCTRL1          0x40    //AGC Control
#define VAL_AGCCTRL0          0x91    //AGC Control
#define VAL_WOREVT1           0x87    //High Byte Event0 Timeout 
#define VAL_WOREVT0           0x6B    //Low Byte Event0 Timeout 
#define VAL_WORCTRL           0xF8    //Wake On Radio Control
#define VAL_FREND1            0x56    //Front End RX Configuration 
#define VAL_FREND0            0x10    //Front End TX configuration 
#define VAL_FSCAL3            0xA9    //Frequency Synthesizer Calibration 
#define VAL_FSCAL2            0x0A    //Frequency Synthesizer Calibration 
#define VAL_FSCAL1            0x00    //Frequency Synthesizer Calibration 
#define VAL_FSCAL0            0x11    //Frequency Synthesizer Calibration 
#define VAL_RCCTRL1           0x41    //RC Oscillator Configuration 
#define VAL_RCCTRL0           0x00    //RC Oscillator Configuration 
#define VAL_FSTEST            0x59    //Frequency Synthesizer Calibration Control 
#define VAL_PTEST             0x7F    //Production Test 
#define VAL_AGCTEST           0x3F    //AGC Test 
#define VAL_TEST2             0x81    //Various Test Settings 
#define VAL_TEST1             0x35    //Various Test Settings 
#define VAL_TEST0             0x0B    //Various Test Settings 
#define VAL_PARTNUM           0x80    //Chip ID 
#define VAL_VERSION           0x03    //Chip ID 
#define VAL_FREQEST           0x00    //Frequency Offset Estimate from Demodulator 
#define VAL_LQI               0x00    //Demodulator Estimate for Link Quality 
#define VAL_RSSI              0x00    //Received Signal Strength Indication 
#define VAL_MARCSTATE         0x00    //Main Radio Control State Machine State 
#define VAL_WORTIME1          0x00    //High Byte of WOR Time 
#define VAL_WORTIME0          0x00    //Low Byte of WOR Time 
#define VAL_PKTSTATUS         0x00    //Current GDOxStatus and Packet Status 
#define VAL_VCO_VC_DAC        0x00    //Current Setting from PLL Calibration Module 
#define VAL_TXBYTES           0x00    //Underflow and Number of Bytes 
#define VAL_RXBYTES           0x00    //Underflow and Number of Bytes 
#define VAL_RCCTRL1_STATUS    0x00    //Last RC Oscillator Calibration Result 
#define VAL_RCCTRL0_STATUS    0x00    //Last RC Oscillator Calibration Result 
