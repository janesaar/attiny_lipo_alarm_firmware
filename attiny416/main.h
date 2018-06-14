#ifndef MAIN_H_
#define MAIN_H_

uint16_t buzzerCntr = 0;

uint8_t transferedBytesRxD = 0;
uint16_t voltageLimit = 0;
char sendVoltageLimit[7] = "v:0000\n";

uint16_t cell1;
uint16_t cell2;
uint16_t cell3;
uint16_t cell4;

void UARTTransmitByte(char data);
void UARTTransmit(char* dataOut);
char* voltageToString(uint16_t voltage);
void addToString(char *dest, char *src, uint8_t destIndex, uint8_t size);
uint16_t measureCell(ADC_MUXPOS_t pin);
void measureCells();
void checkVoltageLimit();
void writeVoltageToEEPROM();
void readVoltageFromEEPROM();

void RTCInit();
void UARTInit();
void ADCInit();
void bzrTimerInit();

#endif /* MAIN_H_ */