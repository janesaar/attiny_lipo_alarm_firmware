#include <avr/io.h>
#include <avr/interrupt.h>

#define		F_CPU	2000000UL
#include <util/delay.h>

// TODO: PWM buzzer (alarmOn), sleep, SPI???

volatile uint8_t rxdReady = 0;
//volatile uint16_t voltageLimit;
uint8_t transferedBytesRxD = 0;

char setVoltageLimit[7] = "v:0000\n";

uint16_t cell1;
uint16_t cell2;
uint16_t cell3;
uint16_t cell4;

ISR(USART0_RXC_vect) {	
	// Read incoming data
	setVoltageLimit[transferedBytesRxD] = USART0_RXDATAL;
	
	// Check if read data matches the template: "v:0000\n"
	switch (transferedBytesRxD) {
		case 0:
			if (setVoltageLimit[0] != 'v') {
				transferedBytesRxD = 0;
			} else {
				transferedBytesRxD ++;
			} break;
		case 1:
			if (setVoltageLimit[1] != ':') {
				transferedBytesRxD = 0;
			} else {
				transferedBytesRxD ++;
			} break;
		case 2: case 3: case 4: case 5:
			if (setVoltageLimit[transferedBytesRxD] == '\n' || setVoltageLimit[transferedBytesRxD] < '0' || setVoltageLimit[transferedBytesRxD] > '9') {
				transferedBytesRxD = 0;
			} else {
				transferedBytesRxD ++;
			} break;	
		case 6:
			transferedBytesRxD = 0;
				rxdReady = 1;
			} break;
	}
}

void UARTInit() {
	// Set PINA1 (TxD) as output and high, PINA2 (RxD) as input and high.
	PORTA_DIR = PIN1_bm | (0 << PIN2_bp);
	PORTA_OUT = PIN1_bm | PIN2_bm;
	
	// Write bit to '1' to select alternative communication pins for USART0.
	// Default: PB2 - TxD, PB3 - RxD; Alternative: PA1 - TxD, PA2 - RxD.
	PORTMUX_CTRLB = PORTMUX_USART0_bm;
	
	uint32_t baudRate = 57600;
	USART0_BAUD = ((F_CPU * 64) / (baudRate * 16));
	// Receive Complete Interrupt Enable
	USART0_CTRLA = USART_RXCIE_bm;
	// Receiver Enable | Transmitter Enable
	USART0_CTRLB = USART_TXEN_bm | USART_RXEN_bm;
	// Character Size 8-bit
	USART0_CTRLC = USART_CHSIZE_8BIT_gc;
}

void UARTTransmitByte(char dataOut) {	
	// While Data Register Empty Flag not set, do not send
	while (!(USART0_STATUS & USART_DREIF_bm)) {
	}	
	// Transmit Data Register Low Byte
	USART0_TXDATAL = dataOut;
}

void UARTTransmit(char* dataOut) {
	char* dataOutPtr = dataOut;
	
	// While not zero termination, transmit byte
	while (*dataOutPtr != 0) {
		UARTTransmitByte(*dataOutPtr);
		dataOutPtr ++;
	}
}

void ADCInit() {
	ADC0_CTRLA = ADC_ENABLE_bm;
	// Reference Voltage 4.34V
	VREF_CTRLA = VREF_ADC0REFSEL_4V34_gc;
}

char* voltageToString(uint16_t voltage) {
	static char buf[5];  //???????????????????
	
	// Convert integer to character
	for (int i = 3; i >= 0; i--) {
		buf[i] = (char)(voltage % 10) + '0';
		voltage /= 10;
	}
	return buf; //????????????????????
}

void concatenateString(char *dest, char *src, uint8_t destIndex) {
	// Add cell voltage level to string allCells
	for (uint8_t i = 0; i < 4; i++) {
		dest[destIndex+i] = src[i];
	}
}

void measureCells() {	
	uint16_t vRef = 4340; // 4.34 V	
	
	// Set AIN4 as ADC - cell 1
	ADC0_MUXPOS = ADC_MUXPOS_AIN4_gc; // PA4	
	// Start ADC conversion STCONV
	ADC0_COMMAND = 1;
	uint32_t adcRead = ADC0_RES;	
	cell1 = (adcRead * vRef) / 1024;

	// Set AIN5 as ADC - cell 2
	ADC0_MUXPOS = ADC_MUXPOS_AIN5_gc; // PA5
	ADC0_COMMAND = 1;
	adcRead = ADC0_RES;		
	cell2 = (adcRead * vRef) / 1024;
	
	// Set AIN6 as ADC - cell 3
	ADC0_MUXPOS = ADC_MUXPOS_AIN6_gc; // PA6
	ADC0_COMMAND = 1;
	adcRead = ADC0_RES;
	cell3 = (adcRead * vRef) / 1024;
	
	// Set AIN7 as ADC - cell 4
	ADC0_MUXPOS = ADC_MUXPOS_AIN7_gc; // PA7
	ADC0_COMMAND = 1;
	adcRead = ADC0_RES;	
	cell4 = (adcRead * vRef) / 1024;	
	
	char allCells[43] = "{c1: xxxx, c2: xxxx, c3: xxxx, c4: xxxx}\n";
	// Send voltage in millivolts
	concatenateString(allCells, voltageToString(cell1), 5);
	concatenateString(allCells, voltageToString(cell2), 15);
	concatenateString(allCells, voltageToString(cell3), 25);
	concatenateString(allCells, voltageToString(cell4), 35);
	
	UARTTransmit(allCells);
}


void alarmOn() {
	// PORTB_OUTTGL = PIN5_bm;
	// PWM Buzzer
}

void sleepMode() {
	// RTC - real time counter interrupt
	
	// Set Sleep Mode, Sleep Mode Enable
	SLPCTRL_CTRLA = SLEEP_MODE_STANDBY | SLPCTRL_SEN_bm;
}

void readVoltageFromEEPROM() {
	cli();
	for (int i = 0; i < 4; i++) {
		setVoltageLimit[i + 2] = *(char*)(EEPROM_START + i);
	}
	sei();
}

void writeVoltageToEEPROM() {
	cli();
	while (NVMCTRL_STATUS & NVMCTRL_EEBUSY_bm);

	// Only needed to write/erase	
	CPU_CCP = CCP_SPM_gc;  // Key SPM - SPM Instruction Protection
	NVMCTRL_CTRLA = NVMCTRL_CMD_PAGEBUFCLR_gc;
	
	// Write voltage limit to EEPROM
	// Start address of EEPROM - EEPROM_START - (0x1400-0x1480)				
	for (int i = 0; i < 4; i++) {
		*(char*)(EEPROM_START + i) = setVoltageLimit[i + 2];
	}
	
	CPU_CCP = CCP_SPM_gc;
	NVMCTRL_CTRLA = NVMCTRL_CMD_PAGEERASEWRITE_gc;
	
	sei();
}

uint16_t getVoltageLimit() {
	return (setVoltageLimit[2]-'0')*1000 + (setVoltageLimit[3]-'0')*100 + (setVoltageLimit[4]-'0')*10 + (setVoltageLimit[5]-'0');	
}

void checkVoltageLimit() {
	writeVoltageToEEPROM();
	uint16_t voltageLimit = getVoltageLimit();
	while (cell1 < voltageLimit || cell2 < voltageLimit || cell3 < voltageLimit || cell4 < voltageLimit) {
		voltageLimit = getVoltageLimit();
		//alarmOn();
		
		//UARTTransmit(setVoltageLimit);
		UARTTransmit("BATTERY LOW\n");
		
		PORTB_OUTTGL = PIN5_bm;
		_delay_ms(100);
	}
	PORTB_OUTTGL = (0 << PIN5_bp);
}


int main(void) {
	// Divide 20MHz Clock by 10 -> 2MHz Clock
	CPU_CCP = CCP_IOREG_gc;
	CLKCTRL_MCLKCTRLB = CLKCTRL_PEN_bm | CLKCTRL_PDIV_10X_gc;
	
	//EEPROM not erased under chip erase
	//If the device is locked the EEPROM is always erased by a chip erase, regardless of this bit.
	FUSE_SYSCFG0 = FUSE_EESAVE_bm;
	
	UARTInit();
	ADCInit();
	
	// Toggle LED when connected
	PORTB_DIR = (1 << PIN5_bp);
	for (uint8_t i = 5; i > 0; i--) {
		PORTB_OUTTGL = PIN5_bm;
		_delay_ms(100);
	}
	
	readVoltageFromEEPROM();
	
	sei();
	
    while (1) {	
		if (rxdReady == 1) {
			//writeVoltageToEEPROM();
			rxdReady = 0;
		}
		_delay_ms(1500);
		
		measureCells();
		checkVoltageLimit();
		UARTTransmit(setVoltageLimit);	
		
		
		// start ADC conversion STCONV
		/*ADC0_COMMAND = 1;
		adcResult = ADC0_RES;	
		
		PORTB_OUTTGL = (1 << 5);
		for (uint16_t i = 0; i < adcResult && i < 1000; i++) {
			_delay_ms(1);
		}*/
    }
}