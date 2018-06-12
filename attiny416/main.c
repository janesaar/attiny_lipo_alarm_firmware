#include <avr/io.h>
#include <C:/Users/Jane/Documents/Atmel Studio/7.0/attiny416/attiny416/main.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

//#define		F_CPU	2000000UL //10x
//#define		F_CPU	10000000UL //2x
//#define			F_CPU	5000000UL //4x
#define			F_CPU	2500000UL //8x
//#define			F_CPU	1250000UL //16x

#include <util/delay.h>


// TODO: PWM buzzer (alarmOn), SPI???

ISR(RTC_CNT_vect) {
	//addToString(sendVoltageLimit, voltageToString(voltageLimit), 2, 4);
	UARTTransmit(sendVoltageLimit);
	measureCells();
	checkVoltageLimit();
	
	// Overflow interrupt flag has to be cleared manually
	RTC_INTFLAGS = (1 << RTC_OVF_bp);
}


ISR(USART0_RXC_vect) {	
	// Read incoming data
	char dataIn[7];
	dataIn[transferedBytesRxD] = USART0_RXDATAL;
	
	// Check if read data matches the template: "v:0000\n"
	if ((transferedBytesRxD == 0 && dataIn[0] == 'v') || 
		(transferedBytesRxD == 1 && dataIn[1] == ':') || 
		(2 <= transferedBytesRxD && transferedBytesRxD <= 5 && 
		'0' <= dataIn[transferedBytesRxD] && dataIn[transferedBytesRxD] <= '9')) {
		transferedBytesRxD ++;
	} else if (transferedBytesRxD == 6 && dataIn[6] == '\n') {
		voltageLimit = (dataIn[2]-'0')*1000 + (dataIn[3]-'0')*100 + (dataIn[4]-'0')*10 + (dataIn[5]-'0');
		
		writeVoltageToEEPROM();
		transferedBytesRxD = 0;
		
		addToString(sendVoltageLimit, voltageToString(voltageLimit), 2, 4);
		UARTTransmit(sendVoltageLimit);
	} else {
		transferedBytesRxD = 0;
	}
}


void UARTInit() {
	// Set PINA1 (TxD) as output and high, PINA2 (RxD) as input and high.
	PORTA_DIR = (1 << PIN1_bp) | (0 << PIN2_bp);
	PORTA_OUT = (1 << PIN1_bp) | (1 << PIN2_bp);
	
	// Write bit to '1' to select alternative communication pins for USART0.
	// Default: PB2 - TxD, PB3 - RxD; Alternative: PA1 - TxD, PA2 - RxD.
	PORTMUX_CTRLB = (1 << PORTMUX_USART0_bp);
	
	uint32_t baudRate = 9600;
	USART0_BAUD = ((F_CPU * 64) / (baudRate * 16));
	// Receive Complete Interrupt Enable
	USART0_CTRLA = (1 << USART_RXCIE_bp);
	// Receiver Enable | Transmitter Enable | Start Frame Detection Enable (for waking up from Standby)
	USART0_CTRLB = (1 << USART_TXEN_bp) | (1 << USART_RXEN_bp) | (1 << USART_SFDEN_bp);
	// Character Size 8-bit
	USART0_CTRLC = USART_CHSIZE_8BIT_gc;
}


void UARTTransmitByte(char data) {
	char dataOut = data;	
	// While Data Register Empty Flag not set, do not send
	while (!(USART0_STATUS & (1 << USART_DREIF_bp))) {
	}	
	// Transmit Data Register Low Byte
	USART0_TXDATAL = dataOut;
}


void UARTTransmit(char* dataOut) {
	char *dataOutPtr = dataOut;
	
	// While not zero termination, transmit byte
	while (*dataOutPtr != 0) {
		UARTTransmitByte(*dataOutPtr);
		dataOutPtr ++;
	} 
}


void ADCInit() {
	ADC0_CTRLA = (1 << ADC_ENABLE_bp);
	// Reference Voltage 4.34V
	VREF_CTRLA = VREF_ADC0REFSEL_4V34_gc;
}


char* voltageToString(uint16_t voltage) {
	static char buf[5];
	
	// Convert integer to character
	for (int i = 3; i >= 0; i--) {
		buf[i] = (char)(voltage % 10) + '0';
		voltage /= 10;
	}
	return buf;
}


void addToString(char *dest, char *src, uint8_t destIndex, uint8_t size) {
	// Add cell voltage level to string allCells
	for (uint8_t i = 0; i < size; i++) {
		if (dest[destIndex+i] != 0) {
			dest[destIndex+i] = src[i];
		} else {
			break;
		}
	}
}


uint16_t measureCell(ADC_MUXPOS_t pin) {
	ADC0_MUXPOS = pin; // PA4, PA5, PA6, PA7
	// Start ADC conversion STCONV
	ADC0_COMMAND = 1;
	
	uint32_t adcRead = ADC0_RES;
	uint16_t vRef = 4340;
			
	return (adcRead * vRef) / 1024;	
}


void measureCells() {	
	char allCells[42] = "{c1: xxxx, c2: xxxx, c3: xxxx, c4: xxxx}\n";
	// Send voltage in millivolts
	addToString(allCells, voltageToString(cell1 = measureCell(ADC_MUXPOS_AIN4_gc)), 5, 4);
	addToString(allCells, voltageToString(cell2 = measureCell(ADC_MUXPOS_AIN5_gc)), 15, 4);
	addToString(allCells, voltageToString(cell3 = measureCell(ADC_MUXPOS_AIN6_gc)), 25, 4);
	addToString(allCells, voltageToString(cell4 = measureCell(ADC_MUXPOS_AIN7_gc)), 35, 4);
	
	UARTTransmit(allCells);
}

void checkVoltageLimit() {	
	if (cell1 < voltageLimit || cell2 < voltageLimit || cell3 < voltageLimit || cell4 < voltageLimit) {
		//alarmOn();
		
		//UARTTransmit(setVoltageLimit);
		//addToString(sendVoltageLimit, voltageToString(voltageLimit), 2, 4);
		//UARTTransmit(sendVoltageLimit);
		//UARTTransmit("BATTERY LOW\n");
		
		PORTB_OUTTGL = (1 << PIN5_bp);
	} else {
		PORTB_OUT = (1 << PIN5_bp);	
	}
}


void writeVoltageToEEPROM() {
	cli();
	while (NVMCTRL_STATUS & (1 << NVMCTRL_EEBUSY_bp)) {	
	}

	// Only needed to write/erase
	CPU_CCP = CCP_SPM_gc;  // Key SPM - SPM Instruction Protection
	NVMCTRL_CTRLA = NVMCTRL_CMD_PAGEBUFCLR_gc;
	
	// Write voltage limit to EEPROM
	// Start address of EEPROM - EEPROM_START - (0x1400-0x1480)
	*(uint8_t*)(EEPROM_START) = (uint8_t)(voltageLimit >> 8); // Get the higher byte of voltageLimit
	*(uint8_t*)(EEPROM_START + 1) = (uint8_t)voltageLimit; // Get the lower byte of voltageLimit
	
	// Enable writing to EEPROM
	CPU_CCP = CCP_SPM_gc;
	NVMCTRL_CTRLA = NVMCTRL_CMD_PAGEERASEWRITE_gc;
	
	sei();
}


void readVoltageFromEEPROM() {
	cli();
	
	// Higher byte / lower byte
	// Start address of EEPROM - EEPROM_START - (0x1400-0x1480)
	voltageLimit = ((uint16_t)(*(uint8_t*)(EEPROM_START)) << 8) | *(uint8_t*)(EEPROM_START + 1);

	sei();
}


void RTCInit() {
	CPU_CCP = CCP_IOREG_gc;
	CLKCTRL_OSC32KCTRLA = (1 << CLKCTRL_RUNSTDBY_bp);
	
	while (RTC_STATUS > 0) { // Wait for all register to be synchronised
	}	
	
	// 32 prescaler (1000 counts per second), RTC enable, allow RTC standby
	RTC_CTRLA = RTC_PRESCALER_DIV32_gc | (1 << RTC_RTCEN_bp) | (1 << RTC_RUNSTDBY_bp);
	//RTC_PER = 0x200; //512
	RTC_PER = 0x400; //1024	
	// Compare match interrupt disable, overflow enable
	RTC_INTCTRL = (0 << RTC_CMP_bp) | (1 << RTC_OVF_bp);	
}


void alarmOn() {
	// PORTB_OUTTGL = PIN5_bm;
	// PWM Buzzer // 2400Hz
}


int main(void) {
	// Divide 20MHz Clock by 10 -> 2MHz Clock
	CPU_CCP = CCP_IOREG_gc;
	//CLKCTRL_MCLKCTRLB = CLKCTRL_PEN_bm | CLKCTRL_PDIV_10X_gc;
	CLKCTRL_MCLKCTRLB = (1 << CLKCTRL_PEN_bp) | CLKCTRL_PDIV_8X_gc;
	
	//EEPROM not erased under chip erase
	//If the device is locked the EEPROM is always erased by a chip erase, regardless of this bit.
	FUSE_SYSCFG0 |= (1 << FUSE_EESAVE_bp);
	
	UARTInit();
	ADCInit();
	RTCInit();
	
	// Toggle LED when connected
	PORTB_DIR = (1 << PIN5_bp);
	for (uint8_t i = 5; i > 0; i--) {
		PORTB_OUTTGL = (1 << PIN5_bp);
		_delay_ms(100);
	}
	
	if (*(uint8_t*)(EEPROM_START) != 255 && *(uint8_t*)(EEPROM_START+1) != 255) {
		readVoltageFromEEPROM();	
		addToString(sendVoltageLimit, voltageToString(voltageLimit), 2, 4);	
	}	
	
	sei();
	
	set_sleep_mode(SLEEP_MODE_STANDBY);
	sleep_enable();	
	
    while (1) {	
		sleep_cpu();

		_delay_ms(2);
		/*
		if (goToSleep == 1) {
			sleep_enable();
			sleep_cpu();
		} else {
			sleep_disable()
		}*/
	
    }
}