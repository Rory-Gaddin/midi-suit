#include <stdio.h>
#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <avr/eeprom.h>

#include <util/delay.h>
#include "uart.h"
#include <avr/interrupt.h>
#include <stdbool.h>
#include <avr/wdt.h>

#define UART_BAUD_RATE      57600
#define F_CPU 16000000

//TLV
// Port B
#define MOSI	5
#define MISO	6
#define SCK		7
#define CS1		4	// CS for TLV #1
#define CS2		3	// CS for TLV #2

#define DEBUG	// Debugging UART output


struct jointMapping {
	char magic[2];
	char device[8];
	char function[8];
	uint16_t lowIn;		// ADC reading - low end
	uint16_t highIn;		// ADC reading - high end
	uint8_t lowOut;		// Mapped value - low end
	uint8_t highOut;		// Mapped value - high end
	uint8_t cc;			// CC 
	uint8_t channel;		// MIDI Channel to output on
}; //	Struct size of 24 bytes

uint16_t adcBuffer[22];
uint16_t adcLast[22];
const uint8_t deadband = 4;

void spi_mode(unsigned char config) {
	// enable SPI master with configuration byte specified
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1) | (1 << SPR0);
	SPSR = 0x00;
}

unsigned char spi_transfer(unsigned char value) {
	unsigned char x;
	SPDR = value;
	while (!(SPSR & (1<<SPIF))) {}
	x  = SPDR;
	return x;
}

uint16_t readADC() {
	// Chan 0-10
	uint8_t idx = 0;
	uint8_t chan;
	uint16_t ad;
      uint8_t ad_l;
	PORTB &= ~(1 << CS1);
	_delay_us(10);
	for (chan = 0; chan < 9; chan++) {
		ad = spi_transfer((chan << 4) | 0x0C);  // 0x0C = 16-Bit, MSB-First, Unipolar  
		// ad = spi_transfer(0xBC); // Test; should be 0x800, 1/2 of vRef
		ad_l = spi_transfer(0x00);
		ad <<= 8;  
		ad |= ad_l;  
		ad >>= 4;
		_delay_us(10);
		if (chan > 0) {
			adcLast[idx] = adcBuffer[idx];
			adcBuffer[idx] = ad;
			idx++;
		}
	}
	PORTB |= (1 << CS1);
	PORTB &= ~(1 << CS2);
	_delay_us(10);
	for (chan = 0; chan < 9; chan++) {
		ad = spi_transfer((chan << 4) | 0x0C);  // 0x0C = 16-Bit, MSB-First, Unipolar  
		// ad = spi_transfer(0xBC); // Test; should be 0x800, 1/2 of vRef
		ad_l = spi_transfer(0x00);
		ad <<= 8;  
		ad |= ad_l;  
		ad >>= 4;
		_delay_us(10);
		if (chan > 0) {
			adcLast[idx] = adcBuffer[idx];
			adcBuffer[idx] = ad;
			idx++;
		}
	}
	PORTB |= (1 << CS2);
}


struct jointMapping readMapping(uint8_t mapId) {
	// Read a mapping from EEPROM and return a struct
	struct jointMapping foo;
	eeprom_read_block(&foo, (mapId * sizeof(foo)), sizeof(foo));
	uart_puts("Reading mapping..\r\n");
	return foo;
}

void writeMapping(uint8_t mapId, struct jointMapping map) {
	// Write mapping to EEPROM
	strncpy(&map.magic, "##", 2);
	eeprom_write_block(&map, (mapId * sizeof(map)), sizeof(map));
}


void dumpMapping(struct jointMapping map) {
	char buf[16];
	if (map.magic[0] == '#' & map.magic[1] == '#') {
			uart_puts("a.Device: ");
			uart_puts(map.device);
			uart_puts("\r\n");
			uart_puts("b.function: ");
			uart_puts(map.function);
			uart_puts("\r\n");
			uart_puts("c.Low in: ");
			itoa(map.lowIn, &buf, 10);
			uart_puts(buf);
			uart_puts("\r\n");
			uart_puts("d.High in: ");
			itoa(map.highIn, &buf, 10);
			uart_puts(buf);
			uart_puts("\r\n");
			uart_puts("e.Low out: ");
			itoa(map.lowOut, &buf, 10);
			uart_puts(buf);
			uart_puts("\r\n");
			uart_puts("f.High out: ");
			itoa(map.highOut, &buf, 10);
			uart_puts(buf);
			uart_puts("\r\n");
			uart_puts("g.CC: ");
			itoa(map.cc, &buf, 10);
			uart_puts(buf);
			uart_puts("\r\n");
			uart_puts("h.Channel: ");
			itoa(map.channel, &buf, 10);
			uart_puts(buf);
			uart_puts("\r\n");
			uart_puts("-----------------------");
			uart_puts("\r\n");
		}
}


void monitorMode(uint8_t idx) {
	// Terminal monitor mode
	char buf[8];
	uart_puts("Editing Joint # ");
	itoa(idx, &buf, 10);
	uart_puts(buf);
	uart_puts("\r\n");
	
	while(true) {
		uart_puts(">  ");
		while(uart_getc() != 'x') {wdt_reset();};
		
	}
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main(void) {
	char buf[16]; // General serial IO buffer
	char cmdIn[16]; // Terminal command buffer
	uint8_t cmdCount; // Counter for command buffer
	struct jointMapping currentJoint;
	uint8_t currentIdx;
	uint16_t rx;
	bool cmdWaiting = false;
	bool stream = false;
	bool editing = false;
	uint8_t tmp;
	uint16_t val;
	uint8_t chanOut;
	
	//Set up watchdog timer
	wdt_enable (WDTO_1S);
	#ifdef DEBUG
		uart_init(UART_BAUD_SELECT(UART_BAUD_RATE, F_CPU) );
		uart_puts("Starting; \r\n");
	#endif
	
	sei(); // Enable interrupts
	
	
	strncpy(&currentJoint.device, "Second\0", 7);
	strncpy(&currentJoint.function, "boo\0", 4);
	currentJoint.lowIn = 1000;
	currentJoint.highIn = 233;
	currentJoint.lowOut = 1;
	currentJoint.highOut = 127;
	currentJoint.cc = 0x0C;
	currentJoint.channel = 1;
	
	
	//writeMapping(1, currentJoint);
	
	//dumpMappings();

  
	// SPI setup
	DDRB &= ~(1 << MISO);	// Input
	DDRB |= (1 << SCK);	// Output
	DDRB |= (1 << MOSI);	// Output
	DDRB |= (1 << CS1);	// Output
	DDRB |= (1 << CS2);	// Output
	// enable SPI Master, MSB, SPI mode 0, FOSC/4
	spi_mode(0);
  
	PORTB |= (1 << CS1);
	PORTB |= (1 << CS2);
  
	
	// Main loop
	while(1) {
		// Read ADC values
		readADC();
		
		// Map
		if (!editing) {
			for (tmp = 0; tmp < 1; tmp++) {
				if (adcBuffer[tmp] < (adcLast[tmp] - deadband) | adcBuffer[tmp] > (adcLast[tmp] + deadband)) {
					// Value has changed sufficiently; look for mapping and send it
					// See if there's a mapping for this channel
					currentJoint = readMapping(tmp);
					if (currentJoint.magic[0] == '#' && currentJoint.magic[1] == '#') {
						// Valid mapping
						chanOut = map(adcBuffer[tmp], currentJoint.lowIn, currentJoint.highIn, currentJoint.lowOut, currentJoint.highOut);
						uart_puts("Mapping #");
						itoa(tmp, &buf, 10);
						uart_puts(buf);
						uart_puts(" = ");
						itoa(chanOut, &buf, 10);
						uart_puts(buf);
						uart_puts("\r\n");
					}
				}
			}
		} else {
			// Editing
			if (adcBuffer[currentIdx] < (adcLast[currentIdx] - deadband) | adcBuffer[currentIdx] > (adcLast[currentIdx] + deadband)) {
				chanOut = map(adcBuffer[currentIdx], currentJoint.lowIn, currentJoint.highIn, currentJoint.lowOut, currentJoint.highOut);
				uart_puts("E.Mapping #");
				itoa(currentIdx, &buf, 10);
				uart_puts(buf);
				uart_puts(" = ");
				itoa(chanOut, &buf, 10);
				uart_puts(buf);
				uart_puts("\r\n");
			}
		}
			
			
			
		rx = uart_getc();
		if (!(rx & UART_NO_DATA) && !cmdWaiting) {
			// Data in UART RX buffer
			cmdIn[cmdCount] = (unsigned char)rx;
			if(cmdIn[cmdCount] == '\n') {
				// We have a newline
				cmdWaiting = true;
			} else {
				cmdCount++;
			}
			if (cmdCount > 16) {
				cmdCount = 0;
			}
		}

		if(cmdWaiting) {			
			//Process a waiting command
			if (cmdIn[0] == 'm') {
				tmp = (uint8_t)atoi(&cmdIn[1]);
				//uart_puts(tmp);
				//uart_puts("\r\n");
				if (tmp >= 0 && tmp < 16) {
					uart_puts("Editing mapping #");
					itoa(tmp, &buf, 10);
					uart_puts(buf);
					uart_puts("\r\n");
					currentIdx = tmp;
					cmdWaiting = false;
					editing = true;
					currentJoint = readMapping(currentIdx);
				}
			}
			if (cmdIn[0] == 'p') {
				// Dump existing mapping
				dumpMapping(currentJoint);
			}
			if (cmdIn[0] == 's') {
				// Start streaming values
				uart_puts("Streaming values: \r\n");
				stream = true;
			}
			if (cmdIn[0] == 'x') {
				// Stop streaming
				if (stream) {
					stream = false;
					uart_puts("Stop streaming values: \r\n");
				} else {
					uart_puts("Exiting edit mode.\r\n");
					editing = false;
				}
			}
			if (cmdIn[0] == 'w') {
				// Write to EEPROM
				writeMapping(currentIdx, currentJoint);
				uart_puts("Written to EEPROM\r\n");
			}
			if (cmdIn[0] == 'a') {
				// Device
				cmdIn[cmdCount-1] = '\0';
				strncpy(&currentJoint.device, &cmdIn[1], cmdCount-1);
			}
			if (cmdIn[0] == 'b') {
				// Function
				cmdIn[cmdCount-1] = '\0';
				strncpy(&currentJoint.function, &cmdIn[1], cmdCount-1);
			}
			if (cmdIn[0] == 'c') {
				// Low In
				val = atoi(&cmdIn[1]);
				currentJoint.lowIn = val;
			}
			if (cmdIn[0] == 'd') {
				// High In
				val = atoi(&cmdIn[1]);
				currentJoint.highIn = val;
			}
			if (cmdIn[0] == 'e') {
				// Low Out
				tmp = (uint8_t)atoi(&cmdIn[1]);
				currentJoint.lowOut = tmp;
			}
			if (cmdIn[0] == 'f') {
				// High Out
				tmp = (uint8_t)atoi(&cmdIn[1]);
				currentJoint.highOut = tmp;
			}
			if (cmdIn[0] == 'g') {
				// CC
				tmp = (uint8_t)atoi(&cmdIn[1]);
				currentJoint.cc = tmp;
			}
			if (cmdIn[0] == 'h') {
				// Channel
				tmp = (uint8_t)atoi(&cmdIn[1]);
				currentJoint.channel = tmp;
			}
			cmdWaiting = false;
			cmdCount = 0;
		}
		
		if (stream) {
			// Stream values for this channel
			itoa(adcBuffer[currentIdx], &buf, 10);
			uart_puts("Raw: ");
			uart_puts(buf);
			uart_puts("\r\n");
		}

		
		// Poke watchdog
		_delay_ms(150);
		wdt_reset();	
	}
	
	return 0;
}


