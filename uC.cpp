#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#define SCL_CLOCK 100000L // 100kHz standard I2C clock speed
#define LCD_ADDR 0x3E      // Common Grove LCD I2C address

void TWI_init(void) {
	TWSR0 = 0x00; // Prescaler = 1
	TWBR0 = ((F_CPU / SCL_CLOCK) - 16) / 2; // Bit rate register
}

void TWI_start(void) {
	TWCR0 = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT); // Send START
	while (!(TWCR0 & (1 << TWINT)));
}

void TWI_stop(void) {
	TWCR0 = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT); // Send STOP
	_delay_ms(1);
}

void TWI_write(uint8_t data) {
	TWDR0 = data;
	TWCR0 = (1 << TWEN) | (1 << TWINT); // Start transmission
	while (!(TWCR0 & (1 << TWINT)));
}

void LCD_sendCommand(uint8_t cmd) {
	TWI_start();
	TWI_write(LCD_ADDR << 1); // Write address
	TWI_write(0x80);          // Co = 1, RS = 0 (Command Mode)
	TWI_write(cmd);
	TWI_stop();
}

void LCD_sendData(uint8_t data) {
	TWI_start();
	TWI_write(LCD_ADDR << 1); // Write address
	TWI_write(0x40);          // Co = 0, RS = 1 (Data Mode)
	TWI_write(data);
	TWI_stop();
}
void LCD_init(void) {
	_delay_ms(50); // Wait for LCD power up
	LCD_sendCommand(0x38); // 8-bit, 2 line, normal font
	LCD_sendCommand(0x39); // Function set
	LCD_sendCommand(0x14); // Internal OSC freq
	LCD_sendCommand(0x70); // Contrast set
	LCD_sendCommand(0x56); // Power/icon/contrast control
	LCD_sendCommand(0x6C); // Follower control
	_delay_ms(200);
	LCD_sendCommand(0x38); // Function set
	LCD_sendCommand(0x0C); // Display ON
	LCD_sendCommand(0x01); // Clear display
	_delay_ms(2);
}

void LCD_setCursor(uint8_t col, uint8_t row) {
	uint8_t address = (row == 0) ? col : (0x40 + col);
	LCD_sendCommand(0x80 | address);
}

void LCD_print(const char *str) {
	while (*str) {
		LCD_sendData(*str++);
	}
}



// ------------------- TIMER1 FOR micros() -------------------
volatile uint32_t microseconds = 0;

void timer1_init() {
	TCCR1B |= (1 << WGM12);    // CTC mode
	OCR1A = 15;                // 1 microsecond tick (16MHz / 8 / 2 = 1us)
	TCCR1B |= (1 << CS11);     // Prescaler 8
	TIMSK1 |= (1 << OCIE1A);   // Enable compare interrupt
	sei();                     // Enable global interrupts
}

ISR(TIMER1_COMPA_vect) {
	microseconds++;
}

uint32_t micros() {
	uint32_t us;
	cli();
	us = microseconds;
	sei();
	return us;
}




// ------------------- GPIO Helpers -------------------
#define TRIG_ECHO_PIN PB0

void pinModeOutput() {
	DDRB |= (1 << TRIG_ECHO_PIN);
}

void pinModeInput() {
	DDRB &= ~(1 << TRIG_ECHO_PIN);
}

void digitalWriteHigh() {
	PORTB |= (1 << TRIG_ECHO_PIN);
}

void digitalWriteLow() {
	PORTB &= ~(1 << TRIG_ECHO_PIN);
}

uint8_t digitalReadPin() {
	return (PINB & (1 << TRIG_ECHO_PIN)) != 0;
}

// ------------------- Pulse and Measurement -------------------
uint32_t MicrosDiff(uint32_t begin, uint32_t end) {
	return end - begin;
}

uint32_t mypulseIn(uint32_t timeout) {
	uint32_t begin = micros();

	// Wait for previous pulse to end
	while (digitalReadPin()) {
		if (MicrosDiff(begin, micros()) >= timeout) return 0;
	}

	// Wait for pulse to start
	while (!digitalReadPin()) {
		if (MicrosDiff(begin, micros()) >= timeout) return 0;
	}
	uint32_t pulseBegin = micros();

	// Wait for pulse to end
	while (digitalReadPin()) {
		if (MicrosDiff(begin, micros()) >= timeout) return 0;
	}
	uint32_t pulseEnd = micros();

	return MicrosDiff(pulseBegin, pulseEnd);
}

uint32_t duration(uint32_t timeout) {
	pinModeOutput();
	digitalWriteLow();
	_delay_us(2);
	digitalWriteHigh();
	_delay_us(5);
	digitalWriteLow();
	pinModeInput();
	return mypulseIn(timeout);
}

uint32_t MeasureInCentimeters(uint32_t timeout) {
	uint32_t dur = duration(timeout);
	return dur ; // speed of sound approximation (us to cm)
}

// ------------------- Main Program -------------------
int main(void) {
	char buffer[16];
	timer1_init();           // Initialize Timer1 for micros()
TWI_init();
		LCD_init();
	while (1) {
		uint32_t range_cm = MeasureInCentimeters(1000000);  // 1s timeout
		
		
		snprintf(buffer, sizeof(buffer), "Range: %u cm", (unsigned int)range_cm);
		LCD_setCursor(0, 0);
		LCD_print(buffer);
		LCD_setCursor(0, 1);
		LCD_print("ATmega328PB :)");
		_delay_ms(500);
	}
}
