#include "ramp.h"
#include <stdio.h>
#include <stdbool.h>

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

uint8_t phasetable[6][3] = {
	{ 0b00, 0b01, 0b11 },
	{ 0b01, 0b00, 0b11 },
	{ 0b01, 0b11, 0b00 },
	{ 0b00, 0b11, 0b01 },
	{ 0b11, 0b00, 0b01 },
	{ 0b11, 0b01, 0b00 },
};
uint8_t const phasecount = sizeof(phasetable) / sizeof(*phasetable);


struct motion {
	uint32_t stepsleft;
	uint32_t rampindex;
	uint32_t rampmax;
	uint32_t rampfactor;
	int32_t position;
} motion = { .position = 0, .stepsleft = 0 };

void uart_putchar(char c, FILE *stream) {
	loop_until_bit_is_set(UCSR0A, UDRE0);
	UDR0 = c;
}
char uart_getchar(FILE *stream) {
	loop_until_bit_is_set(UCSR0A, RXC0); /* Wait until data exists. */
	return UDR0;
}

FILE uart_io = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

void setup_uart(void)
{
	#define BAUD 115200
	UBRR0 = 8;

	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8-bit data */
	UCSR0B = _BV(RXEN0) | _BV(TXEN0);   /* Enable RX and TX */

	/* Set frame format: 8 data bits */
	UCSR0C |= 1*_BV(UCSZ01) | 1*_BV(UCSZ00);

	/* Enable receiver and transmitter */
	UCSR0B |= _BV(RXEN0) | _BV(TXEN0);

	stdout = &uart_io;
	stdin  = &uart_io;
}

void delay_us(uint32_t n)
{
	while(n--)
		_delay_us(1);
}

static inline void set_phase(uint8_t phase)
{
	uint8_t port = PORTD;
	port &= ~0b11111100;

	port |= phasetable[phase][0] << 2;
	port |= phasetable[phase][1] << 4;
	port |= phasetable[phase][2] << 6;

	PORTD = port;
}

static void setup_timer(void)
{
	TCCR1A = 0;
	TCCR1B = 0;
	TCCR1C = 0;

	TCCR1B |= 1*_BV(WGM13) | 1*_BV(WGM12);
	TCCR1A |= 1*_BV(WGM11) | 0*_BV(WGM10);
	// ICR1 = interval
	//TCCR1A |= 1*_BV(COM1A1) | 0*_BV(COM1A0);

	// enable overflow ISR
	TIMSK1 |= _BV(TOIE1);

	// prescaler 1
	OCR1A = (2e-6 * F_CPU / 1);
}

static inline void start_timer(void)
{
	TCCR1B |= (0b001 << CS10);
}

static inline void stop_timer(void)
{
	TCCR1B &= ~(0b111 << CS10);
}

ISR(TIMER1_OVF_vect)
{
	// set ICR1
}

static void setup_stepper(void)
{
	PORTD &= ~0b11111100;
	DDRD |= 0b11111100;

	//setup_timer();
}

static void setup(void)
{
	DDRB |= _BV(DDB5); // D13 LED

	setup_uart();

	setup_stepper();

	sei();
}

int main(void)
{
	setup();

	puts("Hello!");

	signed char const dir = +1;

	uint32_t rampfactor = 40000;
	uint32_t rampfactor_ = rampfactor;
	uint32_t rampindex = 0;
	uint32_t rampindex_ = 0;
	uint32_t const rampmax = 17500;

	uint16_t kshift = 0;
	uint32_t counter = 0;
	bool foo = false;

	while (false)
	{
		char c = getchar();
		counter += 1;
		printf("counter %lu, phase %lu\n", counter, counter % phasecount);
		set_phase(counter % phasecount);
	}

	getchar();

    while(1)
    {
		set_phase((counter * dir) % phasecount);
		//printf("phase %ld\n", (counter * dir) % phasecount);

		counter += 1;
		if (!(counter & 0xff))
			PORTB ^= _BV(PORTB5);

		//_delay_us(5000);
		if (rampindex_ >= ramplen)
		{
			//printf("kshift %d, rampindex_ %u, rampfactor_ %u\n", kshift, rampindex_, rampfactor_);
			rampindex_ >>= 2;
			rampfactor_ >>= 1;
			kshift += 1;
			putchar('%');
			//printf("kshift %d, rampindex_ %u, rampfactor_ %u\n", kshift, rampindex_, rampfactor_);
		}
		uint32_t const delay = (rampfactor_ * (uint32_t)ramp[rampindex_]) >> 16;
		//printf("delay %lu\n", delay);
		if (delay > 0) delay_us(delay);

		if (rampindex < rampmax)
		{
			rampindex += 1;
			uint16_t mod = rampindex & ((1UL << (2*kshift))-1);
			//printf("1<<2k = %u\n", mod);
			if (mod == 0)
			{
				rampindex_ += 1;
			}
		}
		else if (!foo)
		{
			//puts("ramp done");
			foo = true;
		}

    }
}
