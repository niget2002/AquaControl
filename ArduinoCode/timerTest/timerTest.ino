// avr-libc library includes
#include <avr/io.h>
#include <avr/interrupt.h>
#define LEDPIN 13

void setup()
{
pinMode(LEDPIN, OUTPUT);
// initialize Timer1
cli();         // disable global interrupts
TCCR1A = 0;    // set entire TCCR1A register to 0
TCCR1B = 0;    // set entire TCCR1B register to 0 
               // (as we do not know the initial  values) 

// enable Timer1 overflow interrupt:
TIMSK1 |= (1 << TOIE1); //Atmega8 has no TIMSK1 but a TIMSK register

// Set CS10 bit so timer runs at clock speed: (no prescaling)
TCCR1B |= (1 << CS10); // Sets bit CS10 in TCCR1B
TCCR1B |= (1 << CS12);
// This is achieved by shifting binary 1 (0b00000001)
// to the left by CS10 bits. This is then bitwise
// OR-ed into the current value of TCCR1B, which effectively set
// this one bit high. Similar: TCCR1B |= _BV(CS10);

// enable global interrupts:
sei();
}

ISR(TIMER1_OVF_vect)
{

digitalWrite(LEDPIN, !digitalRead(LEDPIN));
// or use: PORTB ^= _BV(PB5);// PB5 =pin 19 is digitalpin 13
}

void loop() {}
