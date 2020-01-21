//////////////////////////////////////////
// This is the ubiquitous Blink program //
//////////////////////////////////////////
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>


#define DDRB_P0    0x01
#define DDRB_P1    0x02
#define DDRB_P2    0x04
#define DDRB_P3    0x08

#define PORTB_P0    0x01
#define PORTB_P1    0x02
#define PORTB_P2    0x04
#define PORTB_P3    0x08

// Photo transistor on PB0
#define PHOTO    ((PINB & PORTB_P0) == 0)

// Out pulse on PB1
#define OUT_PULSE_ON     (PORTB |= PORTB_P1)
#define OUT_PULSE_OFF   (PORTB &= ~(PORTB_P1))

// Opto emitter on PB2
#define EMITTER_ON    (PORTB &= ~(PORTB_P2))
#define EMITTER_OFF    (PORTB |= PORTB_P2)

int main (void)
{
    unsigned char counter = 0;
    DDRB |= DDRB_P2 | DDRB_P1;    // PB2, PB1 output

    while(1)
    {
        _delay_us(7300);
        EMITTER_ON;

        //el pulso dura 160us
        counter = 0;
        _delay_us(80);        
        if (PHOTO)
            counter++;

        _delay_us(20);    //100us
        if (PHOTO)
            counter++;

        _delay_us(20);    //120us
        if (PHOTO)
            counter++;

        _delay_us(20);    //140us
        if (PHOTO)
            counter++;

        _delay_us(20);    //160us
        if (PHOTO)
            counter++;
        
        if (counter > 2)    //si llego el pulso (3 o mas chequeos)
            OUT_PULSE_ON;
        else
            OUT_PULSE_OFF;

        //el pulso dura 160us
        // _delay_us(160);
        // if (PHOTO)
        //     OUT_PULSE_ON;
        // else
        //     OUT_PULSE_OFF;
        
        EMITTER_OFF;
    }

  // TCCR0A = 1<<COM0A0 | 0<<WGM00;  // Toggle OC0A, CTC mode
  // TCCR0B = 1<<WGM02 | 3<<CS00;    // CTC mode; use OCR0A; /64
  // OCR0A = 15624;                  // 1 second; ie 0.5Hz
  // while (1);
}


// #include <avr/io.h>               // import definitions
// #include <avr/interrupt.h>        // import cli()

// int main (void) {
//   char oldsreg = SREG;            // save the interrupt setting register to oldsreg
//   cli();                          // disable all the interrupts (SREG = 0)
//   CCP = 0xD8;                     // signature to CCP
//   CLKMSR = 0;                     // use clock 00: Calibrated Internal 8 MHzOscillator
//   CCP = 0xD8;                     // signature
//   CLKPSR = 0;                     // set prescaler to :1 (0x00)
//   SREG = oldsreg;                 // restore the sreg, enabling the interrupts

//   DDRB = 1;                       // set DDRB.PB0 to output
//   OCR0A = 7812;                   // set OCR0A to 7812. 2Hz toggle.
//   TCCR0A = 1<<COM0A0;             // set timer to toggle PB0 on compare match
//   TCCR0B = 1<<WGM02 | 3<<CS00;    // set CTC mode | enable and set prescaler :64
//   while(1);                       // wait forever
// }


// int main (void)
// {
//     while (1);

//     return 0;
// }

//////////////////
// TEST PROGRAM //
//////////////////
/*
  Read ADC2 (PB2) and output servo signal on PB0 and PB1.
  ATtiny5 or 10 at default 1MHz.
 */

// #include <avr/io.h>

// int main(void)
// {
//     // ADC channel 2
//     ADMUX = 2;
//     // Disable digital input on PB2
//     DIDR0 = (1<<ADC2D);
//     // Enable ADC, presc 1:8 for 125kHz ADC-clock
//     ADCSRA = (1<<ADEN) | (1<<ADPS1) | (1<<ADPS0);

//     // PB0 and 1 output.
//     DDRB = (1<<PB0) | (1<<PB1);

//     // Timer0 50Hz PWM, 1us tick, mode 14 with ICR0 as TOP.
//     ICR0 = 19999; // 1MHz / 50Hz - 1
//     // Start with 1.5 ms pulse-width
//     OCR0A = OCR0B = 1500;

//     // OC0A and OC0B non inverting PWM, mode bit 1
//     TCCR0A = (1<<COM0A1) | (1<<COM0B1) | (1<<WGM01);
//     // mode bits 2 and 3, presc 1:1
//     TCCR0B = (1<<WGM03) | (1<<WGM02) | (1<<CS00);
    

//     while(1)
//     {
// 	uint8_t i;
// 	uint16_t adc4;

// 	// Take four ADC samples, add them in adc4
// 	for (i = 0, adc4 = 0; i < 4; i++)
// 	{
// 	    // Start a conversion
// 	    ADCSRA |= (1<<ADSC);
// 	    // wait until it's finished
// 	    while (ADCSRA & (1<<ADSC))
// 		; // Nothing
// 	    adc4 += ADCL;
// 	}

// 	// Set PWM to 990 to 2010 ms from ADC.
// 	OCR0A = 990 + adc4;
// 	// OCR0B "inverted" servo signal
// 	OCR0B = 2010 - adc4;
//     }
// }


/////////////////////////
// Toggle a LED at 1Hz //
/////////////////////////
// #include <avr/io.h>
// #include <util/delay.h>

// int main(void)
// {
//     // PB2 output
//     DDRB = 1<<2;

//     while(1)
//     {
// 	// Toggle PB2
// 	PINB = 1<<2;
// 	_delay_ms(500);
//     }
// }



/////////////////////////////////////
// Short 1Hz flash using the timer //
/////////////////////////////////////
// #include <avr/io.h>
// #include <util/delay.h>

// int main(void)
// {
//     TCNT0 = 0;
//     OCR0A = 0;
//     // Start timer, free running with presc. 1:64
//     TCCR0B = 1<<CS01 | 1<<CS00;
    
//     DDRB = 1<<2;

//     while(1)
//     {
// 	// Short flash
// 	PORTB |= 1<<2;
// 	_delay_ms(1);
// 	PORTB &= ~(1<<2);

// 	// Add one second
// 	OCR0A += F_CPU / 64 - 1;
// 	// Wait for compare match
// 	while(!(TIFR0 & (1<<OCF0A)))
// 	    ;
// 	// Clear interrupt flag
// 	TIFR0 |= 1<<OCF0A;
//     }
// }



////////////////////////////////////////////
// Fade PB0 and PB1 up and down using PWM //
////////////////////////////////////////////
// #include <avr/io.h>
// #include <util/delay.h>


// int main(void)
// {
//     int i;
    
//     // PB0 and PB1 outputs
//     DDRB = (1<<PB0) | (1<<PB1);
//     // Timer0 in mode 14, fast PWM with ICR0 as top.
//     // Enable OC0A and OC0B, lower mode bits
//     TCCR0A = (1<<COM0A1) | (1<<COM0B1) | (1<<WGM01);
//     // Set top to 1000
//     ICR0 = 1000;
//     // Start timer with prescaler 1:8 and set upper mode bits
//     TCCR0B = (1<<CS01)  | (1<<WGM03) | (1<<WGM02);

//     while(1)
//     {
// 	for (i = 0; i <= 1000; i++)
// 	{
// 	    OCR0A = i;
// 	    OCR0B = 1000-i;
// 	    _delay_ms(1);
// 	}
// 	for (i = 1000; i >= 0; i--)
// 	{
// 	    OCR0A = i;
// 	    OCR0B = 1000-i;
// 	    _delay_ms(1);
// 	}
//     }
// }


///////////////////////////////////////////
// Toggle a LED when a button is pressed //
///////////////////////////////////////////
/*
  Button on PB2 to GND, LED on PB0.
  Toggle led when button is pressed.
  Button is debounced using timer interrupt.
 */
// #include <avr/io.h>
// #include <avr/interrupt.h>
// #include <util/delay.h>

// // Variable to tell main that the button is pressed (and debounced).
// // Main will clear it after a detected button press.
// volatile uint8_t button_down;

// // Check button state on PB2 and set the button_down variable if a debounced
// // button down press is detected.
// // Call this function about 100 times per second.
// static inline void debounce(void)
// {
//     // Counter for number of equal states
//     static uint8_t count = 0;
//     // Keeps track of current (debounced) state
//     static uint8_t button_state = 0;

//     // Check if button is high or low for the moment
//     uint8_t current_state = (~PINB & (1<<PINB2)) != 0;
    
//     if (current_state != button_state) {
//         // Button is about to be pressed or released, increase counter
//         count++;
//         if (count >= 4) {
//             // The button have not bounced for four checks, change state
//             button_state = current_state;
//             // If the button was pressed (not released), tell main so
//             if (current_state != 0) {
// 		button_down = 1;
//                 count = 0;
//             }
//         }
//     } else {
//         // Reset counter
//         count = 0;
//     }
// }


// // Called at 100Hz
// ISR(TIM0_OVF_vect)
// {
//     debounce();
// }

// int main(void)
// {
//     // PB0 output
//     DDRB = 1<<0;
//     // Enable internal pullup on PB2
//     PUEB = 1<<2;

//     // Timer0 in CTC mode
//     TCCR0A = (1<<WGM01) | (1<<WGM00);
//     // Set TOP for 100 Hz overflow rate
//     OCR0A = F_CPU / 8 / 100 - 1;
//     // Enable overflow interrupt
//     TIMSK0 = 1<<TOIE0;
//     // Start timer with presc 1:8
//     TCCR0B = 1<<CS01;
//     // Enable global interrupts
//     sei();

//     while(1)
//     {
//         if (button_down) {
//             // Clear flag
//             button_down = 0;
// 	    // Toggle PB0
// 	    PORTB ^= (1<<0);
//         }
//     }
// }

