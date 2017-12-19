#define BAUD 9600
#include <util/setbaud.h>
#include <avr/io.h>
#include <util/delay.h>


// Servo constants
#define PIN_MIN 1500  // Min ms for PWM
#define PIN_MID 3000  // Mid ms for PWM
#define PIN_MAX 4500  // Max ms for PWN
#define PIN_RANGE (PIN_MAX - PIN_MIN)  // Range between max and min

// Servo variables
boolean searching = true; // Controls whether the device will keep searching

// ADC variables
volatile int analogVal;  // Value to store analog result


/* Initialization for servo */
void init_servo() {
  // Sets up Fast PWM
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM12) | (1 << WGM13);

  // Sets prescaler at 8
  TCCR1B |= (1 << CS11);

  // Sets clear on match, set at bottom
  TCCR1A |= (1 << COM1A1);

  // Sets PB1 to output
  DDRB |= (1 << PB1);

  // Sets match compare to 40000 (2 * 20000 ms)
  ICR1 = 40000;

  // Sets OCR1A to start at min
  OCR1A = PIN_MIN;

  // Sets searching state to true
  searching = true;
}


/* Infrared sensor ADC initialization */
void init_ir() {
  // (1 << ADEN) Turns on ADC
  // (1 << ADPS2) sets clock prescaler to 16
  ADCSRA = (1 << ADEN) | (1 << ADPS2);

  // (1 << REFS0) makes it so AVCC with external capacitor at the AREF pin is used as VRef
  // (1 << MUX1) sets ADC input to ADC2
  ADMUX = (1 << REFS0) | (1 << MUX1);

  // Set ADIE in ADCSRA (0x7A) to enable the ADC interrupt
  ADCSRA |= (1 << ADIE);

  // Enable global interrupts
  sei();

  // Set ADSC in ADCSRA (0x7A) to start the ADC conversion
  ADCSRA |= (1 << ADSC);
}


/* Interrupt service routine for the ADC completion */
ISR(ADC_vect) {
  // Must read low first
  analogVal = ADCL | (ADCH << 8);

  // Calls method if object is detected
  if (analogVal >= 300) {
    check_for_face();
  }

  // Set ADSC in ADCSRA (0x7A) to start the ADC conversion
  ADCSRA |= (1 << ADSC);
}


/* Configuring UART Communication in order to send data */
void init_serial(void)
{
  UBRR0H = UBRRH_VALUE;
  UBRR0L = UBRRL_VALUE;

#if USE_2X
  UCSR0A |= _BV(U2X0);
#else
  UCSR0A &= ~(_BV(U2X0));
#endif

  UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8-bit data */
  UCSR0B = _BV(RXEN0) | _BV(TXEN0);   /* Enable RX and TX */
}


/*
  Rotates towards given degree.
*/
void rotate_towards(int degree) {
  float percentage = degree / 180.0;
  OCR1A = (percentage * (PIN_MAX - PIN_MIN)) + PIN_MIN;
}


/*
  Rotates continuously.
*/
void search() {
  int incrementer = 1;
  int degree = 90;

  while (searching) {
    // Sets increment based on whether the boundaries were passed
    if (degree <= 0) {
      incrementer = 1;
    }
    else if (degree >= 180) {
      incrementer = -1;
    }

    // Increments OCR1A
    degree += incrementer;
    rotate_towards(degree);

    _delay_ms(10);
  }
}

/*
  Communicates with BeagleBone to know if a face is detected.
  Lights up an LED if such is the case.
*/
void check_for_face() {
  char data;
  
  // Transmits request to BeagleBone and receives a response
  loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
  UDR0 = '1';
  loop_until_bit_is_set(UCSR0A, RXC0); /* Wait until data exists. */
  data = UDR0;
  
  // Lights up LED through PD7 if a face was detected by Beaglebone
  if (data == '1') {
    DDRD |= (1 << PD7);
    PORTD |= (1 << PD7);
    _delay_ms(1000);
    PORTD &= ~(1 << PD7);
  }
}

int main() {
  init_servo();
  init_serial();
  init_ir();

  search();
}







