#include <avr/io.h>
#include <util/delay.h>


// Servo constants
#define PIN_MIN 1500  // Min ms for PWM
#define PIN_MID 3000  // Mid ms for PWM
#define PIN_MAX 4500  // Max ms for PWN
#define PIN_RANGE (PIN_MAX - PIN_MIN)  // Range between max and min

// Servo variables
boolean searching = true;

// ADC variables
volatile int readFlag;  // High when a value is ready to be read
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
  ADCSRA = (1 << ADEN) | (1 << ADPS2);
  ADMUX = (1 << REFS0) | (1 << MUX1);

  // Set ADIE in ADCSRA (0x7A) to enable the ADC interrupt.
  // Without this, the internal interrupt will not trigger.
  ADCSRA |= (1 << ADIE);

  // Enable global interrupts
  // AVR macro included in <avr/interrupts.h>, which the Arduino IDE
  // supplies by default.
  sei();


  // Set ADSC in ADCSRA (0x7A) to start the ADC conversion
  ADCSRA |= (1 << ADSC);
}


/* Interrupt service routine for the ADC completion */
ISR(ADC_vect) {
  // Must read low first
  analogVal = ADCL | (ADCH << 8);

  if (analogVal >= 300) {
    check_for_face();
  }
  
  // Set ADSC in ADCSRA (0x7A) to start the ADC conversion
  ADCSRA |= (1 << ADSC);
}


/* Initialize SPI Master Device */
void init_spi (void) {
  // Set MOSI, SCK as Output
  DDRB |= (1 << 5) | (1 << 3);

  // Enable SPI, Set as Master
  //Prescaler: Fosc/16, Enable Interrupts
  SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
}


/* Function to send and receive data for both master and slave */
unsigned char spi_tranceiver (unsigned char data) {
  // Load data into the buffer
  SPDR = data;

  // Wait until transmission complete
  while(!(SPSR & (1 << SPIF)));

  // Return received data
  return(SPDR);
}


/*
Rotates towards given degree.
 */
void rotate_towards(int degree) {
  float percentage = degree/180.0;
  OCR1A = (percentage * (PIN_MAX - PIN_MIN)) + PIN_MIN;
}


/*
Rotates continuously, and blocks until searching is turned false.
 */
void search() {
  int incrementer = 1;
  int degree = 90;

  while (searching) {
    // Sets increment based on whether the boundaries were passed.
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
  DDRD |= (1 << PD7);
  PORTD |= (1 << PD7);
  _delay_ms(10);
  PORTD &= ~(1 << PD7);
  // Transmits request to BeagleBone and returns a response.
  //char data = spi_tranceiver('1');
}

int main() {
  init_servo();
  init_spi();
  init_ir();

  search();
}







