#include <Arduino.h>
#include <avr/interrupt.h>

#define BURST_TIMER TCB0
#define PULSE_TICKS (305*2)
#define BURST_TICKS (2441*2)
#define BURST_TIMER_START() BURST_TIMER.CTRLA |= TCB_ENABLE_bm
#define BURST_TIMER_STOP() BURST_TIMER.CTRLA &= ~(TCB_ENABLE_bm)
#define LED_ON() VPORTA.OUT |= PIN7_bm
#define LED_OFF() VPORTA.OUT &= ~(PIN7_bm)
#define LED_TOGGLE() VPORTA.IN = PIN7_bm
#define Q_LENGTH (128)

typedef struct {
  uint16_t pulses;
  uint16_t micros; // microseconds
} burst_t;
burst_t q[Q_LENGTH];
volatile uint16_t qhead = 0;
volatile uint16_t qtail = 0;

typedef enum {
  ZERO = 0,
  ONE = 1,
  START1 = 2,
  START2 = 3,
  START3 = 4,
  MISSING = 5
} symbol_t;

// typedef enum {
//   HALFBIT = 427,
//   ONEBIT = 854,
//   ONEANDAHALFBITS = 1282,
//   GAPERROR = 0
// } gap_len_t;

bool qwrite(burst_t b) {
  uint16_t nexti = (qhead + 1) % Q_LENGTH;
  if (nexti == qtail) {
    // Queue full
    return false;
  } else {
    q[qhead] = b;
    qhead = nexti;
    return true;
  }
}

bool qread(burst_t *val) {
  if (qhead != qtail) {
    *val = q[qtail];
    qtail = (qtail+1) % Q_LENGTH;
    return true;
  } else {
    return false;
  }
}

uint16_t qlength() {
  if (qhead < qtail) {
    return qhead + Q_LENGTH - qtail;
  } else {
    return qhead - qtail;
  }
}

volatile uint8_t pulse_count;
volatile unsigned long previous = 0;
volatile unsigned long interval = 0;

void timer_init() {
  _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, 0);
  BURST_TIMER.CCMP = BURST_TICKS;        // 244us (max length of quarter-bit)

}

// Start (or restart) a timer of 8 x 32kHz i.e. 244us
void timer_start() {
  BURST_TIMER.CNT = 0;
  BURST_TIMER.CCMP = BURST_TICKS;
  BURST_TIMER.INTCTRL = TCB_CAPT_bm; //irq enable
  BURST_TIMER_START();
}

void timer_stop() {
  BURST_TIMER.INTCTRL = 0; //irq disable
  BURST_TIMER_STOP(); 
}

// Interrupt triggered on falling edge from IR receiver
ISR(PORTB_PORT_vect) {
  // start a timer
  if (pulse_count == 0) {
    LED_ON();  // pin high = LED on
    // Ticks since start of the previous burst
    interval = micros() - previous;
    previous = micros();
    timer_start();
  }
  pulse_count++;
  PORTB.INTFLAGS = PORT_INT0_bm;
}

// Called at expiry of the quarter-bit timer
ISR(TCB0_INT_vect){
  LED_OFF();
  timer_stop();
  burst_t b = {pulse_count, (uint16_t)interval};
  if (!qwrite(b)) {
    Serial.println("Queue full");
  }
  pulse_count = 0;
  TCB0.INTFLAGS = TCB_CAPT_bm;
}

void setup() {
  delay(1000);
  Serial.begin(115200);
  Serial.println("starting");
  // LED for debugging
  VPORTA.DIR |= PIN7_bm;  //led on PA7 is an output
  LED_OFF();
  timer_init();
  timer_start();

  // Setup an interrupt on falling edge from IR receiver
  PORTB.PIN0CTRL = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;

}

char symbol_to_char(symbol_t s) {
  char value;

  switch (s)
  {
  case START1:
  case START2:
  case START3:
    value = 'S';  
    break;
  
  case ZERO:
    value = '0';
    break;

  case ONE:
    value = '1';
    break;

  case MISSING:
    value = 'M';
    break;
  
  default:
    value = 'x';
    break;
  }
  return value;
}

void protocol_error(burst_t b, symbol_t s[], uint8_t scount, int line) {
  Serial.printf(
    "protocol error: line: %d, pulses: %d, micros: %d, symbols: ", 
    line,
    b.pulses, 
    b.micros);
  for (int i=0; i<scount; i++) {
    Serial.print(symbol_to_char(s[i]));
  }
  Serial.println("");
}

// How long is the gap from the previous burst, in units of half a bit
uint8_t gap_len(uint16_t gap_us) {
  const uint16_t half_bit_us = 427;  // integer accuracy should be good enough
  const uint16_t quarter_bit_us = 214;

  return ((gap_us + quarter_bit_us) / half_bit_us);
}

// Receive a burst (pulse count and interval since last burst) and assemble
// into a byte using a state machine. Returns 0 if incomplete, and the byte 
// once a valid byte has been received.
byte process_burst(burst_t b) {
  const uint8_t MAX_SYMBOLS = 15; // 3 start bits, 4 error correctn,  8 data
  static symbol_t bit[MAX_SYMBOLS];
  static uint8_t count = 0;
  uint8_t gap;  // measured in half-bits

  if ((b.pulses < 5) || (b.pulses > 9)) {
    // ignore faulty bursts
  } else if (count == 0) {
    // The first bit is assumed to be first start bit; gap doesn't matter
    bit[count++] = START1;
  } else {
    // behaviour depends on previous bit, and gap length in half-bits
    gap = gap_len(b.micros);
    switch (bit[count-1]) {
    case START1:
      if (gap == 1) { // gap from S1 to S2 should be one half bit
        bit[count++] = START2;
      } else {  // we don't cater for missing start bits
        protocol_error(b, bit, count, __LINE__);
        count = 0;
      }
      break;
    
    case START2:
      if (gap == 1) { // gap from S2 to S3 should be one half bit
        bit[count++] = START3;
      } else {
        protocol_error(b, bit, count, __LINE__);
        count = 0;
      }
    break;

    case START3:
    case ZERO:
      switch (gap) {
        case 1: // one half bit
          bit[count++] = ONE;
          break; 

        case 2: // one full bit
          bit[count++] = ZERO;
          break;

        case 3: // one and a half bits
          // presume we missed a bit, and have skipped to a '1'
          bit[count++] = MISSING;
          bit[count++] = ONE;
          break;

        case 4: // two full bits
          // presume we missed a bit, and have skipped to a '0'
          bit[count++] = MISSING;
          bit[count++] = ZERO;
          break;

        default:
          // anything else is a protocol error
          // this doesn't handle two missed bits for now
          protocol_error(b, bit, count, __LINE__);
          count = 0;       
      }    
      break;

    case ONE:
      switch (gap) {
        case 2: // one full bit
          bit[count++] = ONE;
          break;

        case 3: // one and a half bits
          bit[count++] = ZERO;
          break;

        case 4: // two full bits
          // presume we missed a bit, and have skipped to a '1'
          bit[count++] = MISSING;
          bit[count++] = ONE;
          break;

        case 5: // two full bits and a half bit
          // presume we missed a bit, and have skipped to a '0'
          bit[count++] = MISSING;
          bit[count++] = ZERO;
          break;
          
        default:
          // anything else is a protocol error
          // this doesn't handle two missed bits for now
          protocol_error(b, bit, count, __LINE__);
          count = 0;       
      }        
      break;

    default:
      // Should never happen, log as a protocol error
      protocol_error(b, bit, count, __LINE__);
      count = 0;
      break;
    }
  }

  // Whatever occurred in the FSM, we need to assess whether the word is 
  // complete and return a value accordingly
  if (count == MAX_SYMBOLS) {
    // word complete, let's analyse and return its value
    // ADD ERROR CHECK AND CORRECTION
    for (int i=0; i<count; i++) {
      Serial.print(symbol_to_char(bit[i]));
    }
    Serial.print(' ');
    char c = 0;
    for (int i=7; i<count; i++) {
      c = (c<<1) + bit[i];
    }
    Serial.print(c);
    Serial.println("");
    count = 0;
    return 'Y';
  } else {
    return 0;
  }
}

void loop() {
  burst_t b;
  if (qread(&b)) {
    Serial.print(b.pulses, DEC);
    Serial.print(' ');
    Serial.println(b.micros, DEC);
    process_burst(b);
  }
}


