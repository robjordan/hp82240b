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
  S1 = 2,
  S2 = 3,
  S3 = 4
} symbol_t;

typedef enum {
  HALFBIT = 427,
  ONEBIT = 854,
  ONEANDAHALFBITS = 1282,
  GAPERROR = 0
} gap_len_t;

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
  case S1:
  case S2:
  case S3:
    value = 'S';  
    break;
  
  case ZERO:
    value = '0';
    break;

  case ONE:
    value = '1';
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

gap_len_t gap_len(uint16_t micros) {
  const uint16_t leeway = 85;     // pulse intervals may be +/- this much

  if (abs(micros - HALFBIT) < leeway) {
    return HALFBIT;
  } else if (abs(micros - ONEBIT) < leeway) {
    return ONEBIT;
  } else if (abs(micros - ONEANDAHALFBITS) < leeway) {
    return ONEANDAHALFBITS;
  } else {
    return GAPERROR;
  }
}

// Receive a burst (pulse count and interval since last burst) and assemble
// into a byte using a state machine. Returns 0 if incomplete, and the byte 
// once a valid byte has been received.
byte process_burst(burst_t b) {
  const uint8_t MAX_SYMBOLS = 15; // 3 start bits, 4 error correctn,  8 data
  static symbol_t bit[MAX_SYMBOLS];
  static uint8_t count = 0;
  gap_len_t gap;

  if ((b.pulses < 5) || (b.pulses > 9)) {
    // ignore faulty bursts
  } else if (count == 0) {
    // The first bit is assumed to be first start bit; gap doesn't matter
    bit[count++] = S1;
  } else if ((gap = gap_len(b.micros)) == GAPERROR) {
      protocol_error(b, bit, count, __LINE__);
      count = 0;
      // Hence we return to the home state for the FSM
  } else {
    // behaviour depends on previous bit
    switch (bit[count-1]) {
    case S1:
      if (gap == HALFBIT) {
        bit[count++] = S2;
      } else {
        protocol_error(b, bit, count, __LINE__);
        count = 0;
      }
      break;
    
    case S2:
      if (gap == HALFBIT) {
        bit[count++] = S3;
      } else {
        protocol_error(b, bit, count, __LINE__);
        count = 0;
      }
    break;

    case S3:
    case ZERO:
      if (gap == HALFBIT) {
        bit[count++] = ONE;
      } else if (gap == ONEBIT) {
        bit[count++] = ZERO;
      } else {
        protocol_error(b, bit, count, __LINE__);
        count = 0;
      }          
      break;

    case ONE:
      if (gap == ONEANDAHALFBITS) {
        bit[count++] = ZERO;
      } else if (gap == ONEBIT) {
        bit[count++] = ONE;
      } else {
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


