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
  uint16_t ticks;
} burst_t;
burst_t q[Q_LENGTH];
volatile uint16_t qhead = 0;
volatile uint16_t qtail = 0;

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
  if (!qhead == qtail) {
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
    timer_start();
  }
  pulse_count++;
  PORTB.INTFLAGS = PORT_INT0_bm;
}

// Called at expiry of the quarter-bit timer
ISR(TCB0_INT_vect){
  LED_OFF();
  timer_stop();
  burst_t b = {pulse_count, BURST_TIMER.CNT};
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

void loop() {
  burst_t b;
  if (qread(&b)) {
    Serial.print(b.pulses, DEC);
    Serial.print(b.ticks, DEC);
  }
}


