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

char *Rom8_UTF8[] = {
    // Base index in the Roman 8 code page is 0x80
    " ",            // 128: NBSP
    "\xC3\xB7",     // 129: Division sign
    "\xC3\x97",     // 130: Multiplication sign
    "\xE2\x88\x9A", // 131: Square Root
    "\xE2\x88\xAB", // 132: Integral
    "\xCE\xA3",     // 133: Sigma
    "\xE2\x96\xB6", // 134: Right block arrow
    "\xCF\x80",     // 135: Pi (lower)
    "\xE2\x88\x82", // 136: Partial Differential
    "\xE2\x89\xA4", // 137: Less-than Or Equal To
    "\xE2\x89\xA5", // 138: Greater-than Or Equal To
    "\xE2\x89\xA0", // 139: Not equal to
    "\xCE\xB1",     // 140: Alpha (lower)
    "\xE2\x86\x92", // 141: Right Arrow
    "\xE2\x86\x90", // 142: Left arrow
    "\xC2\xB5",     // 143: Micro (mu)
    "\xE2\x90\x8A", // 144: Symbol for Line Feed
    "\xC2\xB0",     // 145: Degree
    "\xC2\xAB",     // 146: Left-Pointing Double Angle Quotation Mark
    "\xC2\xBB",     // 147: Right-Pointing Double Angle Quotation Mark
    "\xE2\x8A\xA6", // 148: Assertion
    "\xE2\x82\x81", // 149: Subscript One
    "\xE2\x82\x82", // 150: Subscript Two
    "\xC2\xB2",     // 151: Superscript Two
    "\xC2\xB3",     // 152: Superscript Three
    "\xE1\xB5\xA2", // 153: Subscript Small Letter i
    "\xE2\xB1\xBC", // 154: Subscript Small Letter j
    "\xE2\x80\xA5", // 155: Two dots
    "\xE2\x81\xB1", // 156: Superscript Small Letter i
    "\xCA\xB2",     // 157: Superscript Small Letter j
    "\xE1\xB5\x8F", // 158: Superscript Small Letter k
    "\xE2\x81\xBF", // 159: Superscript small letter n
    "\xE2\x88\xA0", // 160: Angle
    "\xC3\x80",     // 161: A grave 
    "\xC3\x82",     // 162: A circumflex
    "\xC3\x88",     // 163: E grave
    "\xC3\x8a",     // 164: E circumflex
    "\xC3\x8B",     // 165: E diaerisis
    "\xC3\x8E",     // 166: I circumflex
    "\xC3\x8F",     // 167: I diaerisis
    "\xC2\xB4",     // 168: Acute accent
    "\x60",         // 169: Grave accent
    "\x5e",         // 170: Circumflex / Caret 
    "\C2\xA8",      // 171: diaerisis / Diacritic
    "\x7e",         // 172: Tilde
    "\xC3\x99",     // 173: U grave
    "\xC3\x9B",     // 174: U circumflex 
    "\xC2\xA3",     // 175: UK Pound sign 
    "\xC2\xAF",     // 176: Macron (bar above) 
    "\xC3\x9D",     // 177: Y acute 
    "\xC3\xBD",     // 178: y acute
    "\xCB\x9A",     // 179: Ring diacritic 
    "\xC3\x87",     // 180: C cedilla
    "\xC3\xA7",     // 181: c cedilla
    "\xC3\x91",     // 182: N tilde
    "\xC3\xB1",     // 183: n tilde
    "\xC2\xA1",     // 184: Inverted exclamation mark 
    "\xC2\xBF",     // 185: Inverted question mark 
    "\xC2\xA4",     // 186: Currency sign
    "\xC2\xA3",     // 187: UK Pound sign 
    "\xC2\xA5",     // 188: Yen sign
    "\xC2\xA7",     // 189: Section sign
    "\xC6\x92",     // 190: f with a hook
    "\xC2\xA2",     // 191: cent sign
    "\xC3\xA2",     // 192: a circumflex
    "\xC3\xAA",     // 193: e circumflex
    "\xC3\xB4",     // 194: o circumflex
    "\xC3\xBB",     // 195: u circumflex
    "\xC3\xA1",     // 196: a acute
    "\xC3\xA9",     // 197: e acute
    "\xC3\xB3",     // 198: o acute
    "\xC3\xBA",     // 199: u acute
    "\xC3\xA0",     // 200: a grave
    "\xC3\xA8",     // 201: e grave
    "\xC3\xB2",     // 202: o grave
    "\xC3\xB9",     // 203: u grave
    "\xC3\xA4",     // 204: a diaerisis
    "\xC3\xAB",     // 205: e diaerisis
    "\xC3\xB6",     // 206: o diaerisis
    "\xC3\xBC",     // 207: u diaerisis
    "\xC3\x85",     // 208: A ring 
    "\xC3\xAE",     // 209: i circumflex
    "\xC3\x98",     // 210: O with stroke 
    "\xC3\x86",     // 211: AE
    "\xC3\xA5",     // 212: a ring
    "\xC3\xAD",     // 213: i acute
    "\xC3\xB8",     // 214: o stroke
    "\xC3\xA6",     // 215: ae
    "\xC3\x84",     // 216: A diaerisis
    "\xC3\xAC",     // 217: i grave
    "\xC3\x96",     // 218: O diaeresis
    "\xC3\x9C",     // 219: U diaeresis
    "\xC3\x89",     // 220: E acute
    "\xC3\xAF",     // 221: i diaresis
    "\xCE\xB2",     // 222: beta
    "\xC3\x94",     // 223: O circumflex
    "\xC3\x81",     // 224: A acute
    "\xC3\x83",     // 225: A tilde
    "\xC3\xA3",     // 226: a tilde
    "\xC3\x90",     // 227: large Eth
    "\xC3\xB0",     // 228: small Eth
    "\xC3\x8D",     // 229: I acute
    "\xC3\x8C",     // 230: I grave
    "\xC3\x93",     // 231: O acute
    "\xC3\x92",     // 232: O grave
    "\xC3\x95",     // 233: O tilde
    "\xC3\xB5",     // 234: o tilde
    "\xC5\xA0",     // 235: S with caron
    "\xC5\xA1",     // 236: s with caron
    "\xC3\x9A",     // 237: U acute
    "\xC5\xB8",     // 238: Y diaeresis
    "\xC3\xBf",     // 239: y diaeresis
    "\xC3\x9E",     // 240: Capital thorn
    "\xC3\xBe",     // 241: Small thorn
    "\xC2\xB7",     // 242: Middle dot
    "\xCE\xBC",     // 243: mu
    "\xC2\xB6",     // 244: Pilcrow (paragraph mark)
    "\xC2\xBE",     // 245: 3/4 fraction
    "\xC2\xAD",     // 246: Soft hyphen
    "\xC2\xBC",     // 247: 1/4 fraction
    "\xC2\xBD",     // 248: 1/2 fraction
    "\xC2\xAA",     // 249: a ordinal
    "\xC2\xBA",     // 250: o ordinal
    "\xC2\xAB",     // 251: Left guillemet <<
    "\xE2\x96\xA0", // 252: Black square
    "\xC2\xBB",     // 253: Right guillemet >>
    "\xC2\xB1"      // 254: plus-minus
};

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
const uint8_t MAX_SYMBOLS = 15; // 3 start bits, 4 error correctn,  8 data

typedef enum {
  binary_mode = 0,
  roman_mode = 1   // translate character codes > 127 into Roman8 extended text
} mode_t;
mode_t mode;

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
    Serial.println("\nQueue full");
  }
  pulse_count = 0;
  TCB0.INTFLAGS = TCB_CAPT_bm;
}

void setup() {
  delay(1000);
  Serial.begin(115200);
  // Serial.println("starting");
  // LED for debugging
  VPORTA.DIR |= PIN7_bm;  //led on PA7 is an output
  PORTA.DIRCLR |= PIN3_bm; // set PA3 as an input
  PORTA.PIN3CTRL = PORT_PULLUPEN_bm; // pullup on 
  // let the pullup status settle
  delay(100);
  if (VPORTA.IN & PIN3_bm) {
    mode = roman_mode;
    // Serial.println("Roman mode");
  } else {
    mode = binary_mode;
    // Serial.println("Binary mode");
  }

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
    "\nprotocol error: line: %d, pulses: %d, micros: %d, symbols: ", 
    line,
    b.pulses, 
    b.micros);
  for (int i=0; i<scount; i++) {
    Serial.print(symbol_to_char(s[i]));
  }
  Serial.println("");
}

void data_error(symbol_t s[], uint8_t scount) {
  Serial.printf("\nunrecoverable data error: symbols: ");
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

// Count the number of bits set in an 8-bit byte
uint8_t count_bits(uint8_t value) {
  int count = 0;
  while (value) {
    count += (value & 1);
    value >>= 1;
  }
  return count;
}

// Check whether ECC can fix a missing bit.
// Returns:
// MISSING: Insufficient information to fix the bit
// ZERO:    Sufficient information - the missing bit is 0
// ONE:     Sufficient information - the missing bit is 1
symbol_t check_ecc(
  byte present,
  byte missing,
  byte value, 
  unsigned h_num, 
  unsigned ecc_bit) {
  // The masks corresponding to the error correction bits are held in an array.
  // For consistency with the nomenclature in the spec, hold them in 
  // h[1], h[2], h[3], h[4]; i.e. h[0] is unused
  const byte h[] = {0, 0b01111000, 0b11100110, 0b11010101, 0b10001011};

  // Serial.print("present: ");
  // Serial.println(present, BIN);
  // Serial.print("missing: ");
  // Serial.println(missing, BIN);
  // Serial.print("value: ");
  // Serial.println(value, BIN);
  // Serial.print("h[h_num]: ");
  // Serial.println(h[h_num], BIN);
  // Serial.print("ecc_bit: ");
  // Serial.println(ecc_bit, BIN);
  if (((present | missing) & h[h_num]) == h[h_num]) {
    // we have all the bits needed to reconstruct the missing bit
    byte masked_value = value & h[h_num];
    // Serial.print("masked_value: ");
    // Serial.println(masked_value, BIN);
    if ((count_bits(masked_value) % 2) == ecc_bit)
      // The ECC bit is correct, so our assumption missing bit is '0' is true
      return ZERO;
    else
      // The ECC bit is incorrect, so the missing bit must be '1'
      return ONE;
  } else {
    // We have insufficient information to fix the missing bit, return MISSING
    return MISSING;
  }
}

// Fill in a missing bit based on the ECC bits
symbol_t error_correct(symbol_t bit[], uint8_t missing_bit) {

  // Construct a mask indicating which bits are present, and also a value for
  // the data byte, assuming at this stage that the missing bit is '0'
  byte present = 0;
  byte value = 0;
  for (int i=7; i<MAX_SYMBOLS; i++) {
      present = (present<<1) + (bit[i] != MISSING ? 1 : 0);
      value = (value<<1) + (bit[i]==ONE ? 1 : 0);
  }
  // Construct a mask with one bit set which is the missing bit
  byte missing = 0b10000000 >> missing_bit;
  
  switch (missing_bit)
  {
  symbol_t rc;
  case 0:
    // ECC bits that contain bit 0 are H2, H3 and H4.
    if ((rc = check_ecc(present, missing, value, 2, bit[4])) != MISSING) {
      // We've fixed it using H2: return the missing bit
      return rc;
    } else if ((rc = check_ecc(present, missing, value, 3, bit[5])) != MISSING) {
      // We've fixed it using H3: return the missing bit
      return rc;
    } else if ((rc = check_ecc(present, missing, value, 4, bit[6])) != MISSING) {
      // We've fixed it using H3: return the missing bit
      return rc;
    } else {
      // This error is unfixable. return ZERO, but log an irrecoverable error
      data_error(bit, MAX_SYMBOLS);
      return ZERO;
    }
    break;
  
  case 1:
    // ECC bits that contain bit 1 are H1, H2 and H3.
    if ((rc = check_ecc(present, missing, value, 1, bit[3])) != MISSING) {
      // We've fixed it using H1: return the missing bit
      return rc;
    } else if ((rc = check_ecc(present, missing, value, 2, bit[4])) != MISSING) {
      // We've fixed it using H2: return the missing bit
      return rc;
    } else if ((rc = check_ecc(present, missing, value, 3, bit[5])) != MISSING) {
      // We've fixed it using H3: return the missing bit
      return rc;
    } else {
      // This error is unfixable. return ZERO, but log an irrecoverable error
      data_error(bit, MAX_SYMBOLS);
      return ZERO;
    }
    break;
  
  case 2:
    // ECC bits that contain bit 2 are H1, and H2.
    if ((rc = check_ecc(present, missing, value, 1, bit[3])) != MISSING) {
      // We've fixed it using H1: return the missing bit
      return rc;
    } else if ((rc = check_ecc(present, missing, value, 2, bit[4])) != MISSING) {
      // We've fixed it using H2: return the missing bit
      return rc;
    } else {
      // This error is unfixable. return ZERO, but log an irrecoverable error
      data_error(bit, MAX_SYMBOLS);
      return ZERO;
    }
    break;
  
  case 3:
    // ECC bits that contain bit 3 are H1 and H3.
    if ((rc = check_ecc(present, missing, value, 1, bit[3])) != MISSING) {
      // We've fixed it using H1: return the missing bit
      return rc;
    } else if ((rc = check_ecc(present, missing, value, 3, bit[5])) != MISSING) {
      // We've fixed it using H3: return the missing bit
      return rc;
    } else {
      // This error is unfixable. return ZERO, but log an irrecoverable error
      data_error(bit, MAX_SYMBOLS);
      return ZERO;
    }
    break;
  
  case 4:
    // ECC bits that contain bit 4 are H1 and H4.
    if ((rc = check_ecc(present, missing, value, 1, bit[3])) != MISSING) {
      // We've fixed it using H1: return the missing bit
      return rc;
    } else if ((rc = check_ecc(present, missing, value, 4, bit[6])) != MISSING) {
      // We've fixed it using H2: return the missing bit
      return rc;
    } else {
      // This error is unfixable. return ZERO, but log an irrecoverable error
      data_error(bit, MAX_SYMBOLS);
      return ZERO;
    }
    break;
  
  case 5:
    // ECC bits that contain bit 5 are H2 and H3.
    if ((rc = check_ecc(present, missing, value, 2, bit[4])) != MISSING) {
      // We've fixed it using H1: return the missing bit
      return rc;
    } else if ((rc = check_ecc(present, missing, value, 3, bit[5])) != MISSING) {
      // We've fixed it using H2: return the missing bit
      return rc;
    } else {
      // This error is unfixable. return ZERO, but log an irrecoverable error
      data_error(bit, MAX_SYMBOLS);
      return ZERO;
    }
    break;
  
  case 6:
    // ECC bits that contain bit 6 are H2 and H4.
    if ((rc = check_ecc(present, missing, value, 2, bit[4])) != MISSING) {
      // We've fixed it using H1: return the missing bit
      return rc;
    } else if ((rc = check_ecc(present, missing, value, 4, bit[6])) != MISSING) {
      // We've fixed it using H2: return the missing bit
      return rc;
    } else {
      // This error is unfixable. return ZERO, but log an irrecoverable error
      data_error(bit, MAX_SYMBOLS);
      return ZERO;
    }
    break;
  
  case 7:
    // ECC bits that contain bit 7 are H3 and H4.
    if ((rc = check_ecc(present, missing, value, 3, bit[5])) != MISSING) {
      // We've fixed it using H1: return the missing bit
      return rc;
    } else if ((rc = check_ecc(present, missing, value, 4, bit[6])) != MISSING) {
      // We've fixed it using H2: return the missing bit
      return rc;
    } else {
      // This error is unfixable. return ZERO, but log an irrecoverable error
      data_error(bit, MAX_SYMBOLS);
      return ZERO;
    }
    break;
  
  default:
    break;
  }
}

// Receive a burst (pulse count and interval since last burst) and assemble
// into a byte using a state machine. Returns 0 if incomplete, and the byte 
// once a valid byte has been received.
byte process_burst(burst_t b) {
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

        case 5: // two and a half bits
          // presume we missed two bits, and have skipped to a '1'
          bit[count++] = MISSING;
          bit[count++] = MISSING;
          bit[count++] = ONE;
          break;

        case 6: // three full bits
          // presume we missed two bits, and have skipped to a '0'
          bit[count++] = MISSING;
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

        case 6: // three full bits
          // presume we missed two bits, and have skipped to a '1'
          bit[count++] = MISSING;
          bit[count++] = MISSING;
          bit[count++] = ONE;
          break;

        case 7: // three full bits and a half bit
          // presume we missed two bits, and have skipped to a '0'
          bit[count++] = MISSING;
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
    // for (int i=0; i<count; i++) {
    //   Serial.print(symbol_to_char(bit[i]));
    // }
    // Serial.print(' ');
    unsigned char c = 0;
    for (int i=7; i<count; i++) {
      if (bit[i] == MISSING) {
        bit[i] = error_correct(bit, i-7);
      }
      c = (c<<1) + bit[i];
    }
    if (c < 128 || mode == binary_mode) {
      Serial.write(c);
    } else if (c < 255) {
      Serial.print(Rom8_UTF8[c-128]);
    } else {
      Serial.print("\\x");
      Serial.print(c, HEX);
    }

    // Serial.println("");
    count = 0;
    return 'Y';
  } else {
    return 0;
  }
}

void test_ecc() {
  symbol_t h33[] = {START1, START2, START3, 
                  ZERO, ZERO, ZERO, ZERO, 
                  ZERO, ZERO, ONE, ONE, ZERO, ZERO, ONE, ONE};
  symbol_t haa[] = {START1, START2, START3, 
                  ZERO, ONE, ONE, ONE, 
                  ONE, ZERO, ONE, ZERO, ONE, ZERO, ONE, ZERO};
  for (int missing=8; missing<MAX_SYMBOLS; missing++) {
    Serial.println("h33");
    for (int i=0; i<MAX_SYMBOLS; i++) {
      Serial.print(symbol_to_char(h33[i]));
    }
    // inject an error
    h33[missing] = MISSING;
    h33[7] = MISSING;
    Serial.println("\nwith error");
    for (int i=0; i<MAX_SYMBOLS; i++) {
      Serial.print(symbol_to_char(h33[i]));
    }
    Serial.print('\n');
    // then repair it
    h33[7] = error_correct(h33, 0);
    h33[missing] = error_correct(h33, missing-7);
    Serial.println("\nerror corrected");
    for (int i=0; i<MAX_SYMBOLS; i++) {
      Serial.print(symbol_to_char(h33[i]));
    }
    Serial.print('\n');
  }
  for (int missing=8; missing<MAX_SYMBOLS; missing++) {

    Serial.println("haa");
    for (int i=0; i<MAX_SYMBOLS; i++) {
      Serial.print(symbol_to_char(haa[i]));
    }    h33[7] = MISSING;
    // inject an error
    haa[missing] = MISSING;
    haa[7] = MISSING;
    Serial.println("\nwith error");
    for (int i=0; i<MAX_SYMBOLS; i++) {
      Serial.print(symbol_to_char(haa[i]));
    }
    Serial.print('\n');
    // then repair it
    haa[7] = error_correct(haa, 0);
    haa[missing] = error_correct(haa, missing-7);
    Serial.println("\nerror corrected");
    for (int i=0; i<MAX_SYMBOLS; i++) {
      Serial.print(symbol_to_char(haa[i]));
    }
    Serial.print('\n');
  }
  
}

void loop() {
  // Unit test error correction
  // test_ecc();
  // while (true) {
  //   delay(1000);
  // }
  burst_t b;
  if (qread(&b)) {
    // Serial.print(b.pulses, DEC);
    // Serial.print(' ');
    // Serial.println(b.micros, DEC);
    process_burst(b);
  }
}


