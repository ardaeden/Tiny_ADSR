#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

/* Pin Definitions */
#define MUX_OUT_ADC PB3 // ADC3
#define ADDR_S0 PB4     // CD4051 S0
#define GATE_IN PB1     // Gate Input & CD4051 S1
#define SDA_PIN PB0     // USI SDA
#define SCL_PIN PB2     // USI SCL

/* ADSR Constants */
#define DAC_ADDR 0x60 // Default MCP4725 address (may vary)

typedef enum { IDLE, ATTACK, DECAY, SUSTAIN, RELEASE } ADSR_State;

volatile ADSR_State state = IDLE;
volatile uint16_t current_value = 0;
volatile uint16_t pot_A = 0, pot_D = 0, pot_S = 0, pot_R = 0;

/* Turbo Bit-Bang I2C - Adjusted for better stability */
#define I2C_DELAY 2 // Increased for communication reliability

static inline void sda_high() {
  DDRB &= ~(1 << SDA_PIN); // Input mode (effectively high due to pull-up)
  PORTB |= (1 << SDA_PIN); // Internal pull-up enable
}

static inline void sda_low() {
  DDRB |= (1 << SDA_PIN);   // Output mode
  PORTB &= ~(1 << SDA_PIN); // Drive low
}

static inline void scl_high() {
  DDRB &= ~(1 << SCL_PIN);
  PORTB |= (1 << SCL_PIN);
  // Optional: Wait for clock stretching
  while (!(PINB & (1 << SCL_PIN)))
    ;
}

static inline void scl_low() {
  DDRB |= (1 << SCL_PIN);
  PORTB &= ~(1 << SCL_PIN);
}

void i2c_init(void) {
  sda_high();
  scl_high();
}

void i2c_start(void) {
  sda_high();
  scl_high();
  _delay_us(I2C_DELAY);
  sda_low();
  _delay_us(I2C_DELAY);
  scl_low();
  _delay_us(I2C_DELAY);
}

void i2c_stop(void) {
  sda_low();
  _delay_us(I2C_DELAY);
  scl_high();
  _delay_us(I2C_DELAY);
  sda_high();
  _delay_us(I2C_DELAY);
}

uint8_t i2c_write(uint8_t data) {
  for (uint8_t i = 0; i < 8; i++) {
    if (data & 0x80)
      sda_high();
    else
      sda_low();
    _delay_us(I2C_DELAY);
    scl_high();
    _delay_us(I2C_DELAY);
    scl_low();
    data <<= 1;
  }

  // Read ACK
  sda_high();
  _delay_us(I2C_DELAY);
  scl_high();
  _delay_us(I2C_DELAY);
  uint8_t ack = !(PINB & (1 << SDA_PIN));
  scl_low();
  _delay_us(I2C_DELAY);

  return ack;
}

void dac_write(uint16_t value) {
  i2c_start();
  i2c_write(DAC_ADDR << 1);
  i2c_write((value >> 8) & 0x0F);
  i2c_write(value & 0xFF);
  i2c_stop();
}

/* ADC Functions */
void adc_init(void) {
  ADMUX = (1 << MUX1) | (1 << MUX0);                  // ADC3 (PB3)
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // Enable, Prescaler 64
}

uint16_t adc_read(void) {
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC))
    ;
  return ADC;
}

/* Application Logic */
void update_pots(uint8_t gate) {
  // Read first channel (S0 = 0)
  PORTB &= ~(1 << ADDR_S0);
  _delay_us(20); // More time for multiplexer to settle
  uint16_t val0 = adc_read();

  // Read second channel (S0 = 1)
  PORTB |= (1 << ADDR_S0);
  _delay_us(20); // More time for multiplexer to settle
  uint16_t val1 = adc_read();

  // Simple EMA Filter (prev * 7 + current) / 8
  if (gate) {
    pot_S = (uint16_t)(((uint32_t)pot_S * 7 + val0) >> 3);
    pot_R = (uint16_t)(((uint32_t)pot_R * 7 + val1) >> 3);
  } else {
    pot_A = (uint16_t)(((uint32_t)pot_A * 7 + val0) >> 3);
    pot_D = (uint16_t)(((uint32_t)pot_D * 7 + val1) >> 3);
  }
}

int main(void) {
  // Pin setup
  DDRB |= (1 << ADDR_S0);
  DDRB &= ~(1 << GATE_IN);

  i2c_init();
  adc_init();

  // Force DAC to 0V at startup and initialize pots
  dac_write(0);
  update_pots(0);
  update_pots(1);

  uint16_t target_sustain = 0;

  while (1) {
    uint8_t gate = (PINB & (1 << GATE_IN));
    update_pots(gate);

    // Simple ADSR state machine
    switch (state) {
    case IDLE:
      if (gate)
        state = ATTACK;
      current_value = 0;
      break;

    case ATTACK:
      if (!gate) {
        state = RELEASE;
        break;
      }
      // Map pot_A to increment (Fast A = high value, Slow A = low value)
      uint16_t attack_inc = 4095 / (pot_A * 4 + 1);
      if (current_value + attack_inc >= 4095) {
        current_value = 4095;
        state = DECAY;
      } else {
        current_value += attack_inc;
      }
      break;

    case DECAY:
      if (!gate) {
        state = RELEASE;
        break;
      }
      target_sustain = pot_S << 2; // ADC 10bit to DAC 12bit
      uint16_t decay_dec = 4095 / (pot_D * 4 + 1);
      if (current_value <= target_sustain + decay_dec) {
        current_value = target_sustain;
        state = SUSTAIN;
      } else {
        current_value -= decay_dec;
      }
      break;

    case SUSTAIN:
      if (!gate) {
        state = RELEASE;
        break;
      }
      current_value = pot_S << 2;
      break;

    case RELEASE:
      if (gate) {
        state = ATTACK;
        break;
      }
      uint16_t release_dec = 4095 / (pot_R * 4 + 1);
      if (current_value <= release_dec) {
        current_value = 0;
        state = IDLE;
      } else {
        current_value -= release_dec;
      }
      break;
    }

    dac_write(current_value);
    _delay_ms(1); // Small loop delay for stability and better timing control
  }

  return 0;
}
