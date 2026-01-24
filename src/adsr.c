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

/* USI I2C Master Functions */
void i2c_init(void) {
  PORTB |= (1 << SDA_PIN) | (1 << SCL_PIN); // Pull-ups
  DDRB |= (1 << SCL_PIN);
  DDRB &= ~(1 << SDA_PIN);
}

uint8_t i2c_transfer(uint8_t data) {
  USIDR = data;
  USISR = (1 << USIOIF); // Clear interrupt flag
  while (!(USISR & (1 << USIOIF))) {
    USICR = (1 << USIWM1) | (1 << USICS1) | (1 << USICLK) | (1 << USITC);
  }
  return USIDR;
}

void i2c_start(void) {
  PORTB |= (1 << SCL_PIN);
  PORTB |= (1 << SDA_PIN);
  DDRB |= (1 << SDA_PIN);
  PORTB &= ~(1 << SDA_PIN);
  PORTB &= ~(1 << SCL_PIN);
}

void i2c_stop(void) {
  DDRB |= (1 << SDA_PIN);
  PORTB &= ~(1 << SDA_PIN);
  PORTB |= (1 << SCL_PIN);
  PORTB |= (1 << SDA_PIN);
}

void dac_write(uint16_t value) {
  i2c_start();
  i2c_transfer(DAC_ADDR << 1);
  i2c_transfer((value >> 8) & 0x0F); // Fast mode, top 4 bits
  i2c_transfer(value & 0xFF);        // Bottom 8 bits
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
  _delay_us(1); // Reduced settling delay
  uint16_t val0 = adc_read();

  // Read second channel (S0 = 1)
  PORTB |= (1 << ADDR_S0);
  _delay_us(1); // Reduced settling delay
  uint16_t val1 = adc_read();

  if (gate) {
    pot_S = val0;
    pot_R = val1;
  } else {
    pot_A = val0;
    pot_D = val1;
  }
}

int main(void) {
  // Pin setup
  DDRB |= (1 << ADDR_S0);
  DDRB &= ~(1 << GATE_IN);

  i2c_init();
  adc_init();

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
      uint16_t attack_inc = 4095 / (pot_A / 8 + 1); // Approximation
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
      uint16_t decay_dec = 4095 / (pot_D / 8 + 1);
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
      uint16_t release_dec = 4095 / (pot_R / 8 + 1);
      if (current_value <= release_dec) {
        current_value = 0;
        state = IDLE;
      } else {
        current_value -= release_dec;
      }
      break;
    }

    dac_write(current_value);
    // Artificial delay removed for maximum performance
  }

  return 0;
}
