#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>
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

// Morphing and Speed settings
uint16_t morph_A = 512, morph_D = 512, morph_R = 512;
uint16_t speed_val = 256;

// EEPROM Addresses
uint16_t EEMEM ee_morph_A = 512;
uint16_t EEMEM ee_morph_D = 512;
uint16_t EEMEM ee_morph_R = 512;
uint16_t EEMEM ee_speed_val = 256;

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

/* Morphing Math */
uint16_t apply_morph(uint16_t val, uint16_t morph) {
  uint32_t lin = val;
  uint32_t expo = (uint32_t)val * val >> 12;
  uint32_t log_val = 4095 - ((uint32_t)(4095 - val) * (4095 - val) >> 12);

  if (morph < 512) {
    // Morph between EXPO and LIN
    return (uint16_t)((expo * (511 - morph) + lin * morph) >> 9);
  } else {
    // Morph between LIN and LOG
    uint16_t m = morph - 512;
    return (uint16_t)((lin * (511 - m) + log_val * m) >> 9);
  }
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

void config_mode(void) {
  // Signal entry with flash
  for (uint8_t i = 0; i < 3; i++) {
    dac_write(4095);
    _delay_ms(100);
    dac_write(0);
    _delay_ms(100);
  }

  // While GATE is held, adjust morphing using R pot as master
  // and Speed using S pot
  while (PINB & (1 << GATE_IN)) {
    update_pots(1); // Read S and R

    morph_A = pot_R;
    morph_D = pot_R;
    morph_R = pot_R;
    speed_val = pot_S + 10; // Range 10 to 1033 (approx 0.04x to 4x)

    _delay_ms(10);
  }

  // Save to EEPROM
  eeprom_update_word(&ee_morph_A, morph_A);
  eeprom_update_word(&ee_morph_D, morph_D);
  eeprom_update_word(&ee_morph_R, morph_R);
  eeprom_update_word(&ee_speed_val, speed_val);

  // Signal exit
  for (uint8_t i = 0; i < 2; i++) {
    dac_write(2048);
    _delay_ms(50);
    dac_write(0);
    _delay_ms(50);
  }
}

int main(void) {
  // Pin setup
  DDRB |= (1 << ADDR_S0);
  DDRB &= ~(1 << GATE_IN);

  i2c_init();
  adc_init();

  // Load from EEPROM
  morph_A = eeprom_read_word(&ee_morph_A);
  if (morph_A > 1023)
    morph_A = 512;
  morph_D = eeprom_read_word(&ee_morph_D);
  if (morph_D > 1023)
    morph_D = 512;
  morph_R = eeprom_read_word(&ee_morph_R);
  if (morph_R > 1023)
    morph_R = 512;
  speed_val = eeprom_read_word(&ee_speed_val);
  if (speed_val > 1100 || speed_val < 5)
    speed_val = 256; // Default to 1x speed

  // If GATE is held at boot, enter config mode
  if (PINB & (1 << GATE_IN)) {
    config_mode();
  }

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
      // Map pot_A to increment with speed scaling
      uint16_t attack_inc =
          ((uint32_t)4095 * speed_val) / ((uint32_t)256 * (pot_A + 1));
      if (attack_inc == 0)
        attack_inc = 1;

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
      uint16_t decay_dec =
          ((uint32_t)4095 * speed_val) / ((uint32_t)256 * (pot_D + 1));
      if (decay_dec == 0)
        decay_dec = 1;

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
      uint16_t release_dec =
          ((uint32_t)4095 * speed_val) / ((uint32_t)256 * (pot_R + 1));
      if (release_dec == 0)
        release_dec = 1;

      if (current_value <= release_dec) {
        current_value = 0;
        state = IDLE;
      } else {
        current_value -= release_dec;
      }
      break;
    }

    uint16_t morphed_value = apply_morph(current_value, morph_R);

    dac_write(morphed_value);
    _delay_ms(1); // Small loop delay for stability and better timing control
  }

  return 0;
}
