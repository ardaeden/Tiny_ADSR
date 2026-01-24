#include <avr/io.h>
#include <util/delay.h>

/* Pin Definitions */
#define SDA_PIN PB0
#define SCL_PIN PB2

/* Address */
#define DAC_ADDR 0x60

/* Turbo Bit-Bang I2C - Optimized for 400kHz (Fast Mode) @ 8MHz */
#define I2C_DELAY 1 // microseconds for ~400kHz

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

int main(void) {
  i2c_init();

  uint16_t val = 0;
  while (1) {
    dac_write(val);
    val = (val + 16) & 0x0FFF; // High-speed sawtooth ramp (0-4095)
    // No artificial delay - running at maximum I2C throughput
  }

  return 0;
}