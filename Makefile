# MCU settings
MCU = attiny85
F_CPU = 8000000UL
PORT = /dev/ttyACM0
BAUD = 19200
PROGRAMMER = arduino

# Compiler settings
CC = avr-gcc
OBJCOPY = avr-objcopy
CFLAGS = -Wall -Os -DF_CPU=$(F_CPU) -mmcu=$(MCU)

# Target name
TARGET = blink

# Source files
SRC = src/main.c

all: build

build: $(TARGET).hex

$(TARGET).elf: $(SRC)
	$(CC) $(CFLAGS) -o $@ $^

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

flash: build
	avrdude -p $(MCU) -c $(PROGRAMMER) -P $(PORT) -b $(BAUD) -U flash:w:$(TARGET).hex:i

clean:
	rm -f $(TARGET).elf $(TARGET).hex

.PHONY: all build flash clean
