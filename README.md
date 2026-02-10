# Tiny ADSR - ATTiny85 & MCP4725

This project is a low-level ADSR envelope generator developed using an **ATtiny85** microcontroller, an **MCP4725 DAC**, and a **CD4051 Multiplexer**. It uses a clever multiplexing logic to read 4 different parameters (Attack, Decay, Sustain, Release) through a single analog input.

## Features
- **MCU:** ATTiny85 (8MHz Internal Clock)
- **DAC:** MCP4725 (12-bit I2C DAC)
- **Multiplexer:** CD4051 (8-channel analog switch)
- **Inputs:** Gate (Digital), 4x Potentiometers (Analog via MUX)
- **Outputs:** 0-5V Analog Envelope

---

## Hardware and Pin Connections

### 1. ATTiny85 Pinout
| Pin | Name | Function | Connection |
|:---:|:---|:---|:---|
| 1 | PB5 | RESET | Pro Micro Pin 10 (ISP) |
| 2 | PB3 | ADC3 | CD4051 Common Out/In |
| 3 | PB4 | S0 | CD4051 S0 (Address 0) |
| 4 | GND | Ground | Ground / 0V |
| 5 | PB0 | SDA | MCP4725 SDA & Pro Micro Pin 16 (ISP) |
| 6 | PB1 | GATE | Gate Input & CD4051 S1 (Address 1) |
| 7 | PB2 | SCL | MCP4725 SCL & Pro Micro Pin 15 (ISP) |
| 8 | VCC | VCC | +5V |

### 2. CD4051 Multiplexer Connections
In this project, the CD4051 is configured to use 4 channels. The **Gate** signal is used as an addresser to save pins.

- **VCC (Pin 16):** +5V
- **VEE/GND (Pin 7, 8):** Ground
- **INH (Pin 6):** Ground (Always active)
- **S0 (Pin 11):** ATTiny PB4 (Pin 3)
- **S1 (Pin 10):** ATTiny PB1 (Pin 6 - Gate)
- **S2 (Pin 9):** Ground
- **Common (Pin 3):** ATTiny PB3 (Pin 2 - ADC)

**Channel / Potentiometer Mapping:**
- **Channel 0 (Pin 13):** Attack Pot
- **Channel 1 (Pin 14):** Decay Pot
- **Channel 2 (Pin 15):** Sustain Pot
- **Channel 3 (Pin 12):** Release Pot

### 3. MCP4725 DAC Connections
- **VCC / GND:** +5V / Ground
- **SDA:** ATTiny PB0 (Pin 5)
- **SCL:** ATTiny PB2 (Pin 7)
- **OUT:** Analog ADSR Output

---

## ADSR Logic and Multiplexing
Due to the limited pin count of the ATtiny85, the **Gate** signal is used as the second address pin (S1) for the multiplexer:
- **Gate LOW (0):** ATTiny reads Attack and Decay pots.
- **Gate HIGH (1):** ATTiny reads Sustain and Release pots.

This allows for the seamless management of 4 potentiometers using only 2 address pins and 1 analog input.

---

## Compilation and Flashing

### 1. Compilation
To compile the code, run in the terminal:
```bash
make
```

### 2. Flashing
To flash the code to the ATTiny85 (with Pro Micro connected as ISP):
```bash
make flash
```
> **Note:** If you are using a different ISP device or port, update the `PORT` and `PROGRAMMER` variables in the `Makefile`.

**Credits:**
- **Lead Designer:** Arda Eden
- **Coder:** Antigravity (AI Coding Assistant by Google DeepMind)
