# Tiny ADSR - ATTiny85 & MCP4725

Bu proje, **ATtiny85** mikrodenetleyicisi, **MCP4725 DAC** ve **CD4051 Multiplexer** kullanılarak geliştirilmiş düşük seviyeli (low-level) bir ADSR zarf üreteci (envelope generator) projesidir. Tek bir analog giriş üzerinden 4 farklı parametreyi (Attack, Decay, Sustain, Release) okuyabilmek için akıllıca bir multiplexing mantığı kullanır.

## Temel Özellikler
- **MCU:** ATTiny85 (8MHz Dahili Saat)
- **DAC:** MCP4725 (12-bit I2C DAC)
- **Multiplexer:** CD4051 (8-kanallı analog switch)
- **Girişler:** Gate (Dijital), 4x Potansiyometre (Analog via MUX)
- **Çıkışlar:** 0-5V Analog Envelope

---

## Donanım ve Pin Bağlantıları

### 1. ATTiny85 Pinout
| Pin | Adı | Görev | Bağlantı |
|:---:|:---|:---|:---|
| 1 | PB5 | RESET | Pro Micro Pin 10 (ISP) |
| 2 | PB3 | ADC3 | CD4051 Common Out/In |
| 3 | PB4 | S0 | CD4051 S0 (Adres 0) |
| 4 | GND | Ground | Şase / 0V |
| 5 | PB0 | SDA | MCP4725 SDA & Pro Micro Pin 16 (ISP) |
| 6 | PB1 | GATE | Gate Girişi & CD4051 S1 (Adres 1) |
| 7 | PB2 | SCL | MCP4725 SCL & Pro Micro Pin 15 (ISP) |
| 8 | VCC | VCC | +5V |

### 2. CD4051 Multiplexer Bağlantıları
Bu projede CD4051, 4 kanal kullanacak şekilde yapılandırılmıştır. **Gate** sinyali aynı zamanda adresleyici olarak kullanılarak pin tasarrufu sağlanmıştır.

- **VCC (Pin 16):** +5V
- **VEE/GND (Pin 7, 8):** Ground
- **INH (Pin 6):** Ground (Daima aktif)
- **S0 (Pin 11):** ATTiny PB4 (Pin 3)
- **S1 (Pin 10):** ATTiny PB1 (Pin 6 - Gate)
- **S2 (Pin 9):** Ground
- **Common (Pin 3):** ATTiny PB3 (Pin 2 - ADC)

**Kanal / Potansiyometre Eşleşmesi:**
- **Kanal 0 (Pin 13):** Attack Pot
- **Kanal 1 (Pin 14):** Decay Pot
- **Kanal 2 (Pin 15):** Sustain Pot
- **Kanal 3 (Pin 12):** Release Pot

### 3. MCP4725 DAC Bağlantıları
- **VCC / GND:** +5V / Ground
- **SDA:** ATTiny PB0 (Pin 5)
- **SCL:** ATTiny PB2 (Pin 7)
- **OUT:** Analog ADSR Çıkışı

---

## ADSR Mantığı ve Multiplexing
ATtiny85'in kısıtlı pin sayısı nedeniyle, potansiyometreleri okumak için **Gate** sinyali multiplexer'ın ikinci adres pini (S1) olarak kullanılmıştır:
- **Gate LOW (0):** ATTiny, Attack ve Decay potlarını okur.
- **Gate HIGH (1):** ATTiny, Sustain ve Release potlarını okur.

Bu sayede sadece 2 adres pini ve 1 analog giriş kullanılarak 4 potansiyometre sorunsuzca yönetilir.

---

## Derleme ve Yükleme

### 1. Derleme
Kodu derlemek için terminalden:
```bash
make
```

### 2. Yükleme (Flash)
ATTiny85'e yüklemek için (Pro Micro ISP olarak bağlıyken):
```bash
make flash
```
> **Not:** ISP olarak farklı bir cihaz veya port kullanıyorsanız `Makefile` içindeki `PORT` ve `PROGRAMMER` değişkenlerini güncelleyin.

**Credits:**
- **Lead Designer:** Arda Eden
- **Coder:** Antigravity (AI Coding Assistant by Google DeepMind)
