# Tiny ADSR - ATTiny85 & MCP4725

Bu proje, ATtiny85 mikrodenetleyicisi ve MCP4725 DAC kullanarak düşük seviyeli (low-level) bir ADSR zarf üreteci (envelope generator) projesidir.


## Dosya Yapısı
- `src/adsr.c`: ADSR durum makinesi ve temel mantık.
- `src/main.c`: Test ve başlangıç kodları.
- `Makefile`: Derleme ve yükleme komutlarını içeren dosya.

## Kullanım

### 1. Derleme
Kodu derlemek için terminalden:
```bash
make
```

### 2. Yükleme (Flash)
ATTiny85'e yüklemek için (Pro Micro bağlıyken):
```bash
make flash
```
> **Not:** Eğer Pro Micro farklı bir porttaysa (örn: `/dev/ttyACM1`), Makefile içindeki `PORT` değişkenini güncelleyin veya şu şekilde çalıştırın:
> `make flash PORT=/dev/ttyACM1`

## Pin Bağlantıları (Pro Micro -> ATTiny85)
- **VCC** -> Pin 8 (VCC)
- **GND** -> Pin 4 (GND)
- **Pin 10 (SS)** -> Pin 1 (RESET)
- **Pin 14 (MISO)** -> Pin 6 (MISO/PB1)
- **Pin 15 (SCLK)** -> Pin 7 (SCK/PB2)
- **Pin 16 (MOSI)** -> Pin 5 (MOSI/PB0)

**Credits:**
- **Lead Designer:** Arda Eden
- **Coder:** Antigravity (AI Coding Assistant by Google DeepMind)
