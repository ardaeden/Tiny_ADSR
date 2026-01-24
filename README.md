# ATTiny85 Project (Pro Micro ISP)

Bu proje, Arduino Pro Micro'yu ISP programlayıcı olarak kullanarak ATTiny85 programlamak için hazırlandı.

## Dosya Yapısı
- `src/main.c`: Blink kodu.
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
