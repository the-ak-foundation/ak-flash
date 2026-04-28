# AK - Flash
[<img src="https://github.com/the-ak-foundation/ak-base-kit-stm32l151/blob/main/hardware/images/ak-foundation-logo.png" width="240"/>](https://github.com/the-ak-foundation)

The application runs on the host PC or on the embedded computer like Raspberry Pi, helping to update firmware via Serial (UART) for microcontroller devices (MCU) using [AK Embedded Base Source](https://github.com/ak-embedded-software/ak-base-kit-stm32l151).

### Installation
```sh
cd ak-flash
make
sudo make install
```

### Example
```sh
ak-flash /dev/ttyUSB0 ak-base-kit-stm32l151-application.bin 0x08003000
```

### Reference
| Topic | Link |
| ------ | ------ |
| Blog & Tutorial | https://epcb.vn/blogs/ak-embedded-software |
| Where to buy KIT? | https://epcb.vn/products/ak-embedded-base-kit-lap-trinh-nhung-vi-dieu-khien-mcu |
