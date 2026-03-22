# mhi-ac-spi-signal-generator

This project is a test signal generator to test mhi-ac wifi modules. It replays a captured SPI frame (CLK and MOSI) in a loop. It is based on a frame from [this trace](https://github.com/hberntsen/mhi-ac-ctrl-esp32-c3/issues/15#issuecomment-2695634858) of my SRK25ZS-W. 

## Hardware

This was tested on a [mhi-ac-ctrl-esp32-c3 board](https://github.com/hberntsen/mhi-ac-ctrl-esp32-c3/blob/99ec16ffa53d411a996df8cfcd93d4b9c1add8b7/Hardware.md). Other ESP32 hardware will probably also work.

## Flashing

1. Make sure to set up the ESP-IDF v5.5.3 toolchain. See the [Espressif documentation](https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32c3/get-started/linux-macos-setup.html)
2. Run `idf.py set-target esp32c3`. 
3. Configure the CLK and MOSI pins in the `main.c` file.
3. Flash your board with `idf.py flash`
