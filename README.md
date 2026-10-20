# RT-Spark Personal MP3 Player

BCA182 Embedded Systems Programming, Laboratory Activity 2.

An MP3-style player on the **RT-Thread RT-Spark (STM32F407ZGT6)** board. It plays 8 songs from `song_def.h` through the board's ES8388 audio codec (3.5 mm jack) or the onboard buzzer.

- **Framework:** PlatformIO with the STM32Cube HAL (no Arduino)
- **RTOS:** FreeRTOS with cooperative scheduling

## Features

- **Song selection:** pick one of 8 songs with 3 buttons in binary (B2 = MSB, B4 = LSB).
- **Select and confirm:** press B1 to select, then press B1 again within **5 seconds** to confirm. If you don't, the player continues as before.
- **Play / pause:** the USER_BUTTON.
- **LCD (240x240 ST7789, FSMC):** shows the song name, the play/pause state, the confirmation message with a countdown, and the volume bar.
- **RGB LED:** **blue** = playing, **red** = paused, **green** = waiting for confirmation.
- **UART:** usage instructions and the song list are sent at 115200 baud.
- **Volume:** set with an external potentiometer.
- **Sleep mode:** the CPU sleeps in the FreeRTOS idle hook.

## Design

| Part | Role |
|---|---|
| `update_lcd_leds_thread()` | Redraws the LCD and drives the RGB LED |
| `polling_buttons()` | Debounces the 5 buttons; handles select, confirm and play/pause |
| `adjust_volume()` | Reads the potentiometer (ADC1) and sets the volume |
| `Ticker` (TIM6, every 5 ms) | Steps through the notes of the current song |
| `Timeout` (TIM7, one-shot 5 s) | Ends the confirmation window |
| LCD mutex | Gives the threads exclusive access to the LCD |

- **Threads:** all 3 run at the same priority, each in an infinite loop that waits with `vTaskDelay()`.
- **Ticker and Timeout:** mbed-style classes built on hardware timers.

## Wiring

All external parts connect to the 2x20 header.

| Part | Pin | Connection |
|---|---|---|
| B1 (select / confirm) | PG0 | pin → button → GND |
| B2 (song bit 2, MSB) | PG1 | pin → button → GND |
| B3 (song bit 1) | PG2 | pin → button → GND |
| B4 (song bit 0, LSB) | PG5 | pin → button → GND |
| USER_BUTTON (play / pause) | PG6 | pin → button → GND |
| Red LED | PA2 | 5V → 1 kΩ → LED → pin |
| Green LED | PA3 | 5V → 1 kΩ → LED → pin |
| Blue LED | PA0 | 5V → 1 kΩ → LED → pin |
| Potentiometer | PA1 | outer legs → 3.3V and GND, middle → PA1 |
| Speaker / headphones | 3.5 mm jack | plug in |

- **Buttons:** the internal pull-ups are enabled, so pressed = LOW.
- **LEDs:** the pins are open-drain and active-LOW.

Onboard parts used:

| Part | Pins |
|---|---|
| LCD | FSMC Bank1 NE3, RS = A18 (PD13), RESET = PD3, backlight = PF9 |
| ES8388 codec, I2C (addr 0x10) | SCL = PF1, SDA = PF0 |
| ES8388 codec, I2S3 | MCK = PC7, WS = PA15, CK = PB3, SD = PB5 (DMA1 Stream5) |
| Buzzer | PB0 (TIM3_CH3) |
| UART | USART1 TX = PA9, RX = PA10 |

## Audio output switch

In `src/main.cpp`:

```cpp
#define USE_SPEAKER 1   // 1 = onboard buzzer, 0 = 3.5 mm jack (speaker / headphones)
```

## Build and flash

1. Open the folder in VS Code with PlatformIO.
2. Click **Build** (✓).
3. Copy `.pio/build/black_f407zg/firmware.bin` onto the **RT-SPARK** USB drive, then press **RESET**.
4. Open the serial monitor at **115200** to see the instructions.

## Files

| File | Contents |
|---|---|
| `src/main.cpp` | Application, drivers, threads |
| `include/song.h`, `include/song_def.h` | Song data (provided) |
| `lib/FreeRTOS` | FreeRTOS kernel V11.1.0 and `FreeRTOSConfig.h` |
| `platformio.ini` | Board and framework settings |
