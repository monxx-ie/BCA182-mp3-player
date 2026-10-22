# RT-Spark Personal MP3 Player

BCA182 Embedded Systems Programming, Laboratory Activity 2.

An MP3-style player on the **RT-Thread RT-Spark (STM32F407ZGT6)** board. It plays 8 songs from `song_def.h` through the board's ES8388 audio codec (3.5 mm jack) or the onboard buzzer.

- **Framework:** PlatformIO with the STM32Cube HAL (no Arduino)
- **RTOS:** FreeRTOS with cooperative scheduling

## Team

BCA182 Embedded Systems Programming, B187.1

| Member | Role |
|---|---|
| Cassandra Khan P. Mendoza | Firmware (`main.cpp`), README |
| Carl Hovan Chan | Hardware wiring and wiring diagram |
| Sean Marie Rabago | Testing, demo video, documentation photos |
| Shaqkobe Dos P. Tejada | System explanations and test trial documentation |

## Project overview

This project implements a small embedded music player on the RT-Spark development board. The user controls it with five push buttons and a potentiometer. The onboard LCD, an RGB status LED and the UART serial output show what the player is doing.

The system is designed to demonstrate the core ideas of embedded programming in a single application:

- reading digital input from buttons and debouncing it in software
- managing multiple tasks in FreeRTOS
- using timers for note timing and timeout logic
- controlling audio generation with the ES8388 codec and buzzer
- reading an analog signal from a potentiometer for volume control
- updating a graphical user interface on the ST7789 LCD

## Features

- **Song selection:** pick one of 8 songs with 3 buttons in binary (B2 = MSB, B4 = LSB).
- **Select and confirm:** press B1 to select, then press B1 again within **5 seconds** to confirm. If you don't, the player continues as before.
- **Play / pause:** the USER_BUTTON.
- **LCD (240x240 ST7789, FSMC):** shows the song name, the play/pause state, the confirmation message with a countdown, and the volume bar.
- **RGB LED:** **blue** = playing, **red** = paused, **green** = waiting for confirmation.
- **UART:** usage instructions and the song list are sent at 115200 baud.
- **Volume:** set with an external potentiometer.
- **Sleep mode:** the CPU sleeps in the FreeRTOS idle hook.

## How the system works

### Song selection flow

The user chooses a song by pressing three binary selector buttons. These inputs are treated as a 3-bit code, so the system can represent values from `000` to `111`, which correspond to songs 1 through 8.

The user holds the combination on B2–B4 and presses B1. The firmware reads the three buttons at that moment, and the LCD shows the selected song's name with a "CHANGE SONG?" message and a countdown.

### Confirmation step

A song selection is not immediately activated. Instead, the player waits for a confirmation input within a 5-second window. If the user presses the confirm button again during this time, the system accepts the song and starts playback. If the timer expires, the selection is ignored and the system continues using the previous song state.

This is important because it prevents accidental track switching when the buttons are pressed unintentionally or in quick succession.

### Playback state machine

The player uses a simple state-based design. The main states include:

- idle / previous selection
- song selected / awaiting confirm
- playing
- paused

The play/pause button toggles the output state without resetting the song selection, making the player behave like a small music device rather than a one-shot demo.

### Volume control

The volume is adjusted with a potentiometer connected to ADC1. The firmware periodically reads this analog value and updates the audio gain. The LCD also shows the current level so the user can see the effect of the control in real time.

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
- **Embedded design pattern:** the firmware uses hardware timers and RTOS scheduling together so that the system remains responsive while generating audio notes on time.

## Wiring

[Show Wiring Diagram](Documentation/Wiring.png)

[Show Live Wiring Photos](Documentation/live-wiring.md)

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

The repository currently uses `1` (onboard buzzer). This setting allows quick switching between the onboard buzzer and the external audio output path without changing the rest of the player logic.

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

## Testing and Demo

To verify that the MP3 player hardware and multi-threaded firmware work as expected, several real-time tests were performed on the board. These tests focused on the main user interactions: song selection, confirmation timeout, playback control, LCD and LED behavior, audio output, and serial logging.

### Demo Video

[Show Demo Video](https://drive.google.com/file/d/1TlXZdpgL6mWdoMecoKZXU907efR9ZTw8/view?usp=sharing)

### Feature test trials

#### Trial 1: Song selection and binary input validation
- **Objective:** verify that all 8 binary song codes can be selected using the three selector buttons.
- **Procedure:** for each code from `000` to `111`, hold the combination on B2–B4, press B1, and observe the LCD.
- **Expected result:** each code shows the correct song name on the LCD.
- **Result:** all 8 codes responded correctly, and the selected song updated on the display.
- **Status:** ✅ PASS

#### Trial 2: 5-second confirmation timeout
- **Objective:** confirm the timeout logic rejects a selection if the user does not confirm within 5 seconds.
- **Procedure:** change the selected song, then wait 5 seconds without pressing the confirm button.
- **Expected result:** the system returns to its previous state without switching the active track.
- **Result:** the original song remained active and the selection timed out as intended.
- **Status:** ✅ PASS

#### Trial 3: Confirm selection and playback start
- **Objective:** validate that a confirmed selection begins playback properly.
- **Procedure:** choose a song, press B1 to confirm, and observe the system response.
- **Expected result:** the song loads and playback begins.
- **Result:** the selected song started successfully and the status indicators updated accordingly.
- **Status:** ✅ PASS

#### Trial 4: Play / pause behavior
- **Objective:** ensure the play/pause control works in both active and paused states.
- **Procedure:** press the USER_BUTTON while the player is playing and while it is paused.
- **Expected result:** the system switches between Playing and Paused without error.
- **Result:** playback toggled smoothly and the LCD/LED state matched the action.
- **Status:** ✅ PASS

#### Trial 5: RGB LED status indication
- **Objective:** verify that LED colors correctly reflect the current state.
- **Procedure:** move through idle, selection, waiting-for-confirmation, playing, and paused states.
- **Expected result:** blue = playing, red = paused, green = waiting for confirmation.
- **Result:** each state produced the correct LED color quickly and consistently.
- **Status:** ✅ PASS

#### Trial 6: Volume control
- **Objective:** test the potentiometer-based volume adjustment.
- **Procedure:** rotate the potentiometer from minimum to maximum while a song is playing.
- **Expected result:** sound level changes smoothly and the volume display updates.
- **Result:** volume responded correctly across the full range with no instability.
- **Status:** ✅ PASS

#### Trial 7: UART output
- **Objective:** verify that the usage instructions and song confirmations are sent over UART.
- **Procedure:** open the serial monitor at 115200 baud, press RESET, then select and confirm a song.
- **Expected result:** the instructions and song list print at startup, and "Now playing: <song>" prints when a song is confirmed.
- **Result:** the instructions, song list and "Now playing" message appeared as expected.
- **Status:** ✅ PASS

### Feature Test Results

| Feature Tested | Test Procedure / Description | Expected Behavior | Status |
| :--- | :--- | :--- | :---: |
| **Song Select (`000`–`111`)** | Selected each of the 8 binary combinations (`000` to `111`) with B1. | LCD shows the corresponding song (Songs 1–8). | ✅ PASS |
| **5-Second Timeout** | Changed the song selection input and waited 5 seconds without pressing confirm. | Selection times out and reverts to the previous state/idle without switching tracks. | ✅ PASS |
| **Confirm Selection** | Selected a binary track code and pressed the Confirm button within 5 seconds. | System locks in the chosen track and starts/loads playback. | ✅ PASS |
| **Play / Pause** | Pressed the Play/Pause button during active playback and while paused. | Audio playback and system state smoothly toggle between Playing and Paused. | ✅ PASS |
| **LED Colors** | Observed the RGB/status LEDs across Idle, Playing, Paused, and Selection states. | Correct LED color lights up to match each system state. | ✅ PASS |
| **Volume Control** | Adjusted the volume input from minimum to maximum while a track was playing. | Audio output level scales smoothly and updates the current volume reading. | ✅ PASS |
| **UART Output** | Opened the serial monitor at 115200 baud, reset the board, then confirmed a song. | Instructions and song list print at startup; "Now playing: <song>" prints on confirm. | ✅ PASS |

## Practical notes and troubleshooting

- If the player does not respond, confirm that the correct firmware binary was copied to the RT-SPARK USB drive.
- If no sound is heard, verify the `USE_SPEAKER` setting in `src/main.cpp` and check the audio output path.
- If the LCD stays blank, press RESET and check that the new firmware was copied. If the colours look inverted, change `0x21` to `0x20` in `init_lcd()`.
- If a button does nothing, check that it straddles the breadboard's centre gap and that its other leg goes to GND.
- If the UART output does not show up, set the serial monitor to **115200 baud** and select the correct port, then press RESET.

## Conclusion

This MP3 player successfully combines embedded system design, timer-driven audio generation, LCD display control, and FreeRTOS scheduling in one practical project. It demonstrates how a small STM32-based board can implement a usable music player using simple physical inputs and state-based firmware logic.

The system also shows a common embedded engineering workflow: hardware interconnection, software task design, state management, debugging via UART, and testing under real operating conditions.
