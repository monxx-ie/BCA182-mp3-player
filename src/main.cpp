/**
 * ============================================================================
 *  BCA182 Embedded Systems Programming - Laboratory Activity 2
 *  Personal MP3 Player  (RT-Thread RT-Spark / STM32F407ZGT6)
 *  EXTERNAL-PARTS VERSION: breadboard buttons, potentiometer and RGB LED,
 *  songs played through the 3.5 mm headphone jack (ES8388 codec).
 *
 *  Framework : PlatformIO + STM32Cube HAL (framework = stm32cube, NO Arduino)
 *  RTOS      : FreeRTOS (cooperative scheduling: configUSE_PREEMPTION = 0)
 *
 *  Threads (exactly 3, each an infinite loop that blocks with vTaskDelay):
 *    1. update_lcd_leds_thread() - redraws the LCD + drives the RGB LED
 *    2. polling_buttons()        - polls/debounces the 5 push buttons
 *    3. adjust_volume()          - reads the potentiometer (ADC) -> volume
 *
 *  Timing objects (hardware-timer interrupts, same idea as mbed's API):
 *    Ticker  (TIM6) - fires every 5 ms and steps through the song's notes
 *    Timeout (TIM7) - one-shot 5 s window to confirm a new song
 *
 *  Pin map (all on the RT-Spark 2x20 header, except the LCD/audio):
 *    LCD  : ST7789 240x240, 8080 8-bit via FSMC Bank1 NE3 (PG10),
 *           RS/DC = FSMC_A18 (PD13), D0-D7 = PD14 PD15 PD0 PD1 PE7-PE10,
 *           NOE = PD4, NWE = PD5, RESET = PD3, BACKLIGHT = PF9
 *    Buttons (pull-up, pressed = LOW):
 *           B1 = PG0 (select / confirm)       B2 = PG1 (song bit 2, MSB)
 *           B3 = PG2 (song bit 1)             B4 = PG5 (song bit 0, LSB)
 *           USER_BUTTON = PG6 (play / pause)
 *    RGB LED (active LOW: 3.3V -> 330R -> LED -> pin):
 *           RED = PA2, GREEN = PA3, BLUE = PA0
 *    Potentiometer wiper : PA1 (ADC1_IN1), ends to 3.3V and GND
 *    Headphone jack      : ES8388 audio codec (I2C addr 0x10 on PF1/PF0)
 *                          fed by I2S3: MCK = PC7, WS = PA15, CK = PB3,
 *                          SD = PB5, DMA1 Stream5 (sine-wave notes, 16 kHz)
 *    Buzzer (PWM)        : PB0 (TIM3_CH3) - used when USE_SPEAKER = 1
 *    UART instructions   : USART1 TX = PA9, RX = PA10 (115200 8N1)
 * ============================================================================
 */

#include "stm32f4xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <cstdint>
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <initializer_list>
#include <cmath>

#include "song.h"
// song_def.h defines short macros (No, Do, Re, Mi, b0..b4 ...), so it is
// included LAST and no identifier below uses those names.
// NHD_0216HZ.h is NOT included: it is mbed-only and its ENABLE macro clashes
// with the HAL's ENABLE. Its API is re-implemented below for the ST7789.
#include "song_def.h"

/* ============================================================================
 *  CONFIGURATION
 * ==========================================================================*/

// ---- Buttons (pull-up, pressed = LOW) -------------------------------------
#define BTN_PORT            GPIOG
#define BTN1_PIN            GPIO_PIN_0      // select / confirm
#define BTN2_PIN            GPIO_PIN_1      // song index bit 2 (MSB)
#define BTN3_PIN            GPIO_PIN_2      // song index bit 1
#define BTN4_PIN            GPIO_PIN_5      // song index bit 0 (LSB)
#define USER_BTN_PIN        GPIO_PIN_6      // play / pause

// ---- RGB LED (active LOW, open drain) ------------------------------------
#define LED_PORT            GPIOA
#define LED_R_PIN           GPIO_PIN_2
#define LED_G_PIN           GPIO_PIN_3
#define LED_B_PIN           GPIO_PIN_0

// ---- Potentiometer --------------------------------------------------------
#define POT_PORT            GPIOA
#define POT_PIN             GPIO_PIN_1
#define POT_ADC_CHANNEL     ADC_CHANNEL_1

// ---- Software I2C to the ES8388 codec (onboard) ---------------------------
#define I2C_PORT            GPIOF
#define I2C_SCL_PIN         GPIO_PIN_1
#define I2C_SDA_PIN         GPIO_PIN_0

// ---- Headphone jack (ES8388 codec + I2S3) ----------------------------------
#define AUDIO_FS            16000U          // sample rate (Hz)
#define AUDIO_BUF_FRAMES    256             // stereo frames in the DMA ring buffer
// ---- Output select ---------------------------------------------------------
//   1 = onboard buzzer on the RT-Spark (PB0)
//   0 = IEMs / headphones / speaker on the 3.5 mm jack
#define USE_SPEAKER         1

#define AUDIO_MAX_AMP       6000            // jack: peak sample at 100 % volume (of 32767)
#define JACK_VOLUME         0x18            // ES8388 LOUT1/ROUT1: 0x00 = -45 dB .. 0x21 = +4.5 dB
#define ES8388_ADDR         0x10

// ---- Onboard buzzer (PWM) -------------------------------------------------
#define SPK_PORT            GPIOB
#define SPK_PIN             GPIO_PIN_0
#define SPK_AF              GPIO_AF2_TIM3
#define SPK_TIM_CHANNEL     TIM_CHANNEL_3

// ---- LCD control pins -----------------------------------------------------
#define LCD_RST_PORT        GPIOD
#define LCD_RST_PIN         GPIO_PIN_3
#define LCD_BL_PORT         GPIOF
#define LCD_BL_PIN          GPIO_PIN_9

// FSMC Bank1 / NE3 starts at 0x68000000. A18 drives the LCD's RS (D/C) line:
//   A18 = 0 -> command register, A18 = 1 -> data/GRAM.
#define LCD_CMD_REG         (*(volatile uint8_t *)(0x68000000UL))
#define LCD_DATA_REG        (*(volatile uint8_t *)(0x68000000UL | (1UL << 18)))

// ---- Player behaviour -----------------------------------------------------
#define NUM_SONGS           8         // 3 binary buttons -> 8 songs
#define TICKER_MS           5         // Ticker period (note resolution)
#define CONFIRM_WINDOW_S    5.0f      // Timeout length for confirmation
#define CONFIRM_WINDOW_MS   5000U
// note duration [ms] = beat * tempo * TEMPO_SCALE_MS
// (e.g. Fur Elise eighth note: 0.125 * 0.18 * 10000 = 225 ms).
// Increase to slow every song down, decrease to speed it up.
#define TEMPO_SCALE_MS      10000.0f

// ---- Button bit masks -----------------------------------------------------
#define MASK_BTN1           (1U << 0)
#define MASK_BTN2           (1U << 1)
#define MASK_BTN3           (1U << 2)
#define MASK_BTN4           (1U << 3)
#define MASK_USER           (1U << 4)

// ---- RGB565 colours -------------------------------------------------------
#define COLOR_BLACK         0x0000
#define COLOR_WHITE         0xFFFF
#define COLOR_RED           0xF800
#define COLOR_GREEN         0x07E0
#define COLOR_BLUE          0x001F
#define COLOR_CYAN          0x07FF
#define COLOR_YELLOW        0xFFE0
#define COLOR_GRAY          0x8410

/* ============================================================================
 *  PERIPHERAL HANDLES
 * ==========================================================================*/
static UART_HandleTypeDef huart1;
static ADC_HandleTypeDef  hadc1;
static TIM_HandleTypeDef  htim3;      // buzzer PWM
static SRAM_HandleTypeDef hsram3;     // FSMC -> LCD
static I2S_HandleTypeDef  hi2s3;      // I2S3 -> ES8388 (headphone jack)
static DMA_HandleTypeDef  hdma_i2s3_tx;

static void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}

/* ============================================================================
 *  5x7 FONT (ASCII 0x20..0x7E), one byte per column, LSB = top pixel
 * ==========================================================================*/
static const uint8_t kFont5x7[95][5] = {
    {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5F,0x00,0x00},{0x00,0x07,0x00,0x07,0x00},
    {0x14,0x7F,0x14,0x7F,0x14},{0x24,0x2A,0x7F,0x2A,0x12},{0x23,0x13,0x08,0x64,0x62},
    {0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},{0x00,0x1C,0x22,0x41,0x00},
    {0x00,0x41,0x22,0x1C,0x00},{0x08,0x2A,0x1C,0x2A,0x08},{0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},{0x00,0x60,0x60,0x00,0x00},
    {0x20,0x10,0x08,0x04,0x02},{0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},{0x18,0x14,0x12,0x7F,0x10},
    {0x27,0x45,0x45,0x45,0x39},{0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},{0x00,0x36,0x36,0x00,0x00},
    {0x00,0x56,0x36,0x00,0x00},{0x00,0x08,0x14,0x22,0x41},{0x14,0x14,0x14,0x14,0x14},
    {0x41,0x22,0x14,0x08,0x00},{0x02,0x01,0x51,0x09,0x06},{0x32,0x49,0x79,0x41,0x3E},
    {0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x01,0x01},
    {0x3E,0x41,0x41,0x51,0x32},{0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},
    {0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},{0x7F,0x40,0x40,0x40,0x40},
    {0x7F,0x02,0x04,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},
    {0x1F,0x20,0x40,0x20,0x1F},{0x7F,0x20,0x18,0x20,0x7F},{0x63,0x14,0x08,0x14,0x63},
    {0x03,0x04,0x78,0x04,0x03},{0x61,0x51,0x49,0x45,0x43},{0x00,0x00,0x7F,0x41,0x41},
    {0x02,0x04,0x08,0x10,0x20},{0x41,0x41,0x7F,0x00,0x00},{0x04,0x02,0x01,0x02,0x04},
    {0x40,0x40,0x40,0x40,0x40},{0x00,0x01,0x02,0x04,0x00},{0x20,0x54,0x54,0x54,0x78},
    {0x7F,0x48,0x44,0x44,0x38},{0x38,0x44,0x44,0x44,0x20},{0x38,0x44,0x44,0x48,0x7F},
    {0x38,0x54,0x54,0x54,0x18},{0x08,0x7E,0x09,0x01,0x02},{0x08,0x14,0x54,0x54,0x3C},
    {0x7F,0x08,0x04,0x04,0x78},{0x00,0x44,0x7D,0x40,0x00},{0x20,0x40,0x44,0x3D,0x00},
    {0x00,0x7F,0x10,0x28,0x44},{0x00,0x41,0x7F,0x40,0x00},{0x7C,0x04,0x18,0x04,0x78},
    {0x7C,0x08,0x04,0x04,0x78},{0x38,0x44,0x44,0x44,0x38},{0x7C,0x14,0x14,0x14,0x08},
    {0x08,0x14,0x14,0x18,0x7C},{0x7C,0x08,0x04,0x04,0x08},{0x48,0x54,0x54,0x54,0x20},
    {0x04,0x3F,0x44,0x40,0x20},{0x3C,0x40,0x40,0x20,0x7C},{0x1C,0x20,0x40,0x20,0x1C},
    {0x3C,0x40,0x30,0x40,0x3C},{0x44,0x28,0x10,0x28,0x44},{0x0C,0x50,0x50,0x50,0x3C},
    {0x44,0x64,0x54,0x4C,0x44},{0x00,0x08,0x36,0x41,0x00},{0x00,0x00,0x7F,0x00,0x00},
    {0x00,0x41,0x36,0x08,0x00},{0x08,0x08,0x2A,0x1C,0x08},
};

/* ============================================================================
 *  LCD DRIVER - NHD_0216HZ API (init_lcd, write_cmd, write_data, printf,
 *  set_cursor, clr_lcd) re-implemented for the on-board ST7789 over FSMC.
 *  Text is drawn on a character grid, like the original 16x2 LCD.
 * ==========================================================================*/
class LCD_ST7789 {
public:
    static constexpr uint16_t WIDTH  = 240;
    static constexpr uint16_t HEIGHT = 240;
    static constexpr uint8_t  SCALE  = 2;               // 5x7 font scaled x2
    static constexpr uint8_t  CELL_W = 6 * SCALE;       // 12 px per char
    static constexpr uint8_t  CELL_H = 8 * SCALE;       // 16 px per line
    static constexpr uint8_t  COLS   = WIDTH / CELL_W;  // 20 columns
    static constexpr uint8_t  ROWS   = HEIGHT / CELL_H; // 15 rows

    void init_lcd(void)
    {
        // Hardware reset
        HAL_GPIO_WritePin(LCD_BL_PORT, LCD_BL_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_RESET);
        HAL_Delay(20);
        HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_SET);
        HAL_Delay(120);

        write_cmd(0x11);                       // SLPOUT
        HAL_Delay(120);
        send(0x36, {0x00});            // MADCTL: normal
        send(0x3A, {0x05});            // COLMOD: RGB565
        send(0xB2, {0x0C,0x0C,0x00,0x33,0x33}); // porch
        send(0xB7, {0x35});            // gate control
        send(0xBB, {0x19});            // VCOM
        send(0xC0, {0x2C});            // LCM control
        send(0xC2, {0x01});            // VDV/VRH enable
        send(0xC3, {0x12});            // VRH
        send(0xC4, {0x20});            // VDV
        send(0xC6, {0x0F});            // 60 Hz frame rate
        send(0xD0, {0xA4,0xA1});       // power control
        send(0xE0, {0xD0,0x04,0x0D,0x11,0x13,0x2B,0x3F,
                                     0x54,0x4C,0x18,0x0D,0x0B,0x1F,0x23});
        send(0xE1, {0xD0,0x04,0x0C,0x11,0x13,0x2C,0x3F,
                                     0x44,0x51,0x2F,0x1F,0x1F,0x20,0x23});
        write_cmd(0x21);                       // INVON (IPS panel) - use 0x20 if colours look inverted
        write_cmd(0x29);                       // DISPON
        HAL_Delay(20);

        HAL_GPIO_WritePin(LCD_BL_PORT, LCD_BL_PIN, GPIO_PIN_SET);  // backlight ON
        set_color(COLOR_WHITE, COLOR_BLACK);
        set_cursor(0, 0);
    }

    void write_cmd(uint8_t cmd)  { LCD_CMD_REG  = cmd;  }
    void write_byte(uint8_t dat) { LCD_DATA_REG = dat;  }

    void set_color(uint16_t fg, uint16_t bg) { fg_ = fg; bg_ = bg; }

    void set_cursor(int column, int row) { col_ = column; row_ = row; }

    void clr_lcd(void)
    {
        fill_rect(0, 0, WIDTH, HEIGHT, bg_);
        set_cursor(0, 0);
    }

    void fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
    {
        if (w == 0 || h == 0) return;
        set_window(x, y, x + w - 1, y + h - 1);
        for (uint32_t i = 0; i < (uint32_t)w * h; ++i) write_pixel(color);
    }

    // Draw one character at the cursor and advance (clips at line end)
    void write_data(char c)
    {
        if (col_ >= COLS || row_ >= ROWS) return;
        if (c < 0x20 || c > 0x7E) c = '?';
        const uint8_t *glyph = kFont5x7[c - 0x20];
        const uint16_t x0 = col_ * CELL_W;
        const uint16_t y0 = row_ * CELL_H;

        set_window(x0, y0, x0 + CELL_W - 1, y0 + CELL_H - 1);
        for (uint8_t py = 0; py < CELL_H; ++py) {
            const uint8_t fy = py / SCALE;
            for (uint8_t px = 0; px < CELL_W; ++px) {
                const uint8_t fx = px / SCALE;
                const bool on = (fx < 5) && (fy < 7) && ((glyph[fx] >> fy) & 0x01);
                write_pixel(on ? fg_ : bg_);
            }
        }
        ++col_;
    }

    // printf-style output, max one line (COLS chars), like NHD_0216HZ::printf
    void printf(const char *format, ...)
    {
        char buffer[COLS + 1];
        va_list v;
        va_start(v, format);
        vsnprintf(buffer, sizeof(buffer), format, v);
        va_end(v);
        for (const char *b = buffer; *b && col_ < COLS; ++b) write_data(*b);
    }

    // Print a full line padded with spaces (overwrites old text, no flicker)
    void print_line(int row, uint16_t fg, const char *format, ...)
    {
        char buffer[COLS + 1];
        va_list v;
        va_start(v, format);
        vsnprintf(buffer, sizeof(buffer), format, v);
        va_end(v);
        size_t len = strlen(buffer);
        while (len < COLS) buffer[len++] = ' ';
        buffer[COLS] = '\0';
        set_color(fg, COLOR_BLACK);
        set_cursor(0, row);
        for (const char *b = buffer; *b; ++b) write_data(*b);
    }

private:
    void send(uint8_t cmd, std::initializer_list<uint8_t> data)
    {
        write_cmd(cmd);
        for (uint8_t d : data) write_byte(d);
    }

    void set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
    {
        write_cmd(0x2A);                                  // CASET
        write_byte(x0 >> 8); write_byte(x0 & 0xFF);
        write_byte(x1 >> 8); write_byte(x1 & 0xFF);
        write_cmd(0x2B);                                  // RASET
        write_byte(y0 >> 8); write_byte(y0 & 0xFF);
        write_byte(y1 >> 8); write_byte(y1 & 0xFF);
        write_cmd(0x2C);                                  // RAMWR
    }

    void write_pixel(uint16_t color)
    {
        write_byte(color >> 8);
        write_byte(color & 0xFF);
    }

    int col_ = 0, row_ = 0;
    uint16_t fg_ = COLOR_WHITE, bg_ = COLOR_BLACK;
};

static LCD_ST7789 lcd;

/* ============================================================================
 *  TICKER / TIMEOUT  (mbed-style API on basic timers TIM6 / TIM7)
 *  Ticker  : calls the function repeatedly every <seconds>
 *  Timeout : calls the function ONCE after <seconds> (one-pulse mode)
 *  Timer clock = 84 MHz / 8400 = 10 kHz  ->  0.1 ms resolution, max 6.5 s
 * ==========================================================================*/
typedef void (*Callback)(void);

class HwTimer {
public:
    HwTimer(TIM_TypeDef *instance, IRQn_Type irq, bool one_shot)
        : instance_(instance), irq_(irq), one_shot_(one_shot) {}

    void attach(Callback cb, float seconds)
    {
        detach();
        cb_ = cb;

        uint32_t ticks = (uint32_t)(seconds * 10000.0f + 0.5f);
        if (ticks < 1) ticks = 1;
        if (ticks > 65536) ticks = 65536;

        htim_.Instance               = instance_;
        htim_.Init.Prescaler         = 8400 - 1;
        htim_.Init.CounterMode       = TIM_COUNTERMODE_UP;
        htim_.Init.Period            = ticks - 1;
        htim_.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
        htim_.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
        if (HAL_TIM_Base_Init(&htim_) != HAL_OK) Error_Handler();

        if (one_shot_) htim_.Instance->CR1 |=  TIM_CR1_OPM;   // stop after 1 update
        else           htim_.Instance->CR1 &= ~TIM_CR1_OPM;

        __HAL_TIM_SET_COUNTER(&htim_, 0);
        __HAL_TIM_CLEAR_FLAG(&htim_, TIM_FLAG_UPDATE);        // drop UG flag from init

        // Priority 6 >= configMAX_SYSCALL_INTERRUPT_PRIORITY (5):
        // masked by taskENTER_CRITICAL(), so shared variables stay consistent.
        HAL_NVIC_SetPriority(irq_, 6, 0);
        HAL_NVIC_EnableIRQ(irq_);
        HAL_TIM_Base_Start_IT(&htim_);
    }

    void detach(void)
    {
        if (htim_.Instance != nullptr) HAL_TIM_Base_Stop_IT(&htim_);
        cb_ = nullptr;
    }

    void irq_handler(void)
    {
        if (__HAL_TIM_GET_FLAG(&htim_, TIM_FLAG_UPDATE) &&
            __HAL_TIM_GET_IT_SOURCE(&htim_, TIM_IT_UPDATE)) {
            __HAL_TIM_CLEAR_IT(&htim_, TIM_IT_UPDATE);
            Callback c = cb_;
            if (one_shot_) detach();
            if (c != nullptr) c();
        }
    }

private:
    TIM_TypeDef        *instance_;
    IRQn_Type           irq_;
    bool                one_shot_;
    TIM_HandleTypeDef   htim_{};
    volatile Callback   cb_ = nullptr;
};

class Ticker  : public HwTimer { public: Ticker()  : HwTimer(TIM6, TIM6_DAC_IRQn, false) {} };
class Timeout : public HwTimer { public: Timeout() : HwTimer(TIM7, TIM7_IRQn,     true)  {} };

static Ticker  music_ticker;     // plays the notes
static Timeout confirm_timeout;  // 5-second confirmation window

extern "C" void TIM6_DAC_IRQHandler(void) { music_ticker.irq_handler(); }
extern "C" void TIM7_IRQHandler(void)     { confirm_timeout.irq_handler(); }

/* ============================================================================
 *  SHARED STATE (written by threads and timer ISRs)
 * ==========================================================================*/
struct SongSlot {
    Song    *song;
    uint16_t length;   // safe number of notes to play
};
static SongSlot g_songs[NUM_SONGS];

static SemaphoreHandle_t g_lcd_mutex = nullptr;   // exclusive LCD access

static volatile uint8_t  g_song_idx       = 0;      // song being played
static volatile uint8_t  g_pending_idx    = 0;      // song waiting for confirmation
static volatile bool     g_playing        = false;  // play / pause
static volatile bool     g_confirming     = false;  // inside the 5 s window
static volatile uint32_t g_confirm_start  = 0;      // FreeRTOS tick at selection
static volatile uint8_t  g_volume         = 50;     // 0..100 %
static volatile bool     g_lcd_dirty      = true;   // LCD needs a redraw

// Note-player state (used by the Ticker ISR)
static volatile uint16_t g_note_idx       = 0;
static volatile uint32_t g_note_ticks     = 0;      // ticks left in current note
static volatile uint32_t g_gap_ticks      = 0;      // silent ticks at note end
static volatile float    g_note_period_ms = 0.0f;   // 0 = rest ("No")

#define ARR_COUNT(a) (sizeof(a) / sizeof((a)[0]))

// Never play past the end of either array, even if a .length is wrong
static uint16_t safe_len(int declared, size_t n_notes, size_t n_beats)
{
    size_t n = (n_notes < n_beats) ? n_notes : n_beats;
    if (declared > 0 && (size_t)declared < n) n = (size_t)declared;
    return (uint16_t)n;
}

static void init_song_table(void)
{
    g_songs[0] = { &FUR_ELISE,          safe_len(FUR_ELISE.length,          ARR_COUNT(Fur_Elise_note),          ARR_COUNT(Fur_Elise_beat)) };
    g_songs[1] = { &CANNON_IN_D,        safe_len(CANNON_IN_D.length,        ARR_COUNT(Canon_In_D_note),         ARR_COUNT(Canon_In_D_beat)) };
    g_songs[2] = { &MINUET_IN_G_MAJOR,  safe_len(MINUET_IN_G_MAJOR.length,  ARR_COUNT(Minuet_In_G_major_note),  ARR_COUNT(Minuet_In_G_major_beat)) };
    g_songs[3] = { &TURKISH_MARCH,      safe_len(TURKISH_MARCH.length,      ARR_COUNT(Turkish_March_note),      ARR_COUNT(Turkish_March_beat)) };
    g_songs[4] = { &NOCTRUNE_IN_E_FLAT, safe_len(NOCTRUNE_IN_E_FLAT.length, ARR_COUNT(Nocturne_in_E_flat_note), ARR_COUNT(Nocturne_in_E_flat_beat)) };
    g_songs[5] = { &WALTZ_NO2,          safe_len(WALTZ_NO2.length,          ARR_COUNT(Waltz_No2_note),          ARR_COUNT(Waltz_No2_beat)) };
    g_songs[6] = { &NOCTRUNE_IN_C_SHARP_MAJOR,
                   safe_len(NOCTRUNE_IN_C_SHARP_MAJOR.length, ARR_COUNT(Nocturne_in_C_sharp_minor_note), ARR_COUNT(Nocturne_in_C_sharp_minor_beat)) };
    g_songs[7] = { &SYMPHONY_NO40,      safe_len(SYMPHONY_NO40.length,      ARR_COUNT(Symphony_No40_note),      ARR_COUNT(Symphony_No40_beat)) };
}

/* ============================================================================
 *  SPEAKER (PWM): frequency = 1 / note period, volume = duty cycle (0..50 %)
 *  song_def.h stores each note as its PERIOD in ms (Do = 3.822 ms = 261.6 Hz)
 * ==========================================================================*/
static void speaker_set(float period_ms, uint8_t volume)
{
    if (period_ms <= 0.0f || volume == 0) {             // rest or muted
        __HAL_TIM_SET_COMPARE(&htim3, SPK_TIM_CHANNEL, 0);
        return;
    }
    const uint32_t period_us = (uint32_t)(period_ms * 1000.0f + 0.5f); // 1 MHz timer
    __HAL_TIM_SET_AUTORELOAD(&htim3, period_us - 1);
    __HAL_TIM_SET_COMPARE(&htim3, SPK_TIM_CHANNEL, (period_us * volume) / 200U);
}

/* ============================================================================
 *  HEADPHONE AUDIO: a sine-wave oscillator streamed to the ES8388 by I2S3+DMA.
 *  The DMA ring buffer is refilled half at a time from its interrupts.
 * ==========================================================================*/
static int16_t           g_audio_buf[AUDIO_BUF_FRAMES * 2];   // L,R,L,R...
static int16_t           g_sine[256];
static volatile uint32_t g_tone_inc = 0;      // phase step per sample
static volatile int32_t  g_tone_amp = 0;      // target amplitude (0 = silent)
static uint32_t          g_phase    = 0;
static int32_t           g_amp_now  = 0;      // ramped amplitude (no clicks)

static void audio_set_tone(float period_ms, uint8_t volume)
{
    if (period_ms <= 0.0f || volume == 0) { g_tone_amp = 0; return; }
    const float freq = 1000.0f / period_ms;                       // Hz
    g_tone_inc = (uint32_t)(freq * (4294967296.0f / (float)AUDIO_FS));
    g_tone_amp = (int32_t)volume * AUDIO_MAX_AMP / 100;
}

static void audio_fill(int16_t *dst, uint32_t frames)
{
    const uint32_t inc    = g_tone_inc;
    const int32_t  target = g_tone_amp;
    for (uint32_t i = 0; i < frames; ++i) {
        if (g_amp_now < target)      { g_amp_now += 64; if (g_amp_now > target) g_amp_now = target; }
        else if (g_amp_now > target) { g_amp_now -= 64; if (g_amp_now < target) g_amp_now = target; }
        const int16_t v = (int16_t)((g_sine[g_phase >> 24] * g_amp_now) >> 15);
        g_phase += inc;
        dst[2 * i]     = v;    // left
        dst[2 * i + 1] = v;    // right
    }
}

extern "C" void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *h)
{
    (void)h;
    audio_fill(&g_audio_buf[0], AUDIO_BUF_FRAMES / 2);
}

extern "C" void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *h)
{
    (void)h;
    audio_fill(&g_audio_buf[AUDIO_BUF_FRAMES], AUDIO_BUF_FRAMES / 2);
}

extern "C" void DMA1_Stream5_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_i2s3_tx); }

/* ============================================================================
 *  TICKER CALLBACK (ISR context, every TICKER_MS) - plays the current song
 * ==========================================================================*/
static void play_tick(void)
{
    if (!g_playing) {                       // paused -> silence
        speaker_set(0.0f, 0);
        audio_set_tone(0.0f, 0);
        return;
    }

    if (g_note_ticks == 0) {                // load the next note
        const SongSlot &slot = g_songs[g_song_idx];
        if (g_note_idx >= slot.length) g_note_idx = 0;   // end -> replay song

        const float dur_ms = slot.song->beat[g_note_idx] * slot.song->tempo * TEMPO_SCALE_MS;
        uint32_t ticks = (uint32_t)(dur_ms / TICKER_MS);
        if (ticks < 2) ticks = 2;

        g_note_ticks     = ticks;
        g_gap_ticks      = (ticks / 8) ? (ticks / 8) : 1;   // short gap between notes
        g_note_period_ms = slot.song->note[g_note_idx];     // 0 = rest
        ++g_note_idx;
    }

    const bool  sounding = (g_note_ticks > g_gap_ticks);
    const float period   = sounding ? g_note_period_ms : 0.0f;
    audio_set_tone(USE_SPEAKER ? 0.0f : period, g_volume);             // 3.5 mm jack
    speaker_set(USE_SPEAKER ? period : 0.0f, g_volume);                // onboard buzzer
    --g_note_ticks;
}

/* ============================================================================
 *  TIMEOUT CALLBACK (ISR context) - 5 s passed without confirmation
 * ==========================================================================*/
static void on_confirm_timeout(void)
{
    g_confirming = false;       // give up the change, keep current song state
    g_lcd_dirty  = true;
}

/* ============================================================================
 *  HELPERS
 * ==========================================================================*/
static void uart_puts(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)s, (uint16_t)strlen(s), 1000);
}

/* ---------------------------------------------------------------------------
 *  Cycle counter (DWT) - precise busy-wait delays for WS2812 + software I2C
 * -------------------------------------------------------------------------*/
static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
    const uint32_t start  = DWT->CYCCNT;
    const uint32_t cycles = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < cycles) { }
}

// Common-anode / active-LOW LED: pin LOW = LED ON
static void led_set(bool red, bool green, bool blue)
{
    HAL_GPIO_WritePin(LED_PORT, LED_R_PIN, red   ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_PORT, LED_G_PIN, green ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_PORT, LED_B_PIN, blue  ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

// Pressed buttons as a bit mask (pull-up -> pressed reads LOW)
static uint8_t read_buttons(void)
{
    uint8_t m = 0;
    if (HAL_GPIO_ReadPin(BTN_PORT, BTN1_PIN)     == GPIO_PIN_RESET) m |= MASK_BTN1;
    if (HAL_GPIO_ReadPin(BTN_PORT, BTN2_PIN)     == GPIO_PIN_RESET) m |= MASK_BTN2;
    if (HAL_GPIO_ReadPin(BTN_PORT, BTN3_PIN)     == GPIO_PIN_RESET) m |= MASK_BTN3;
    if (HAL_GPIO_ReadPin(BTN_PORT, BTN4_PIN)     == GPIO_PIN_RESET) m |= MASK_BTN4;
    if (HAL_GPIO_ReadPin(BTN_PORT, USER_BTN_PIN) == GPIO_PIN_RESET) m |= MASK_USER;
    return m;
}

static uint16_t read_pot(void)
{
    uint16_t value = 0;
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK) value = (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return value;
}

// Restart a song from its first note (task context)
static void load_song(uint8_t idx, bool play)
{
    taskENTER_CRITICAL();               // masks the Ticker ISR
    g_song_idx   = idx;
    g_note_idx   = 0;
    g_note_ticks = 0;
    g_playing    = play;
    taskEXIT_CRITICAL();
}

/* ---------------------------------------------------------------------------
 *  Software I2C (PF1 = SCL, PF0 = SDA, open-drain) ~100 kHz
 * -------------------------------------------------------------------------*/
#define SCL_H()   (I2C_PORT->BSRR = I2C_SCL_PIN)
#define SCL_L()   (I2C_PORT->BSRR = (uint32_t)I2C_SCL_PIN << 16)
#define SDA_H()   (I2C_PORT->BSRR = I2C_SDA_PIN)
#define SDA_L()   (I2C_PORT->BSRR = (uint32_t)I2C_SDA_PIN << 16)
#define SDA_IN()  ((I2C_PORT->IDR & I2C_SDA_PIN) != 0)

static void i2c_start(void) { SDA_H(); SCL_H(); delay_us(5); SDA_L(); delay_us(5); SCL_L(); delay_us(5); }
static void i2c_stop(void)  { SDA_L(); delay_us(5); SCL_H(); delay_us(5); SDA_H(); delay_us(5); }

static bool i2c_write_byte(uint8_t v)            // returns true on ACK
{
    for (int i = 0; i < 8; ++i) {
        if (v & 0x80) SDA_H(); else SDA_L();
        v <<= 1;
        delay_us(5); SCL_H(); delay_us(5); SCL_L();
    }
    SDA_H(); delay_us(5); SCL_H(); delay_us(5);
    const bool ack = !SDA_IN();
    SCL_L(); delay_us(5);
    return ack;
}

static uint8_t i2c_read_byte(bool ack)
{
    uint8_t v = 0;
    SDA_H();
    for (int i = 0; i < 8; ++i) {
        delay_us(5); SCL_H(); delay_us(5);
        v = (uint8_t)((v << 1) | (SDA_IN() ? 1 : 0));
        SCL_L();
    }
    if (ack) SDA_L(); else SDA_H();
    delay_us(5); SCL_H(); delay_us(5); SCL_L(); SDA_H(); delay_us(5);
    return v;
}

static bool i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val)
{
    i2c_start();
    bool ok = i2c_write_byte((uint8_t)(addr << 1)) && i2c_write_byte(reg) && i2c_write_byte(val);
    i2c_stop();
    return ok;
}

static bool i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n)
{
    i2c_start();
    bool ok = i2c_write_byte((uint8_t)(addr << 1)) && i2c_write_byte(reg);
    if (ok) {
        i2c_start();                                     // repeated start
        ok = i2c_write_byte((uint8_t)((addr << 1) | 1));
        for (uint8_t i = 0; ok && i < n; ++i) buf[i] = i2c_read_byte(i + 1 < n);
    }
    i2c_stop();
    return ok;
}

/* ---------------------------------------------------------------------------
 *  ES8388 audio codec (same software I2C bus) - DAC -> headphone jack
 *  Register values follow the RT-Thread RT-Spark BSP driver (drv_es8388.c).
 * -------------------------------------------------------------------------*/
static bool es8388_init(void)
{
    uint8_t probe = 0;
    if (!i2c_read_regs(ES8388_ADDR, 0x00, &probe, 1)) return false;

    i2c_write_reg(ES8388_ADDR, 0x19, 0x04);   // DACCONTROL3: mute while configuring
    i2c_write_reg(ES8388_ADDR, 0x01, 0x50);   // CONTROL2
    i2c_write_reg(ES8388_ADDR, 0x02, 0x00);   // CHIPPOWER: all on
    i2c_write_reg(ES8388_ADDR, 0x08, 0x00);   // MASTERMODE: codec is I2S slave
    i2c_write_reg(ES8388_ADDR, 0x04, 0xC0);   // DACPOWER: off while configuring
    i2c_write_reg(ES8388_ADDR, 0x00, 0x12);   // CONTROL1
    i2c_write_reg(ES8388_ADDR, 0x17, 0x18);   // DACCONTROL1: I2S, 16-bit
    i2c_write_reg(ES8388_ADDR, 0x18, 0x02);   // DACCONTROL2: MCLK = 256 x Fs
    i2c_write_reg(ES8388_ADDR, 0x26, 0x00);   // DACCONTROL16: mixer input
    i2c_write_reg(ES8388_ADDR, 0x27, 0x90);   // DACCONTROL17: left DAC -> left mixer
    i2c_write_reg(ES8388_ADDR, 0x2A, 0x90);   // DACCONTROL20: right DAC -> right mixer
    i2c_write_reg(ES8388_ADDR, 0x2B, 0x80);   // DACCONTROL21: shared LRCK
    i2c_write_reg(ES8388_ADDR, 0x2D, 0x00);   // DACCONTROL23
    i2c_write_reg(ES8388_ADDR, 0x1A, 0x00);   // DACCONTROL4: left DAC 0 dB
    i2c_write_reg(ES8388_ADDR, 0x1B, 0x00);   // DACCONTROL5: right DAC 0 dB
    i2c_write_reg(ES8388_ADDR, 0x03, 0xFF);   // ADCPOWER: ADC off (not used)
    i2c_write_reg(ES8388_ADDR, 0x2E, JACK_VOLUME);   // DACCONTROL24: LOUT1 volume
    i2c_write_reg(ES8388_ADDR, 0x2F, JACK_VOLUME);   // DACCONTROL25: ROUT1 volume
    i2c_write_reg(ES8388_ADDR, 0x04, 0x3C);   // DACPOWER: DAC + outputs on
    i2c_write_reg(ES8388_ADDR, 0x02, 0xF0);   // restart the state machine
    i2c_write_reg(ES8388_ADDR, 0x02, 0x00);
    i2c_write_reg(ES8388_ADDR, 0x19, 0x00);   // unmute
    return true;
}

/* ============================================================================
 *  THREAD 1 - update_lcd_leds_thread : LCD screen + RGB LED
 * ==========================================================================*/
static void draw_screen(bool confirming, bool playing, int secs_left)
{
    const SongSlot &cur = g_songs[g_song_idx];
    const uint8_t   n   = g_song_idx;

    if (confirming)   lcd.print_line(2, COLOR_GREEN, "  CHANGE SONG?");
    else if (playing) lcd.print_line(2, COLOR_CYAN,  "  > PLAYING");
    else              lcd.print_line(2, COLOR_RED,   "  || PAUSED");

    lcd.print_line(4, COLOR_GRAY,  "Song %u/%u  [%c%c%c]", n + 1, NUM_SONGS,
                   (n & 4) ? '1' : '0', (n & 2) ? '1' : '0', (n & 1) ? '1' : '0');
    lcd.print_line(5, COLOR_WHITE, "%s", cur.song->name1.c_str());
    lcd.print_line(6, COLOR_WHITE, "%s", cur.song->name2.c_str());

    if (confirming) {
        const uint8_t p = g_pending_idx;
        const SongSlot &next = g_songs[p];
        lcd.print_line(8,  COLOR_GREEN,  "Selected [%c%c%c]:",
                       (p & 4) ? '1' : '0', (p & 2) ? '1' : '0', (p & 1) ? '1' : '0');
        lcd.print_line(9,  COLOR_YELLOW, "%s", next.song->name1.c_str());
        lcd.print_line(10, COLOR_YELLOW, "%s", next.song->name2.c_str());
        lcd.print_line(11, COLOR_GREEN,  "B1 = confirm (%ds)", secs_left);
    } else {
        for (int r = 8; r <= 11; ++r) lcd.print_line(r, COLOR_BLACK, "");
    }
}

static void update_lcd_leds_thread(void *argument)
{
    (void)argument;
    int last_secs = -1;

    for (;;) {
        const bool confirming = g_confirming;
        const bool playing    = g_playing;

        // RGB LED: green = waiting to change, blue = playing, red = paused
        if (confirming)   led_set(false, true,  false);
        else if (playing) led_set(false, false, true);
        else              led_set(true,  false, false);

        // Seconds left in the confirmation window (for the countdown)
        int secs = -1;
        if (confirming) {
            const uint32_t elapsed = (xTaskGetTickCount() - g_confirm_start) * portTICK_PERIOD_MS;
            secs = (elapsed >= CONFIRM_WINDOW_MS) ? 0
                 : (int)((CONFIRM_WINDOW_MS - elapsed + 999U) / 1000U);
        }

        if (g_lcd_dirty || secs != last_secs) {
            g_lcd_dirty = false;
            if (xSemaphoreTake(g_lcd_mutex, portMAX_DELAY) == pdTRUE) {
                draw_screen(confirming, playing, secs);
                xSemaphoreGive(g_lcd_mutex);
            }
            last_secs = secs;
        }

        vTaskDelay(pdMS_TO_TICKS(100));          // yield to the other threads
    }
}

/* ============================================================================
 *  THREAD 2 - polling_buttons : debounce + song selection / confirm / play
 * ==========================================================================*/
// Second B1 press inside the 5 s window -> confirm and play
static void confirm_song(void)
{
    taskENTER_CRITICAL();
    confirm_timeout.detach();
    g_confirming = false;
    taskEXIT_CRITICAL();
    load_song(g_pending_idx, true);
    g_lcd_dirty = true;

    char msg[64];
    snprintf(msg, sizeof(msg), "[MP3] Now playing: %s %s\r\n",
             g_songs[g_pending_idx].song->name1.c_str(),
             g_songs[g_pending_idx].song->name2.c_str());
    uart_puts(msg);
}

static void on_button1(uint8_t pressed)
{
    if (g_confirming) {
        confirm_song();
    } else {
        // First B1 press -> read B2..B4 as a 3-bit number (B2 = MSB)
        const uint8_t idx = ((pressed & MASK_BTN2) ? 4 : 0) |
                            ((pressed & MASK_BTN3) ? 2 : 0) |
                            ((pressed & MASK_BTN4) ? 1 : 0);
        g_pending_idx   = idx;
        g_confirm_start = xTaskGetTickCount();
        g_confirming    = true;
        confirm_timeout.attach(on_confirm_timeout, CONFIRM_WINDOW_S);
        g_lcd_dirty     = true;
    }
}

static void on_user_button(void)
{
    g_playing   = !g_playing;     // toggle play / pause
    g_lcd_dirty = true;
}

static void polling_buttons(void *argument)
{
    (void)argument;
    uint8_t stable   = read_buttons();
    uint8_t last_raw = stable;

    for (;;) {
        const uint8_t raw = read_buttons();

        // Accept a new state only if it is the same for 2 samples (20 ms)
        if (raw == last_raw && raw != stable) {
            const uint8_t new_presses = raw & (uint8_t)~stable;   // rising edges
            stable = raw;
            if (new_presses & MASK_BTN1) on_button1(raw);
            if (new_presses & MASK_USER) on_user_button();
        }
        last_raw = raw;

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ============================================================================
 *  THREAD 3 - adjust_volume : potentiometer -> volume 0..100 %
 * ==========================================================================*/
static void draw_volume(int vol)
{
    if (xSemaphoreTake(g_lcd_mutex, portMAX_DELAY) == pdTRUE) {
        lcd.print_line(13, COLOR_WHITE, "Volume: %3d%%", vol);
        const uint16_t bar = (uint16_t)(vol * 216 / 100);   // 216 px max
        lcd.fill_rect(12, 14 * LCD_ST7789::CELL_H + 4, bar, 8, COLOR_CYAN);
        lcd.fill_rect(12 + bar, 14 * LCD_ST7789::CELL_H + 4, 216 - bar, 8, COLOR_GRAY);
        xSemaphoreGive(g_lcd_mutex);
    }
}

static void adjust_volume(void *argument)
{
    (void)argument;
    // The headphone codec is set up once here (its I2C bus belongs to this thread)
    if (!es8388_init() && xSemaphoreTake(g_lcd_mutex, portMAX_DELAY) == pdTRUE) {
        lcd.print_line(12, COLOR_RED, "No audio chip");
        xSemaphoreGive(g_lcd_mutex);
    }

    uint32_t filtered = read_pot();
    int      last_vol = -1;

    for (;;) {
        filtered = (filtered * 3U + read_pot()) / 4U;           // simple low-pass
        const int vol = (int)((filtered * 100U) / 4095U);

        const int diff = (vol > last_vol) ? (vol - last_vol) : (last_vol - vol);
        if (last_vol < 0 || diff >= 2 || (vol == 0 && last_vol != 0) || (vol == 100 && last_vol != 100)) {
            g_volume = (uint8_t)vol;
            last_vol = vol;
            draw_volume(vol);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ============================================================================
 *  HARDWARE INITIALISATION
 * ==========================================================================*/
// 168 MHz from the internal 16 MHz HSI (no dependency on the external crystal)
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState            = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState        = RCC_PLL_ON;
    osc.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM            = 16;            // 16 MHz / 16 = 1 MHz
    osc.PLL.PLLN            = 336;           // x336 = 336 MHz VCO
    osc.PLL.PLLP            = RCC_PLLP_DIV2; // /2 = 168 MHz SYSCLK
    osc.PLL.PLLQ            = 7;             // /7 = 48 MHz (USB)
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                         RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;    // HCLK  168 MHz
    clk.APB1CLKDivider = RCC_HCLK_DIV4;      // PCLK1  42 MHz (timers 84 MHz)
    clk.APB2CLKDivider = RCC_HCLK_DIV2;      // PCLK2  84 MHz
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    // Push buttons: inputs with pull-up (external pull-ups also fitted)
    g.Pin  = BTN1_PIN | BTN2_PIN | BTN3_PIN | BTN4_PIN | USER_BTN_PIN;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BTN_PORT, &g);

    // RGB LED: open-drain, start OFF (HIGH)
    HAL_GPIO_WritePin(LED_PORT, LED_R_PIN | LED_G_PIN | LED_B_PIN, GPIO_PIN_SET);
    g.Pin   = LED_R_PIN | LED_G_PIN | LED_B_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &g);

    // Software I2C to the ES8388: open-drain with pull-up, idle HIGH
    HAL_GPIO_WritePin(I2C_PORT, I2C_SCL_PIN | I2C_SDA_PIN, GPIO_PIN_SET);
    g.Pin   = I2C_SCL_PIN | I2C_SDA_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(I2C_PORT, &g);

    // LCD reset + backlight: push-pull outputs
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin   = LCD_RST_PIN;
    HAL_GPIO_Init(LCD_RST_PORT, &g);
    g.Pin   = LCD_BL_PIN;
    HAL_GPIO_Init(LCD_BL_PORT, &g);
}

// FSMC Bank1 NE3, 8-bit, SRAM mode (8080 interface) - same as CubeMX project
static void MX_FSMC_Init(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_FSMC_CLK_ENABLE();

    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF12_FSMC;

    // D2 D3 NOE NWE A18 D0 D1
    g.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 |
            GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOD, &g);
    // D4..D7
    g.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    HAL_GPIO_Init(GPIOE, &g);
    // NE3
    g.Pin = GPIO_PIN_10;
    HAL_GPIO_Init(GPIOG, &g);

    FSMC_NORSRAM_TimingTypeDef t = {0};
    hsram3.Instance                = FSMC_NORSRAM_DEVICE;
    hsram3.Extended                = FSMC_NORSRAM_EXTENDED_DEVICE;
    hsram3.Init.NSBank             = FSMC_NORSRAM_BANK3;
    hsram3.Init.DataAddressMux     = FSMC_DATA_ADDRESS_MUX_DISABLE;
    hsram3.Init.MemoryType         = FSMC_MEMORY_TYPE_SRAM;
    hsram3.Init.MemoryDataWidth    = FSMC_NORSRAM_MEM_BUS_WIDTH_8;
    hsram3.Init.BurstAccessMode    = FSMC_BURST_ACCESS_MODE_DISABLE;
    hsram3.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
    hsram3.Init.WrapMode           = FSMC_WRAP_MODE_DISABLE;
    hsram3.Init.WaitSignalActive   = FSMC_WAIT_TIMING_BEFORE_WS;
    hsram3.Init.WriteOperation     = FSMC_WRITE_OPERATION_ENABLE;
    hsram3.Init.WaitSignal         = FSMC_WAIT_SIGNAL_DISABLE;
    hsram3.Init.ExtendedMode       = FSMC_EXTENDED_MODE_DISABLE;
    hsram3.Init.AsynchronousWait   = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
    hsram3.Init.WriteBurst         = FSMC_WRITE_BURST_DISABLE;

    t.AddressSetupTime      = 1;
    t.AddressHoldTime       = 15;
    t.DataSetupTime         = 60;
    t.BusTurnAroundDuration = 0;
    t.CLKDivision           = 16;
    t.DataLatency           = 17;
    t.AccessMode            = FSMC_ACCESS_MODE_A;
    if (HAL_SRAM_Init(&hsram3, &t, NULL) != HAL_OK) Error_Handler();
}

static void MX_USART1_UART_Init(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_USART1_CLK_ENABLE();

    g.Pin       = GPIO_PIN_9 | GPIO_PIN_10;   // TX, RX
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &g);

    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

static void MX_ADC1_Init(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_ADC1_CLK_ENABLE();

    g.Pin  = POT_PIN;
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(POT_PORT, &g);

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

    ADC_ChannelConfTypeDef ch = {0};
    ch.Channel      = POT_ADC_CHANNEL;
    ch.Rank         = 1;
    ch.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &ch) != HAL_OK) Error_Handler();
}

// TIM3 PWM at 1 MHz count rate: ARR = note period in us, CCR sets volume
static void MX_TIM3_PWM_Init(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_TIM3_CLK_ENABLE();

    g.Pin       = SPK_PIN;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_LOW;
    g.Alternate = SPK_AF;
    HAL_GPIO_Init(SPK_PORT, &g);

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 84 - 1;                // 84 MHz / 84 = 1 MHz
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 3822 - 1;              // any start value
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;  // glitch-free
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) Error_Handler();

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 0;                                    // silent
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &oc, SPK_TIM_CHANNEL) != HAL_OK) Error_Handler();
    HAL_TIM_PWM_Start(&htim3, SPK_TIM_CHANNEL);
}

// I2S3 master TX -> ES8388, 16 kHz, 16-bit stereo, MCLK out, circular DMA
static void MX_I2S3_Audio_Init(void)
{
    for (int i = 0; i < 256; ++i)
        g_sine[i] = (int16_t)(32767.0f * sinf(6.2831853f * (float)i / 256.0f));

    // PLLI2S: 1 MHz (HSI/16) x 192 / 2 = 96 MHz I2S kernel clock
    RCC_PeriphCLKInitTypeDef pclk = {0};
    pclk.PeriphClockSelection = RCC_PERIPHCLK_I2S;
    pclk.PLLI2S.PLLI2SN       = 192;
    pclk.PLLI2S.PLLI2SR       = 2;
    if (HAL_RCCEx_PeriphCLKConfig(&pclk) != HAL_OK) Error_Handler();

    __HAL_RCC_SPI3_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF6_SPI3;
    g.Pin = GPIO_PIN_15;               HAL_GPIO_Init(GPIOA, &g);   // WS
    g.Pin = GPIO_PIN_3 | GPIO_PIN_5;   HAL_GPIO_Init(GPIOB, &g);   // CK, SD
    g.Pin = GPIO_PIN_7;                HAL_GPIO_Init(GPIOC, &g);   // MCK

    hi2s3.Instance            = SPI3;
    hi2s3.Init.Mode           = I2S_MODE_MASTER_TX;
    hi2s3.Init.Standard       = I2S_STANDARD_PHILIPS;
    hi2s3.Init.DataFormat     = I2S_DATAFORMAT_16B;
    hi2s3.Init.MCLKOutput     = I2S_MCLKOUTPUT_ENABLE;
    hi2s3.Init.AudioFreq      = AUDIO_FS;
    hi2s3.Init.CPOL           = I2S_CPOL_LOW;
    hi2s3.Init.ClockSource    = I2S_CLOCK_PLL;
    hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
    if (HAL_I2S_Init(&hi2s3) != HAL_OK) Error_Handler();

    hdma_i2s3_tx.Instance                 = DMA1_Stream5;
    hdma_i2s3_tx.Init.Channel             = DMA_CHANNEL_0;
    hdma_i2s3_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_i2s3_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_i2s3_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_i2s3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_i2s3_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_i2s3_tx.Init.Mode                = DMA_CIRCULAR;
    hdma_i2s3_tx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_i2s3_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_i2s3_tx) != HAL_OK) Error_Handler();
    __HAL_LINKDMA(&hi2s3, hdmatx, hdma_i2s3_tx);

    // Priority 6: below configMAX_SYSCALL (5); the callbacks use no RTOS calls
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

    memset(g_audio_buf, 0, sizeof(g_audio_buf));
    // Starts MCLK/BCLK/LRCK now - the ES8388 needs MCLK before it is configured
    HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t *)g_audio_buf, AUDIO_BUF_FRAMES * 2);
}

/* ============================================================================
 *  SYSTICK shared by the HAL (HAL_GetTick) and the FreeRTOS kernel.
 *  -> FreeRTOSConfig.h must NOT contain "#define xPortSysTickHandler SysTick_Handler"
 * ==========================================================================*/
extern "C" void xPortSysTickHandler(void);

extern "C" void SysTick_Handler(void)
{
    HAL_IncTick();
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
}

/* ============================================================================
 *  FREERTOS HOOKS
 * ==========================================================================*/
// Idle task = nothing to do -> SLEEP mode (CPU halts until the next interrupt)
extern "C" void vApplicationIdleHook(void)
{
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
}

extern "C" void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task; (void)name;
    Error_Handler();
}

extern "C" void vApplicationMallocFailedHook(void)
{
    Error_Handler();
}

/* ============================================================================
 *  MAIN
 * ==========================================================================*/
static const char kInstructions[] =
    "\r\n==================== MP3 PLAYER ====================\r\n"
    " SELECT : hold B2 B3 B4 as a binary number (B2 = MSB)\r\n"
    "          000 = song 1 ... 111 = song 8,\r\n"
    "          press B1, then release all buttons.\r\n"
    " CONFIRM: press B1 again within 5 seconds.\r\n"
    "          No press -> keep the current song.\r\n"
    " PLAY / PAUSE : press USER_BUTTON.\r\n"
#if USE_SPEAKER
    " AUDIO  : onboard buzzer.\r\n"
#else
    " AUDIO  : IEMs / headphones on the 3.5 mm jack.\r\n"
#endif
    " VOLUME : turn the potentiometer.\r\n"
    " LEDS   : BLUE = playing, RED = paused,\r\n"
    "          GREEN = waiting for confirmation.\r\n"
    " Songs  :\r\n";

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_FSMC_Init();
    MX_USART1_UART_Init();
    MX_ADC1_Init();
    MX_TIM3_PWM_Init();
    MX_I2S3_Audio_Init();
    dwt_init();
    __HAL_RCC_TIM6_CLK_ENABLE();
    __HAL_RCC_TIM7_CLK_ENABLE();
    init_song_table();

    // 1) Clear the LCD display and draw the static title
    lcd.init_lcd();
    lcd.set_color(COLOR_WHITE, COLOR_BLACK);
    lcd.clr_lcd();
    lcd.print_line(0, COLOR_YELLOW, " MSU-IIT MP3 PLAYER");
    led_set(true, false, false);                 // start paused (red)
    load_song(0, false);

    // 2) Send the usage instructions via UART.
    //    (Done before any FreeRTOS call: creating kernel objects before the
    //     scheduler starts masks interrupts, which would stall HAL_GetTick.)
    uart_puts(kInstructions);
    for (uint8_t i = 0; i < NUM_SONGS; ++i) {
        char line[64];
        snprintf(line, sizeof(line), "   [%c%c%c] %u. %s %s\r\n",
                 (i & 4) ? '1' : '0', (i & 2) ? '1' : '0', (i & 1) ? '1' : '0', i + 1,
                 g_songs[i].song->name1.c_str(), g_songs[i].song->name2.c_str());
        uart_puts(line);
    }
    uart_puts("====================================================\r\n");

    // Ticker drives the notes every TICKER_MS (runs once interrupts are enabled)
    music_ticker.attach(play_tick, TICKER_MS / 1000.0f);

    // 3) Create the LCD mutex and the 3 threads (same priority, cooperative)
    g_lcd_mutex = xSemaphoreCreateMutex();
    if (g_lcd_mutex == nullptr) Error_Handler();

    if (xTaskCreate(update_lcd_leds_thread, "lcd_leds", 768, nullptr, 2, nullptr) != pdPASS) Error_Handler();
    if (xTaskCreate(polling_buttons,        "buttons",  384, nullptr, 2, nullptr) != pdPASS) Error_Handler();
    if (xTaskCreate(adjust_volume,          "volume",   512, nullptr, 2, nullptr) != pdPASS) Error_Handler();

    // 4) Start the threads. When all threads are blocked, the idle hook puts
    //    the MCU in SLEEP mode to reduce power consumption.
    vTaskStartScheduler();

    // Only reached if there was not enough heap for the idle task
    Error_Handler();
    return 0;
}
