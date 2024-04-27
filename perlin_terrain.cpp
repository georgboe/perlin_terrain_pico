#include <string.h>
#include <math.h>
#include <vector>
#include <cstdlib>
#include <string>

#include "hardware/pll.h"
#include "hardware/vreg.h"
#include "hardware/regs/rosc.h"
#include "hardware/regs/addressmap.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"

#include "drivers/st7789/st7789.hpp"
#include "libraries/pico_graphics/pico_graphics.hpp"
#include "drivers/rgbled/rgbled.hpp"
// #include "FixedPoint15.cpp"
// #include "PerlinNoiseFixed15.hpp"
#include "PerlinNoise.hpp"
#include "drivers/button/button.hpp"
#include "build/st7789_lcd.pio.h"
#if PICO_DISPLAY_2
  #include "libraries/pico_display_2/pico_display_2.hpp"
#else
  #include "libraries/pico_display/pico_display.hpp"
#endif

using namespace pimoroni;

SPIPins spi_pins = get_spi_pins(BG_SPI_FRONT);



#if PICO_DISPLAY_2
  #define WIDTH PicoDisplay2::WIDTH
  #define HEIGHT PicoDisplay2::HEIGHT
#else
  #define WIDTH PicoDisplay::WIDTH
  #define HEIGHT PicoDisplay::HEIGHT
#endif

// ST7789 st7789(WIDTH, HEIGHT, ROTATE_0, false, spi_pins);
void *frame_buffer = (void *)(new uint8_t[WIDTH * HEIGHT * sizeof(RGB565)]);
// PicoGraphics_PenRGB565 graphics(st7789.width, st7789.height, frame_buffer);


#if PICO_DISPLAY_2
  RGBLED led(PicoDisplay2::LED_R, PicoDisplay2::LED_G, PicoDisplay2::LED_B);
#else
  RGBLED led(PicoDisplay::LED_R, PicoDisplay::LED_G, PicoDisplay::LED_B);
#endif

siv::PerlinNoise perlin{ };

PIO pio = pio0;
uint sm = 0;

void set_binary_info() {
  bi_decl(bi_1pin_with_name(6, "LED_R"));
  bi_decl(bi_1pin_with_name(7, "LED_G"));
  bi_decl(bi_1pin_with_name(8, "LED_B"));

  bi_decl(bi_1pin_with_name(12, "SW_A"));
  bi_decl(bi_1pin_with_name(13, "SW_B"));
  bi_decl(bi_1pin_with_name(14, "SW_X"));
  bi_decl(bi_1pin_with_name(15, "SW_Y"));

  bi_decl(bi_1pin_with_name(16, "LCD_DC"));
  bi_decl(bi_1pin_with_name(17, "LCD_CS"));
  bi_decl(bi_1pin_with_name(18, "LCD_SCLK"));
  bi_decl(bi_1pin_with_name(19, "LCD_MOSI"));
  bi_decl(bi_1pin_with_name(20, "BL_EN"));
}

uint32_t rnd(void){
    int k, random=0;
    volatile uint32_t *rnd_reg=(uint32_t *)(ROSC_BASE + ROSC_RANDOMBIT_OFFSET);
    
    for(k=0;k<32;k++){
    
    random = random << 1;
    random=random + (0x00000001 & (*rnd_reg));

    }
    return random;
}

double clamp(double value, double min, double max) {
  return std::min(std::max(value, min), max);
}

constexpr uint16_t rgb_to_rgb565(uint8_t r, uint8_t g, uint8_t b) {
      uint16_t p = ((r & 0b11111000) << 8) |
                   ((g & 0b11111100) << 3) |
                   ((b & 0b11111000) >> 3);

      return __builtin_bswap16(p);
      // return 0b0000000000000000;;
}

uint16_t get_pen_for_noise(double noise) {

  // colors ={
  //     0.3: 0x3363c1,
  //     0.4: 0x3666c5,
  //     0.45: 0xced17f,
  //     0.55: 0x559716,
  //     0.6: 0x3f6a13,
  //     0.7: 0x5c433e,
  //     0.9: 0x4b3c38,
  //     1.0: 0xffffff
  // }


  if (noise <= 0.3) {
    return rgb_to_rgb565(44, 86, 168);
  }
  else if (noise <= 0.4) {
    return rgb_to_rgb565(54, 102, 197);
  }
  else if (noise <= 0.45) {
    return rgb_to_rgb565(206, 209, 127);
  }
  else if (noise <= 0.55) {
    return rgb_to_rgb565(85, 151, 22);
  }
  else if (noise <= 0.6) {
    return rgb_to_rgb565(63, 106, 19);
  }
  else if (noise <= 0.7) {
    return rgb_to_rgb565(92, 67, 62);
  }
  else if (noise <= 0.9) {
    return rgb_to_rgb565(73, 59, 54);
  }
  else {
    return rgb_to_rgb565(255, 255, 255);
  }
}

float evaluate(float value) {
  float a = 3;
  float b = 2.2f;
  return pow(value, a) / (pow(value, a) + pow(b - b * value, a));
}

inline void compute_line(uint16_t y, int32_t y_with_offset, uint16_t start_x, uint16_t end_x) {
  uint16_t *buf = (uint16_t *)frame_buffer;
  int32_t octaves = 9;
  for (int x = start_x; x < end_x; x++)
  {
    double noise = perlin.octave2D_01((x * 0.01), (y_with_offset * 0.01), octaves);
    buf[(y * WIDTH) + x] = get_pen_for_noise((float) noise);
  }
}

#define PIN_DIN 19
#define PIN_CLK 18
#define PIN_CS 17
#define PIN_DC 16
// #define PIN_RESET 30
#define PIN_BL 20

// #define SERIAL_CLK_DIV 50.f
#define SERIAL_CLK_DIV 3.0f

#define INVERTED false

// Format: cmd length (including cmd byte), post delay in units of 5 ms, then cmd payload
// Note the delays have been shortened a little
static const uint8_t st7789_init_seq[] = {
        1, 20, 0x01,                         // Software reset
        1, 10, 0x11,                         // Exit sleep mode
        2, 2, 0x3a, 0x55,                   // Set colour mode to 16 bit
        2, 0, 0x36, INVERTED ? 0x60 : 0x20,                   // Set MADCTL: row then column, refresh is bottom to top ????
#if PICO_DISPLAY_2
        5, 0, 0x2a, 0x00, 0x00, 0x01, 0x40, // CASET: column addresses from 0 to 240 (f0)
        5, 0, 0x2b, 0x00, 0x00, 0x00, 0xf0, // RASET: row addresses from 0 to 240 (f0)
#else
        5, 0, 0x2a, 0x00, 0x28, 0x01, 0x17, // CASET: column addresses from 0 to 240 (f0)
        5, 0, 0x2b, 0x00, 0x35, 0x00, 0xbb, // RASET: row addresses from 0 to 240 (f0)
#endif
        1, 2, 0x21,                         // Inversion on, then 10 ms delay (supposedly a hack?)
        1, 2, 0x13,                         // Normal display on, then 10 ms delay
        1, 2, 0x29,                         // Main screen turn on, then wait 500 ms
        0                                     // Terminate list
};

static inline void lcd_set_dc_cs(bool dc, bool cs) {
    sleep_us(1);
    gpio_put_masked((1u << PIN_DC) | (1u << PIN_CS), !!dc << PIN_DC | !!cs << PIN_CS);
    sleep_us(1);
}

static inline void lcd_write_cmd(PIO pio, uint sm, const uint8_t *cmd, size_t count) {
    st7789_lcd_wait_idle(pio, sm);
    lcd_set_dc_cs(0, 0);
    st7789_lcd_put(pio, sm, *cmd++);
    if (count >= 2) {
        st7789_lcd_wait_idle(pio, sm);
        lcd_set_dc_cs(1, 0);
        for (size_t i = 0; i < count - 1; ++i)
            st7789_lcd_put(pio, sm, *cmd++);
    }
    st7789_lcd_wait_idle(pio, sm);
    lcd_set_dc_cs(1, 1);
}

static inline void lcd_init(PIO pio, uint sm, const uint8_t *init_seq) {
    const uint8_t *cmd = init_seq;
    while (*cmd) {
        lcd_write_cmd(pio, sm, cmd + 2, *cmd);
        sleep_ms(*(cmd + 1) * 5);
        cmd += *cmd + 2;
    }
}

static inline void st7789_start_pixels(PIO pio, uint sm) {
    uint8_t cmd = 0x2c; // RAMWR
    lcd_write_cmd(pio, sm, &cmd, 1);
    lcd_set_dc_cs(1, 0);
}

void set_backlight(uint8_t brightness) {
  // gamma correct the provided 0-255 brightness value onto a
  // 0-65535 range for the pwm counter
  float gamma = 2.8;
  uint16_t value = (uint16_t)(pow((float)(brightness) / 255.0f, gamma) * 65535.0f + 0.5f);
  pwm_set_gpio_level(PIN_BL, value);
}

void core1_entry() {
    while (true) {
        uint32_t y_with_offset = multicore_fifo_pop_blocking();
        compute_line(0, -y_with_offset, (WIDTH/2), WIDTH);
        // compute_line(0, -y_with_offset, 0, WIDTH);
        multicore_fifo_push_blocking(true);
    }
}

#define DMA_CHANNEL         (0) /* bit plane content DMA channel */
#define DMA_CHANNEL_MASK    (1u<<DMA_CHANNEL)
static struct semaphore reset_delay_complete_sem;

void dma_complete_handler(void) {
  if (dma_hw->ints0 & DMA_CHANNEL_MASK) { /* are we called for our DMA channel? */
    dma_hw->ints0 = DMA_CHANNEL_MASK; /* clear IRQ */
    sem_release(&reset_delay_complete_sem);
  }
}

static void dma_init(PIO pio, unsigned int sm) {
  dma_claim_mask(DMA_CHANNEL_MASK); /* check that the DMA channel we want is available */
  dma_channel_config channel_config = dma_channel_get_default_config(DMA_CHANNEL); /* get default configuration */
  channel_config_set_dreq(&channel_config, pio_get_dreq(pio, sm, true)); /* configure data request. true: sending data to the PIO state machine */
  channel_config_set_transfer_data_size(&channel_config, DMA_SIZE_8); /* data transfer size is 32 bits */
  channel_config_set_read_increment(&channel_config, true); /* each read of the data will increase the read pointer */
  dma_channel_configure(DMA_CHANNEL,
                        &channel_config,
                        &pio->txf[sm], /* write address: write to PIO FIFO */
                        NULL, /* don't provide a read address yet */
                        WIDTH * HEIGHT * sizeof(RGB565), /* number of transfers */
                        false); /* don't start yet */
  irq_set_exclusive_handler(DMA_IRQ_0, dma_complete_handler); /* after DMA all data, raise an interrupt */
  dma_channel_set_irq0_enabled(DMA_CHANNEL, true); /* map DMA channel to interrupt */
  irq_set_enabled(DMA_IRQ_0, true); /* enable interrupt */
}




int main()
{
  stdio_init_all();
  vreg_set_voltage(VREG_VOLTAGE_1_30);
  set_sys_clock_khz(420000, true);

  set_binary_info();

  uint pio_offset = pio_add_program(pio, &st7789_lcd_program);
  st7789_lcd_program_init(pio, sm, pio_offset, PIN_DIN, PIN_CLK, SERIAL_CLK_DIV);

  gpio_init(PIN_CS);
  gpio_init(PIN_DC);
  // gpio_init(PIN_RESET);
  gpio_init(PIN_BL);
  gpio_set_dir(PIN_CS, GPIO_OUT);
  gpio_set_dir(PIN_DC, GPIO_OUT);
  // gpio_set_dir(PIN_RESET, GPIO_OUT);
  gpio_set_dir(PIN_BL, GPIO_OUT);

  gpio_put(PIN_CS, 1);
  // gpio_put(PIN_RESET, 1);
  lcd_init(pio, sm, st7789_init_seq);

  pwm_config cfg = pwm_get_default_config();
  pwm_set_wrap(pwm_gpio_to_slice_num(PIN_BL), 65535);
  pwm_init(pwm_gpio_to_slice_num(PIN_BL), &cfg, true);
  gpio_set_function(PIN_BL, GPIO_FUNC_PWM);
  set_backlight(0); // Turn backlight off initially to avoid nasty surprises

  // gpio_put(PIN_BL, 1);

  sem_init(&reset_delay_complete_sem, 1, 1);
  dma_init(pio, sm);

  set_backlight(200);
  led.set_rgb(0, 255, 0);

  multicore_launch_core1(core1_entry);

restart:
  printf("restarting\n");
  // graphics.set_pen(get_pen_for_noise(0));
  // graphics.clear();
  // st7789.update(&graphics);

  uint32_t reseed_value = rnd();
  printf("Reseeding with value %lu\n", reseed_value);
  perlin.reseed(reseed_value);

  uint16_t *buf = (uint16_t *)frame_buffer;
  for (int y = 0; y < WIDTH; ++y) {
      for (int x = 0; x < HEIGHT; ++x) {
        buf[(y * HEIGHT) + x] = get_pen_for_noise(0);
    }
  }

  uint32_t offset = 0;
  // bool led_is_active = false;
  absolute_time_t last_frame = get_absolute_time();
  multicore_fifo_push_blocking(offset);

  st7789_start_pixels(pio, sm);
  while (true)
  {
    offset++;
    if (offset == UINT32_MAX) {
      goto restart;
    }
    if (offset % 2 == 0) {
      led.set_rgb(0, 64, 0);
    } else {
      led.set_rgb(0, 0, 0);
    }

    sem_acquire_blocking(&reset_delay_complete_sem);
    multicore_fifo_pop_blocking();
    memmove(buf + WIDTH, buf, ((HEIGHT*WIDTH) - WIDTH) * sizeof(RGB565));
    multicore_fifo_push_blocking(offset);
    compute_line(0, -offset, 0, (WIDTH/2));  

     /* get semaphore */
    dma_channel_set_read_addr(DMA_CHANNEL, (void*)buf, true);

    uint64_t diff = absolute_time_diff_us(last_frame, get_absolute_time());
    printf("frametime: %lld us\n", diff);
    last_frame = get_absolute_time();

  }

  return 0;
}
