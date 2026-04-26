#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

/* --- TWI --- */
#define TWI_TIMEOUT 2000 

/* --- SSD1306 --- */
#define SSD1306_I2C_ADDR (0x3C << 1)
#define WIDTH 128
#define HEIGHT 64
#define PAGES (HEIGHT / 8)
static uint8_t buffer[WIDTH*PAGES];

/* --- MAX30102 --- */
#define MAX30102_ADDR   0x57
#define REG_PART_ID     0xFF
#define REG_MODE_CONFIG 0x09
#define REG_SPO2_CONFIG 0x0A
#define REG_LED1_PA     0x0C
#define REG_LED2_PA     0x0D
#define REG_FIFO_DATA   0x07
#define REG_FIFO_WR_PTR 0x04
#define REG_FIFO_RD_PTR 0x06
#define REG_FIFO_CONFIG 0x08

/* --- WS2812B --- */
#define LED_PIN PD6     
#define LED_COUNT 14  

/* --- Beat detection variables --- */
int16_t IR_AC_Signal_Current = 0;
int16_t IR_AC_Signal_Previous = 0;
int16_t IR_AC_Signal_min = 0;
int16_t IR_AC_Signal_max = 0;
int16_t IR_Average_Estimated = 0;
int16_t positiveEdge = 0;
int16_t negativeEdge = 0;
int32_t ir_avg_reg = 0;
int16_t cbuf[32] = {0};
uint8_t offset = 0;
static const uint16_t FIRCoeffs[12] = {172, 321, 579, 927, 1360, 1858, 2390, 2916, 3391, 3768, 4012, 4096};

/* --- BPM calculation --- */
volatile uint32_t millis_counter = 0;
uint32_t lastBeatTime = 0;
uint16_t bpm = 0;
uint8_t validBeatCount = 0;

// HRV variables
#define IBI_SIZE 5
uint16_t ibi[IBI_SIZE];
uint8_t ibiSpot = 0;
bool ibiFilled = false;
uint16_t hrvValue = 0;

// BPM averaging
#define BPM_SIZE 16
uint16_t bpmHistory[BPM_SIZE];
uint8_t bpmSpot = 0;
bool bpmFilled = false;

// LED data array (RGB format - we'll handle GRB conversion in send function)
typedef struct {
    uint8_t r, g, b;
} rgb_color_t;

static rgb_color_t led_array[LED_COUNT];

/* --- Font for display (extended) --- */
static const uint8_t font5x7[][5] = {
    {0x3E,0x51,0x49,0x45,0x3E}, // 0
    {0x00,0x42,0x7F,0x40,0x00}, // 1
    {0x42,0x61,0x51,0x49,0x46}, // 2
    {0x21,0x41,0x45,0x4B,0x31}, // 3
    {0x18,0x14,0x12,0x7F,0x10}, // 4
    {0x27,0x45,0x45,0x45,0x39}, // 5
    {0x3C,0x4A,0x49,0x49,0x30}, // 6
    {0x01,0x71,0x09,0x05,0x03}, // 7
    {0x36,0x49,0x49,0x49,0x36}, // 8
    {0x06,0x49,0x49,0x29,0x1E}, // 9
    {0x00,0x00,0x00,0x00,0x00}, // (10) space    
    {0x7E,0x11,0x11,0x11,0x7E}, // (11) A
    {0x7F,0x49,0x49,0x49,0x36}, // (12) B
    {0x3E,0x41,0x41,0x41,0x22}, // (13) C
    {0x7F,0x41,0x41,0x22,0x1C}, // (14) D
    {0x7F,0x49,0x49,0x49,0x41}, // (15) E
    {0x3E,0x41,0x49,0x49,0x3A}, // (16) G
    {0x7F,0x08,0x08,0x08,0x7F}, // (17) H
    {0x00,0x41,0x7F,0x41,0x00}, // (18) I
    {0x7F,0x40,0x40,0x40,0x40}, // (19) L
    {0x7F,0x02,0x0C,0x02,0x7F}, // (20) M
    {0x7F,0x04,0x08,0x10,0x7F}, // (21) N
    {0x3E,0x41,0x41,0x41,0x3E}, // (22) O
    {0x7F,0x09,0x09,0x09,0x06}, // (23) P   
    {0x7F,0x09,0x19,0x29,0x46}, // (24) R    
    {0x01,0x01,0x7F,0x01,0x01}, // (25) T
    {0x3F,0x40,0x40,0x40,0x3F}, // (26) U
    {0x1F,0x20,0x40,0x20,0x1F}, // (27) V
    {0x7F,0x10,0x08,0x10,0x7F}, // (28) W

};

// WS2812B function prototypes
void set_pixel_color(uint8_t pixel, uint8_t r, uint8_t g, uint8_t b);
void clear_all(void);
uint32_t color(uint8_t r, uint8_t g, uint8_t b);
void breathing_effect(uint32_t color);
void set_strip_brightness(uint32_t color, uint8_t brightness);
void color_wipe(uint32_t color, uint16_t wait);
void delay_ms(uint16_t ms);

/* --- Timer0 interrupt every 1ms --- */
ISR(TIMER0_COMPA_vect) {
    millis_counter++;
}

void timer0_init(void) {
    TCCR0A = (1 << WGM01);           // CTC mode
    TCCR0B = (1 << CS01) | (1 << CS00);  // Prescaler 64
    OCR0A = 124;                     // (8MHz / 64 / 1000) - 1 = 124
    TIMSK0 = (1 << OCIE0A);          // Enable compare match interrupt
    sei();                           // Enable global interrupts
}

uint32_t millis_get(void) {
    uint32_t m;
    cli();
    m = millis_counter;
    sei();
    return m;
}

/* --- Simple integer square root function --- */
uint16_t int_sqrt(uint32_t x) {
    uint32_t op = x;
    uint32_t res = 0;
    uint32_t one = 1UL << 30;

    while (one > op) one >>= 2;
    while (one != 0) {
        if (op >= res + one) {
            op -= res + one;
            res = (res >> 1) + one;
        } else {
            res >>= 1;
        }
        one >>= 2;
    }
    return (uint16_t)res;
}


/* --- Beat detection support --- */
int32_t mul16(int16_t x, int16_t y) { 
    return (int32_t)x * y; 
}
int16_t lowPassFIRFilter(int16_t din) {
    cbuf[offset] = din;
    int32_t z = mul16(FIRCoeffs[11], cbuf[(offset - 11) & 0x1F]);
    
    for (uint8_t i = 0; i < 11; i++) {
        z += mul16(FIRCoeffs[i], cbuf[(offset - i) & 0x1F] + cbuf[(offset - 22 + i) & 0x1F]);
    }
    
    offset = (offset + 1) & 0x1F;
    return z >> 15;
}

int16_t averageDCEstimator(int32_t *p, uint16_t x) {
    *p += ((((long)x << 15) - *p) >> 4);
    return (*p >> 15);
}

bool checkForBeat(int32_t sample) {
    bool beatDetected = false;
    IR_AC_Signal_Previous = IR_AC_Signal_Current;
    IR_Average_Estimated = averageDCEstimator(&ir_avg_reg, sample);
    IR_AC_Signal_Current = lowPassFIRFilter(sample - IR_Average_Estimated);

    // Detect positive zero crossing (rising edge)
    if ((IR_AC_Signal_Previous < 0) && (IR_AC_Signal_Current >= 0)) {
        int16_t amplitude = IR_AC_Signal_max - IR_AC_Signal_min;
        positiveEdge = 1; 
        negativeEdge = 0; 
        IR_AC_Signal_max = 0;
        
        if (amplitude > 20 && amplitude < 1000) {
            beatDetected = true;
        }
    }
    
    // Detect negative zero crossing (falling edge)
    if ((IR_AC_Signal_Previous > 0) && (IR_AC_Signal_Current <= 0)) {
        positiveEdge = 0; 
        negativeEdge = 1; 
        IR_AC_Signal_min = 0;
    }
    
    // Find maximum in positive cycle
    if (positiveEdge && IR_AC_Signal_Current > IR_AC_Signal_Previous) {
        IR_AC_Signal_max = IR_AC_Signal_Current;
    }
    
    // Find minimum in negative cycle
    if (negativeEdge && IR_AC_Signal_Current < IR_AC_Signal_Previous) {
        IR_AC_Signal_min = IR_AC_Signal_Current;
    }

    return beatDetected;
}

/* --- BPM and HRV calculation --- */
void updateBpmAndHrv(uint32_t ibiInterval) {
    // Ignore invalid IBI (outside 300–2000 ms range)
    if (ibiInterval < 300 || ibiInterval > 2000) return;

    // Calculate current BPM
    uint16_t currentBPM = 60000U / ibiInterval;

    // Store BPM in circular buffer
    bpmHistory[bpmSpot] = currentBPM;
    bpmSpot++;
    if (bpmSpot >= BPM_SIZE) {
        bpmSpot = 0;
        bpmFilled = true;
    }

    // Average BPM
    if (bpmFilled) {
        uint32_t sum = 0;
        for (uint8_t i = 0; i < BPM_SIZE; i++) {
            sum += bpmHistory[i];
        }
        bpm = (uint16_t)(sum / BPM_SIZE);
    } else {
        bpm = currentBPM;
    }

    // Store IBI in circular buffer
    ibi[ibiSpot] = ibiInterval;
    ibiSpot++;
    if (ibiSpot >= IBI_SIZE) {
        ibiSpot = 0;
        ibiFilled = true;
    }

    // HRV calculation (RMSSD)
    if (ibiFilled) {
        uint32_t sumIbi = 0;
        for (uint8_t i = 0; i < IBI_SIZE; i++) {
            sumIbi += ibi[i];
        }
        //uint16_t meanIbi = (uint16_t)(sumIbi / IBI_SIZE);

        uint32_t sumSquaredDiffs = 0;
        uint8_t count = 0;

        for (uint8_t i = 1; i < IBI_SIZE; i++) {
            int16_t diff = (int16_t)ibi[i] - (int16_t)ibi[i - 1];

            // Filter: ignore sudden large changes
            if (diff < -400 || diff > 400) continue;

            // Filter: ignore IBI too far from mean (>30% deviation)
            //if (ibi[i] < (meanIbi * 70U / 100U) || ibi[i] > (meanIbi * 130U / 100U)) continue;

            sumSquaredDiffs += (uint32_t)(diff * diff);
            count++;
        }

        if (count > 0) {
            uint32_t meanSquaredDiff = sumSquaredDiffs / count;
            hrvValue = int_sqrt(meanSquaredDiff);  // RMSSD
        }
    }
}

/* --- String helpers --- */
void reverse_string(char* str, int len) {
    int s = 0, e = len - 1;
    while (s < e) {
        char t = str[s]; 
        str[s] = str[e]; 
        str[e] = t; 
        s++; 
        e--;
    }
}

void uint32_to_string(uint32_t num, char* str) {
    int i = 0;
    if (num == 0) {
        str[i++] = '0';
        str[i] = 0;
        return;
    }
    while (num) {
        str[i++] = (num % 10) + '0';
        num /= 10;
    }
    str[i] = 0;
    reverse_string(str, i);
}

/* --- I2C functions --- */
void twi_init(void) {
    TWSR = 0x00; 
    TWBR = ((F_CPU / 100000UL) - 16) / 2; 
    TWCR = (1 << TWEN);
}

uint8_t twi_start(uint8_t address, uint8_t rw) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    
    uint8_t status = TWSR & 0xF8;
    if (status != 0x08 && status != 0x10) return 0;
    
    TWDR = (address << 1) | (rw & 1);
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    
    status = TWSR & 0xF8;
    if ((rw == 0 && status != 0x18) || (rw == 1 && status != 0x40)) return 0;
    return 1;
}

void twi_stop(void) { 
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
    while (TWCR & (1 << TWSTO));
}

void twi_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

uint8_t twi_read_ack(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

uint8_t twi_read_nack(void) {
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

/* --- MAX30102 helpers --- */
uint8_t max30102_read(uint8_t reg) {
    if (!twi_start(MAX30102_ADDR, 0)) return 0xFF;
    twi_write(reg);
    if (!twi_start(MAX30102_ADDR, 1)) return 0xFF;
    uint8_t val = twi_read_nack();
    twi_stop();
    return val;
}

void max30102_write(uint8_t reg, uint8_t val) {
    if (!twi_start(MAX30102_ADDR, 0)) return;
    twi_write(reg);
    twi_write(val);
    twi_stop();
}

void max30102_read_fifo_spo2(uint32_t* red, uint32_t* ir) {
    *red = 0;
    *ir = 0;
    
    if (!twi_start(MAX30102_ADDR, 0)) return;
    twi_write(REG_FIFO_DATA);
    if (!twi_start(MAX30102_ADDR, 1)) return;
    
    uint8_t r1 = twi_read_ack(), r2 = twi_read_ack(), r3 = twi_read_ack();
    uint8_t i1 = twi_read_ack(), i2 = twi_read_ack(), i3 = twi_read_nack();
    twi_stop();
    
    *red = ((uint32_t)r1 << 16) | ((uint32_t)r2 << 8) | r3;
    *red &= 0x3FFFF;
    *ir = ((uint32_t)i1 << 16) | ((uint32_t)i2 << 8) | i3;
    *ir &= 0x3FFFF;
}

/* --- SSD1306 functions --- */
void ssd1306_command(uint8_t cmd) {
    if (!twi_start(SSD1306_I2C_ADDR >> 1, 0)) return;
    twi_write(0x00);
    twi_write(cmd);
    twi_stop();
}

void ssd1306_init(void) {
    _delay_ms(100);
    ssd1306_command(0xAE); ssd1306_command(0xD5); ssd1306_command(0x80);
    ssd1306_command(0xA8); ssd1306_command(0x3F); ssd1306_command(0xD3);
    ssd1306_command(0x00); ssd1306_command(0x40); ssd1306_command(0x8D);
    ssd1306_command(0x14); ssd1306_command(0x20); ssd1306_command(0x00);
    ssd1306_command(0xA1); ssd1306_command(0xC8); ssd1306_command(0xDA);
    ssd1306_command(0x12); ssd1306_command(0x81); ssd1306_command(0xCF);
    ssd1306_command(0xD9); ssd1306_command(0xF1); ssd1306_command(0xDB);
    ssd1306_command(0x40); ssd1306_command(0xA4); ssd1306_command(0xA6);
    ssd1306_command(0xAF);
}

void ssd1306_clear(void) { 
    memset(buffer, 0, sizeof(buffer)); 
}

void init_ws2812(void) {
    // Set LED_PIN as output
    DDRD |= (1 << LED_PIN);
    PORTD &= ~(1 << LED_PIN);  // Start with pin low
    
    // Initialize all LEDs to off
    clear_all();
}

void ws2812_send_bit(uint8_t bit) {
    if (bit) {
        // Send '1' bit: 0.875us high, 0.375us low (7 + 3 = 10 cycles total)
        PORTD |= (1 << LED_PIN);   // Set high
        // 7 cycles high (0.875us at 8MHz)
        __asm__ __volatile__ (
            "nop\n\t"
            "nop\n\t" 
            "nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t"
            ::
        );
        PORTD &= ~(1 << LED_PIN);  // Set low
        // 3 cycles low (0.375us at 8MHz) - but return and loop overhead provides this
        __asm__ __volatile__ (
            "nop\n\t"
            ::
        );
    } else {
        // Send '0' bit: 0.25us high, 1.0us low (2 + 8 = 10 cycles total)
        PORTD |= (1 << LED_PIN);   // Set high
        // 2 cycles high (0.25us at 8MHz)
        __asm__ __volatile__ (
            "nop\n\t"
            "nop\n\t"
            ::
        );
        PORTD &= ~(1 << LED_PIN);  // Set low
        // 8 cycles low (1.0us at 8MHz) - but return and loop overhead provides some of this
        __asm__ __volatile__ (
            "nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t"
            ::
        );
    }
}

// Send a byte (8 bits) MSB first
void ws2812_send_byte(uint8_t byte) {
    for (uint8_t bit = 0; bit < 8; bit++) {
        ws2812_send_bit((byte & 0x80) ? 1 : 0);
        byte <<= 1;
    }
}

// Send entire LED array in GRB format (as required by WS2812B)
void ws2812_send_array(rgb_color_t *array, uint16_t length) {
    cli();  // Disable interrupts for precise timing
    
    for (uint16_t i = 0; i < length; i++) {
        // WS2812B expects GRB order, not RGB
        ws2812_send_byte(array[i].g);  // Green first
        ws2812_send_byte(array[i].r);  // Red second  
        ws2812_send_byte(array[i].b);  // Blue third
    }
    
    sei();  // Re-enable interrupts
    
    // Reset pulse - keep line low for >50us
    PORTD &= ~(1 << LED_PIN);
    _delay_us(60);
}

// Update the LED strip
void ws2812_show(void) {
    ws2812_send_array(led_array, LED_COUNT);
}

// Set individual pixel color
void set_pixel_color(uint8_t pixel, uint8_t r, uint8_t g, uint8_t b) {
    if (pixel < LED_COUNT) {
        led_array[pixel].r = r;
        led_array[pixel].g = g;
        led_array[pixel].b = b;
    }
}

// Clear all LEDs
void clear_all(void) {
    for (uint8_t i = 0; i < LED_COUNT; i++) {
        set_pixel_color(i, 0, 0, 0);
    }
}

// Create 32-bit color value
uint32_t color(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

// Millisecond delay function
void delay_ms(uint16_t ms) {
    while (ms--) {
        _delay_ms(1);
    }
}

// Breathing effect function
void breathing_effect(uint32_t color) {
    // Fade in
    for (int brightness = 0; brightness < 255; brightness++) {
        set_strip_brightness(color, brightness);
        delay_ms(5);
    }
    
    // Fade out
    for (int brightness = 255; brightness > 0; brightness--) {
        set_strip_brightness(color, brightness);
        delay_ms(5);
    }
}

// Set brightness for entire strip
void set_strip_brightness(uint32_t color, uint8_t brightness) {
    uint8_t r = (uint8_t)((color >> 16) & 0xFF);
    uint8_t g = (uint8_t)((color >> 8) & 0xFF);
    uint8_t b = (uint8_t)(color & 0xFF);
    
    // Scale colors by brightness
    r = (r * brightness) / 255;
    g = (g * brightness) / 255;
    b = (b * brightness) / 255;
    
    // Set all pixels to the scaled color
    for (int i = 0; i < LED_COUNT; i++) {
        set_pixel_color(i, r, g, b);
    }
    ws2812_show();
}

// Color wipe effect
void color_wipe(uint32_t color, uint16_t wait) {
    uint8_t r = (uint8_t)((color >> 16) & 0xFF);
    uint8_t g = (uint8_t)((color >> 8) & 0xFF);
    uint8_t b = (uint8_t)(color & 0xFF);
    
    for (int i = 0; i < LED_COUNT; i++) {
        set_pixel_color(i, r, g, b);
        ws2812_show();
        delay_ms(wait);
    }
}

uint8_t get_char_index(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c == ' ') return 10;
    if (c == ':') return 11;
    switch (c) {
        case 'A': return 11; 
        case 'B': return 12; 
        case 'C': return 13;
        case 'D': return 14; 
        case 'E': return 15;        
        case 'G': return 16; 
        case 'H': return 17; 
        case 'I': return 18;     
        case 'L': return 19; 
        case 'M': return 20;
        case 'N': return 21; 
        case 'O': return 22; 
        case 'P': return 23;        
        case 'R': return 24;        
        case 'T': return 25;
        case 'U': return 26;
        case 'V': return 27; 
        case 'W': return 28;       
        default: return 10; // space for unknown
    }
}
void ssd1306_draw_char(uint8_t x, uint8_t page, char c) {
    if (x + 5 >= WIDTH || page >= PAGES) return;
    
    uint16_t idx = page * WIDTH + x;
    uint8_t char_idx = get_char_index(c);
    
    for (int i = 0; i < 5; i++) {
        buffer[idx + i] = font5x7[char_idx][i];
    }
    
    if (x + 5 < WIDTH) buffer[idx + 5] = 0; // Space after char
}

void ssd1306_draw_string(uint8_t x, uint8_t page, const char* s) {
    uint8_t cursor = x;
    while (*s && cursor < WIDTH - 6) {
        ssd1306_draw_char(cursor, page, *s++);
        cursor += 6;
    }
}

void ssd1306_display(void) {
    for (uint8_t page = 0; page < PAGES; page++) {
        ssd1306_command(0xB0 | page);
        ssd1306_command(0x00);
        ssd1306_command(0x10);
        
        if (!twi_start(SSD1306_I2C_ADDR >> 1, 0)) continue;
        twi_write(0x40);
        for (uint8_t col = 0; col < WIDTH; col++) {
            twi_write(buffer[page * WIDTH + col]);
        }
        twi_stop();
    }
}

//Configure INT0 (PD2) as wake-up source
void ext_int0_init(void) {
    EICRA |= (1 << ISC01);   // Falling edge triggers INT0
    EIMSK |= (1 << INT0);    // Enable INT0
    DDRD &= ~(1 << PD2);     // Set PD2 as input
    PORTD &= ~(1 << PD2);    // Ensure pull-up is disabled (since external pull-down used)
}

//sleep management
volatile bool wake_up_flag = false;

ISR(INT0_vect) {
    wake_up_flag = true; // Set flag on button press
}

void enter_sleep(void) {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sei();     // Ensure interrupts are enabled
    sleep_cpu();
    sleep_disable();  // CPU wakes up here
}

//Track inactivity
uint32_t lastActivityTime = 0;

void reset_activity_timer(void) {
    lastActivityTime = millis_get();
}

//Stress level Logic
void healthLevel(float hrvValue) {
  color_wipe(color(255, 255, 255), 20); // default reset

  if (hrvValue == 0 && bpm == 0) {
    color_wipe(color(255, 255, 255), 20); 
    ssd1306_draw_string(0, 6, "WAIT");    

  } else if (hrvValue >= 67 || bpm <= 70) {
    color_wipe(color(0, 128, 0), 20);    // Green       
    ssd1306_draw_string(0, 6, "BALANCED");    

  } else if (hrvValue >= 47 || bpm <= 80) {
    color_wipe(color(255, 255, 0), 20);  // Yellow
    ssd1306_draw_string(0, 6, "UNBALANCED");    

  } else {    
    color_wipe(color(255, 0, 0), 20);    // Red
    ssd1306_draw_string(0, 6, "LOW"); 
  }
}

/* --- Main --- */
int main(void) {
    twi_init();
    timer0_init();
    delay_ms(50);
    
    ssd1306_init();
    ssd1306_clear();
    ssd1306_display();
    
    init_ws2812();   
    clear_all();
    ws2812_show();
    breathing_effect(color(255, 255, 255));
    
    ext_int0_init();
    reset_activity_timer();

    // Reset and configure sensor
    max30102_write(REG_MODE_CONFIG, 0x40);
    delay_ms(100);
    max30102_write(REG_FIFO_CONFIG, 0x0F);
    max30102_write(REG_SPO2_CONFIG, 0x27);
    max30102_write(REG_LED1_PA, 0x32);  // ~10mA
    max30102_write(REG_LED2_PA, 0x32);  // ~10mA
    max30102_write(REG_FIFO_WR_PTR, 0x00);
    max30102_write(REG_FIFO_RD_PTR, 0x00);
    max30102_write(REG_MODE_CONFIG, 0x03);
    delay_ms(100);
    uint32_t sensorReadTime = 0;
    uint32_t displayCounter = 0;
    uint32_t ir = 0, red = 0;
    
    while (1) {
        uint32_t now = millis_get();
                
        // Read sensor every 10ms
        if (now - sensorReadTime >= 10) {
            sensorReadTime = now;
            
            max30102_read_fifo_spo2(&red, &ir);
            if (ir > 10000){
                reset_activity_timer();
                // Process beat detection
                if (checkForBeat(ir)) {
                    if (lastBeatTime != 0) {
                        uint32_t ibiInterval = now - lastBeatTime;
                        
                        // Valid heart rate range (30-200 BPM -> 300-2000ms IBI)
                        if (ibiInterval >= 300 && ibiInterval <= 2000) {
                            updateBpmAndHrv(ibiInterval);
                            validBeatCount++;
                        }
                    }
                    lastBeatTime = now;
                }
            }
        }
        // Update display every 500ms
        if (++displayCounter >= 50) {
            displayCounter = 0;
            
            char bpm_str[12];
            char hrv_str[12];
            char ir_str[12];
                        
            uint32_to_string(bpm, bpm_str);
            uint32_to_string(hrvValue, hrv_str);
            uint32_to_string(ir, ir_str);
                                 
            ssd1306_clear();
            ssd1306_draw_string(0, 0, "_____AURA LIGHT___");
            ssd1306_draw_string(0, 2, "BPM ");
            ssd1306_draw_string(0, 4, bpm_str);
            ssd1306_draw_string(80, 2, "HRV ");
            ssd1306_draw_string(80, 4, hrv_str);
            healthLevel(hrvValue);
            ssd1306_draw_string(90, 7, ir_str);
            ssd1306_display();
        }
        
        delay_ms(10);
        
        now = millis_get();
        
        if (now - lastActivityTime >= 60000UL) {  // 3 minutes
            clear_all();
            ws2812_show();
            ssd1306_clear();
            ssd1306_display();

            enter_sleep();            // Go to sleep
            reset_activity_timer();   // Reset after wake-up
        }
    } 
    
    return 0;
}

           
