#include "ssd1306.h"
#include <string.h>

static I2C_HandleTypeDef *s_hi2c;
static uint8_t s_buf[SSD1306_WIDTH * SSD1306_HEIGHT / 8]; /* 1024-byte framebuffer */

/* Minimal 5x7 font: space, '.', '-', ':', digits, and the capital letters
   actually used by this project's status text. Each glyph is 5 columns,
   7 rows tall, stored as 5 bytes (bit0 = top row). */
typedef struct { char c; uint8_t col[5]; } Glyph;

static const Glyph FONT[] = {
    {' ', {0x00,0x00,0x00,0x00,0x00}},
    {'.', {0x00,0x60,0x60,0x00,0x00}},
    {'-', {0x08,0x08,0x08,0x08,0x08}},
    {':', {0x00,0x36,0x36,0x00,0x00}},
    {'0', {0x3E,0x51,0x49,0x45,0x3E}},
    {'1', {0x00,0x42,0x7F,0x40,0x00}},
    {'2', {0x62,0x51,0x49,0x49,0x46}},
    {'3', {0x22,0x49,0x49,0x49,0x36}},
    {'4', {0x18,0x14,0x12,0x7F,0x10}},
    {'5', {0x2F,0x49,0x49,0x49,0x31}},
    {'6', {0x3C,0x4A,0x49,0x49,0x30}},
    {'7', {0x01,0x71,0x09,0x05,0x03}},
    {'8', {0x36,0x49,0x49,0x49,0x36}},
    {'9', {0x06,0x49,0x49,0x29,0x1E}},
    {'A', {0x7E,0x11,0x11,0x11,0x7E}},
    {'C', {0x3E,0x41,0x41,0x41,0x22}},
    {'D', {0x7F,0x41,0x41,0x22,0x1C}},
    {'E', {0x7F,0x49,0x49,0x49,0x41}},
    {'F', {0x7F,0x09,0x09,0x09,0x01}},
    {'H', {0x7F,0x08,0x08,0x08,0x7F}},
    {'I', {0x00,0x41,0x7F,0x41,0x00}},
    {'L', {0x7F,0x40,0x40,0x40,0x40}},
    {'M', {0x7F,0x02,0x0C,0x02,0x7F}},
    {'N', {0x7F,0x04,0x08,0x10,0x7F}},
    {'O', {0x3E,0x41,0x41,0x41,0x3E}},
    {'Q', {0x3E,0x41,0x51,0x21,0x5E}},
    {'R', {0x7F,0x09,0x19,0x29,0x46}},
    {'S', {0x46,0x49,0x49,0x49,0x31}},
    {'T', {0x01,0x01,0x7F,0x01,0x01}},
    {'U', {0x3F,0x40,0x40,0x40,0x3F}},
    {'V', {0x1F,0x20,0x40,0x20,0x1F}},
    {'Z', {0x61,0x51,0x49,0x45,0x43}},
};
#define FONT_N (sizeof(FONT)/sizeof(FONT[0]))

static const uint8_t *glyph_for(char c)
{
    for (unsigned i = 0; i < FONT_N; i++)
        if (FONT[i].c == c) return FONT[i].col;
    return FONT[0].col; /* unknown char -> blank */
}

static HAL_StatusTypeDef cmd(uint8_t c)
{
    uint8_t pkt[2] = {0x00, c}; /* 0x00 = command stream */
    return HAL_I2C_Master_Transmit(s_hi2c, SSD1306_I2C_ADDR, pkt, 2, 100);
}

void SSD1306_Init(I2C_HandleTypeDef *hi2c)
{
    s_hi2c = hi2c;
    HAL_Delay(100);
    cmd(0xAE);               /* display off */
    cmd(0xD5); cmd(0x80);    /* clock divide */
    cmd(0xA8); cmd(0x3F);    /* multiplex ratio 64 */
    cmd(0xD3); cmd(0x00);    /* no display offset */
    cmd(0x40);                /* start line 0 */
    cmd(0xAD); cmd(0x8B);    /* charge pump (SH1106-style); SSD1306 modules
                                  generally accept this too, but if your
                                  screen stays blank on a real SSD1306 try
                                  0x8D / 0x14 instead */
    cmd(0xA1);                /* segment remap */
    cmd(0xC8);                /* COM scan direction */
    cmd(0xDA); cmd(0x12);    /* COM pins */
    cmd(0x81); cmd(0x80);    /* contrast */
    cmd(0xD9); cmd(0xF1);    /* pre-charge */
    cmd(0xDB); cmd(0x40);    /* VCOMH deselect */
    cmd(0xA4);                 /* resume RAM content display */
    cmd(0xA6);                 /* normal (not inverted) display */
    cmd(0xAF);                 /* display on */
    SSD1306_Clear();
    SSD1306_UpdateScreen();
}

void SSD1306_Clear(void)
{
    memset(s_buf, 0, sizeof(s_buf));
}

/* Page addressing mode: works on both SSD1306 and SH1106, one 8-row page
   at a time, since SH1106 does not support horizontal addressing mode. */
void SSD1306_UpdateScreen(void)
{
    for (uint8_t page = 0; page < SSD1306_HEIGHT / 8; page++) {
        cmd(0xB0 + page);                                   /* set page address        */
        uint8_t col = SSD1306_COL_OFFSET;
        cmd(0x00 | (col & 0x0F));                            /* set lower column nibble */
        cmd(0x10 | ((col >> 4) & 0x0F));                      /* set upper column nibble */

        uint8_t pkt[17];
        pkt[0] = 0x40; /* data stream */
        for (uint16_t i = 0; i < SSD1306_WIDTH; i += 16) {
            uint8_t chunk = (SSD1306_WIDTH - i) < 16 ? (SSD1306_WIDTH - i) : 16;
            memcpy(&pkt[1], &s_buf[page * SSD1306_WIDTH + i], chunk);
            HAL_I2C_Master_Transmit(s_hi2c, SSD1306_I2C_ADDR, pkt, chunk + 1, 100);
        }
    }
}

void SSD1306_DrawString(uint8_t x, uint8_t page, const char *str)
{
    if (page > 7) return;
    while (*str && x < SSD1306_WIDTH - 6) {
        const uint8_t *g = glyph_for(*str);
        uint16_t base = page * SSD1306_WIDTH + x;
        for (int i = 0; i < 5; i++)
            s_buf[base + i] = g[i];
        s_buf[base + 5] = 0x00; /* 1px spacing */
        x += 6;
        str++;
    }
}
