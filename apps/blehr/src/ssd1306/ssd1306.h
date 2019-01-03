/**
 * This Library was originally written by Olivier Van den Eede (4ilo) in 2016.
 * Some refactoring was done and SPI support was added by Aleksander Alekseev (afiskon) in 2018.
 * Ported to Apache mynewt by arglurgl in 2018
 * Original library can be found here:
 * https://github.com/afiskon/stm32-ssd1306
 */

#ifndef __SSD1306_H__
#define __SSD1306_H__

#include "ssd1306_fonts.h"


/* vvv SPI config vvv */

#ifndef SSD1306_SPI_Nr
#define SSD1306_SPI_Nr          0
#endif

#ifndef SSD1306_CS_Pin
#define SSD1306_CS_Pin          29
#endif

#ifndef SSD1306_DC_Pin
#define SSD1306_DC_Pin          0
#endif

#ifndef SSD1306_Reset_Pin
#define SSD1306_Reset_Pin       30
#endif

#ifndef SSD1306_SPI_Baudrate
#define SSD1306_SPI_Baudrate    8000
#endif

/* ^^^ SPI config ^^^ */

// SSD1306 OLED height in pixels
#ifndef SSD1306_HEIGHT
#define SSD1306_HEIGHT          32
#endif

// SSD1306 width in pixels
#ifndef SSD1306_WIDTH
#define SSD1306_WIDTH           64
#endif

// some LEDs don't display anything in first two columns
// #define SSD1306_WIDTH           130

// Enumeration for screen colors
typedef enum {
    Black = 0x00, // Black color, no pixel
    White = 0x01  // Pixel is set. Color depends on OLED
} SSD1306_COLOR;

// Struct to store transformations
typedef struct {
    uint16_t CurrentX;
    uint16_t CurrentY;
    uint8_t Inverted;
    uint8_t Initialized;
} SSD1306_t;

// Procedure definitions
void ssd1306_Init(void);
void ssd1306_Fill(SSD1306_COLOR color);
void ssd1306_UpdateScreen(void);
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color);
char ssd1306_WriteChar(char ch, FontDef Font, SSD1306_COLOR color);
char ssd1306_WriteString(char* str, FontDef Font, SSD1306_COLOR color);
void ssd1306_SetCursor(uint8_t x, uint8_t y);

// Low-level procedures
void ssd1306_Reset(void);
void ssd1306_WriteCommand(uint8_t byte);
void ssd1306_WriteData(uint8_t* buffer, unsigned long buff_size);

#endif // __SSD1306_H__
