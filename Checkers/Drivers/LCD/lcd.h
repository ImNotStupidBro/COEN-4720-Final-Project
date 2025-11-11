#ifndef __ST7789_H__
#define __ST7789_H__

#include "images.h"
#include "ugui.h"
#include "main.h"
#include <stdio.h>

/* For demo only. Minimum is 32, 128 and higher will enable all tests */
#define DEMO_FLASH_KB 64 // 64

/* choose a Hardware SPI port to use. */
#define LCD_HANDLE          hspi1
extern SPI_HandleTypeDef    LCD_HANDLE;

/* Pin connections. Use same names as in CubeMX */
#define LCD_DC               LCD_DC
#define LCD_RST              LCD_RST /* Disable if your display has no RST pin */
#define LCD_CS               LCD_CS  /* Disable if your display has no CS pin */
//#define LCD_BL             LCD_BL  /* Enable if you need backlight control */

//#define USE_DMA                    /* Use DMA for transfers when possible */
//#define LCD_LOCAL_FB               /* Use local framebuffer. Needs a lot of ram, but removes flickering and redrawing glitches */

//#define USE_ST7735                 /* LCD Selection */
#define USE_ST7789

#define LCD_ROTATION 2               /* XY rotation/mirroring. Valid values: 0...3 */

#ifdef USE_ST7789                    /* ST7789 LCD sizes */
  //#define LCD_135X240
  //#define LCD_240X240
  #define LCD_240X320
#endif

#ifdef USE_ST7789
  #define LCD_X_SHIFT 0
  #define LCD_Y_SHIFT 0
  #ifdef LCD_135X240
    #if (LCD_ROTATION == 0) || (LCD_ROTATION == 2)
      #define LCD_WIDTH  135
      #define LCD_HEIGHT 240
    #elif (LCD_ROTATION == 1) || (LCD_ROTATION == 3)
      #define LCD_WIDTH  240
      #define LCD_HEIGHT 135
    #endif
  #elif defined LCD_240X240
    #define LCD_WIDTH  240
    #define LCD_HEIGHT 240
  #elif defined LCD_240X320
    #if (LCD_ROTATION == 0) || (LCD_ROTATION == 2)
      #define LCD_WIDTH  240
      #define LCD_HEIGHT 320
    #elif (LCD_ROTATION == 1) || (LCD_ROTATION == 3)
      #define LCD_WIDTH  320
      #define LCD_HEIGHT 240
    #endif
  #endif

  #if LCD_ROTATION == 0
    #define LCD_ROTATION_CMD (CMD_MADCTL_MX | CMD_MADCTL_MY | CMD_MADCTL_RGB)
  #elif LCD_ROTATION == 1
    #define LCD_ROTATION_CMD (CMD_MADCTL_MY | CMD_MADCTL_MV | CMD_MADCTL_RGB)
  #elif LCD_ROTATION == 2
    #define LCD_ROTATION_CMD (CMD_MADCTL_RGB)
  #elif LCD_ROTATION == 3
    #define LCD_ROTATION_CMD (CMD_MADCTL_MX | CMD_MADCTL_MV | CMD_MADCTL_RGB)
  #endif
#endif

/* LCD Commands */
typedef enum{
  CMD_MADCTL_MY  = 0x80,
  CMD_MADCTL_MX  = 0x40,
  CMD_MADCTL_MV  = 0x20,
  CMD_MADCTL_ML  = 0x10,
  CMD_MADCTL_RGB = 0x00,
  CMD_MADCTL_BGR = 0x08,
  CMD_MADCTL_MH  = 0x04,
  CMD_NOP        = 0x00,
  CMD_SWRESET    = 0x01,
  CMD_RDDID      = 0x04,
  CMD_RDDST      = 0x09,
  CMD_SLPIN      = 0x10,
  CMD_SLPOUT     = 0x11,
  CMD_PTLON      = 0x12,
  CMD_NORON      = 0x13,
  CMD_INVOFF     = 0x20,
  CMD_INVON      = 0x21,
  CMD_GAMSET     = 0x26,
  CMD_DISPOFF    = 0x28,
  CMD_DISPON     = 0x29,
  CMD_CASET      = 0x2A,
  CMD_RASET      = 0x2B,
  CMD_RAMWR      = 0x2C,
  CMD_RAMRD      = 0x2E,
  CMD_PTLAR      = 0x30,
  CMD_MADCTL     = 0x36,
  CMD_IDMOFF     = 0x38,
  CMD_IDMON      = 0x39,
  CMD_COLMOD     = 0x3A,
  CMD_RAMCTRL    = 0xB0,
  CMD_FRMCTR1    = 0xB1,
  CMD_RGBCTRL    = 0xB1,
  CMD_FRMCTR2    = 0xB2,
  CMD_PORCTRL    = 0xB2,
  CMD_FRMCTR3    = 0xB3,
  CMD_FRCTRL1    = 0xB3,
  CMD_INVCTR     = 0xB4,
  CMD_PARCTRL    = 0xB5,
  CMD_DISSET5    = 0xB6,
  CMD_GCTRL      = 0xB7,
  CMD_VCOMS      = 0xBB,
  CMD_PWCTR1     = 0xC0,
  CMD_LCMCTRL    = 0xC0,
  CMD_PWCTR2     = 0xC1,
  CMD_IDSET      = 0xC1,
  CMD_PWCTR3     = 0xC2,
  CMD_VDVVRHEN   = 0xC2,
  CMD_PWCTR4     = 0xC3,
  CMD_VRHS       = 0xC3,
  CMD_PWCTR5     = 0xC4,
  CMD_VDVS       = 0xC4,
  CMD_VMCTR1     = 0xC5,
  CMD_VCMOFSET   = 0xC5,
  CMD_FRCTRL2    = 0xC6,
  CMD_PWCTRL1     = 0xD0,
  CMD_RDID1      = 0xDA,
  CMD_RDID2      = 0xDB,
  CMD_RDID3      = 0xDC,
  CMD_RDID4      = 0xDD,
  CMD_PWCTR6     = 0xFC,
  CMD_GMCTRP1    = 0xE0,
  CMD_GMCTRN1    = 0xE1,
  CMD_COLOR_MODE_16bit = 0x55,
  CMD_COLOR_MODE_18bit = 0x66,
}lcd_cmds;

#define ABS(x) ((x) > 0 ? (x) : -(x))

// stuff related to set or reset pins connected to RST, DC, and CS of the
// LCD display module;
#define SCK_Pin GPIO_PIN_3
#define SCK_GPIO_Port GPIOB // PB3
#define SDO_Pin GPIO_PIN_5
#define SDO_GPIO_Port GPIOB
#define LCD_DC_Pin GPIO_PIN_8
#define LCD_DC_GPIO_Port GPIOA // PA8
#define LCD_RST_Pin GPIO_PIN_7
#define LCD_RST_GPIO_Port GPIOC // PC7
#define LCD_CS_Pin GPIO_PIN_6
#define LCD_CS_GPIO_Port GPIOB // PB6
#define LCD_RST_Clr() HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET)
#define LCD_RST_Set() HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET)
#define LCD_DC_Clr() HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET)
#define LCD_DC_Set() HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET)
#define LCD_CS_Clr() HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET)
#define LCD_CS_Set() HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET)


void LCD_SetRotation(uint8_t m);
void LCD_DrawPixel(int16_t x, int16_t y, uint16_t color);
void LCD_DrawPixelFB(int16_t x, int16_t y, uint16_t color);
int8_t LCD_Fill(uint16_t xSta, uint16_t ySta, uint16_t xEnd, uint16_t yEnd, uint16_t color);

/* Graphical functions. */
void LCD_DrawImage(uint16_t x, uint16_t y, UG_BMP* bmp);
int8_t LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void LCD_InvertColors(uint8_t invert);

/* Text functions. */
void LCD_PutChar(uint16_t x, uint16_t y, char ch, UG_FONT* font, uint16_t color, uint16_t bgcolor);
void LCD_PutStr(uint16_t x, uint16_t y,  char *str, UG_FONT* font, uint16_t color, uint16_t bgcolor);

/* Extended Graphical functions. */
/* Command functions */
void LCD_TearEffect(uint8_t tear);
void LCD_setPower(uint8_t power);

/* Initialization. */
void LCD_init(void);

/* Simple test function. */
void LCD_Test(void);

#endif // __ST7789_H__
