
#include "stm32f0xx.h"

void lcd_init();
void oled_init();
void lcd_command( uint8_t );
void lcd_printc( uint8_t );
void lcd_prints( const char* );
void lcd_printd2( int32_t );
void lcd_printd3( int32_t );
void lcd_printd4( int32_t );
void lcd_printd( int32_t, uint32_t );
void lcd_position( uint8_t );
void lcd_clear();
void lcd_pulse();
extern void lcd_delay( uint32_t delay );

// Support for the ER-OLEDM1602-4Y OLED from Buydisplay.
// In lieu of the LED on PB1, PB1 must be connected to
// RESET. CS should be tied to ground.
// Jumper settings: BS2 HIGH BS1 LOW BS0 HIGH
//
// Pin mapping:
// VCC - 3.3V
// RD - E
// RW - R/W
// DC - RS
// RES - PB1
// CS - GND
// D7-D4 to D7-D4 straight.
//
// I placed an extra cap VCC-GND on the OLED board, this
// might not be necessary.
// Note that programming may fail if the display is connected
// (unsure why)
//
// 3.3V current draw ~65mA continuous. Suggest maybe
// substitute a different 3.3V regulator for more margin?

// Define for ER-OLEDM1602-4Y, undef for WEH001602AGPP5N00001
#define OLED_EASTRISING
