
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
