
#include <stdio.h>
#include "lcd.h"
#define DAT_LINE        ((uint16_t)0x00F0)
#define E_LINE          GPIO_PIN_3
#define RS_LINE         GPIO_PIN_2

char digits[11];

void lcd_init()
{
  HAL_GPIO_WritePin( GPIOA, E_LINE, GPIO_PIN_SET ); // E line startup state
  HAL_GPIO_WritePin( GPIOA, RS_LINE, GPIO_PIN_RESET ); // command mode
  
  lcd_delay( 100000 ); // awaiting for power supply stability

  // 8-bit initialization
  GPIOA->ODR = GPIOA->ODR & ~DAT_LINE | 0x30; // 0011 0000b
  HAL_GPIO_WritePin( GPIOA, E_LINE, GPIO_PIN_RESET );
  lcd_delay( 5000 );

  lcd_pulse();
  lcd_delay( 1000 );
  
  lcd_pulse();
  lcd_delay( 1000 );
  
  // switching to 4-bit mode
  GPIOA->ODR = GPIOA->ODR & ~DAT_LINE | 0x20; // 0010 0000b
  lcd_pulse();
  lcd_delay( 40 );
  
  // function set
  lcd_command( 0x28 ); // 0010 1000b; 4-bit, 2-line, 5x8
  lcd_command( 0x0c ); // 0000 1100b; display=on, cursor=off, blinking=off
  lcd_command( 0x06 ); // 0000 0110b; increment, shift=disabled
}

void oled_init()
{
  HAL_GPIO_WritePin( GPIOA, E_LINE, GPIO_PIN_SET ); // E line startup state
  HAL_GPIO_WritePin( GPIOA, RS_LINE, GPIO_PIN_RESET ); // command mode
  
  lcd_delay( 100000 ); // awaiting for power supply stability

  // synchronization with OLED bus and display reinit
  GPIOA->ODR = GPIOA->ODR & ~DAT_LINE; // 0000 0000b
  HAL_GPIO_WritePin( GPIOA, E_LINE, GPIO_PIN_RESET );
  lcd_delay( 40 );

  lcd_pulse();
  lcd_delay( 40 );

  lcd_pulse();
  lcd_delay( 40 );

  lcd_pulse();
  lcd_delay( 40 );

  lcd_pulse();
  lcd_delay( 40 );

  // switching to 4-bit mode
  GPIOA->ODR = GPIOA->ODR & ~DAT_LINE | 0x20; // 0010 0000b
  lcd_pulse();
  lcd_delay( 40 );
  
  // function set
  lcd_command( 0x28 ); // 0010 1000b; 4-bit, 2-line, 5x8
  lcd_command( 0x0c ); // 0000 1100b; display=on, cursor=off, blinking=off
  lcd_command( 0x06 ); // 0000 0110b; increment, shift=disabled
}

void lcd_command( uint8_t command )
{
  HAL_GPIO_WritePin( GPIOA, RS_LINE, GPIO_PIN_RESET ); // command mode
  
  GPIOA->ODR = GPIOA->ODR & ~DAT_LINE | (command & 0xF0); // upper nibble
  lcd_pulse();
  
  GPIOA->ODR = GPIOA->ODR & ~DAT_LINE | ( (command & 0x0F) << 4 ); // lower nibble
  lcd_pulse();

  lcd_delay( 40 );
}

void lcd_printc( uint8_t data )
{
  HAL_GPIO_WritePin( GPIOA, RS_LINE, GPIO_PIN_SET ); // data mode
  
  GPIOA->ODR = GPIOA->ODR & ~DAT_LINE | (data & 0xF0); // upper nibble
  lcd_pulse();

  GPIOA->ODR = GPIOA->ODR & ~DAT_LINE | ( (data & 0x0F) << 4 ); // lower nibble
  lcd_pulse();

  lcd_delay( 40 );
}

void lcd_prints( const char* s )
{
  uint32_t i = 0;
  while( s[i] )
    lcd_printc( s[i++] );
}

void lcd_printd( int32_t n, uint32_t l ) // ultimately simplified sprintf
{
  uint32_t k = 0;

  if( n < 0 )
  {
    digits[0] = '-';
    k++;
    n = -n;
  }
  
  uint32_t i = l;
  while( i > k )
  {
    digits[--i] = n % 10 + 0x30;
    n = n / 10;
  }
    
  digits[l] = 0;
  lcd_prints( digits );
}

void lcd_printd2( int32_t n )
{
  if( n > 99 ) n = 99;
  if( n < -9 ) n = -9;
  
  lcd_printd(n, 2);
}

void lcd_printd3( int32_t n )
{
  if( n > 999 ) n = 999;
  if( n < -99 ) n = -99;
  
  lcd_printd(n, 3);
}

void lcd_printd4( int32_t n )
{
  if( n > 9999 ) n = 9999;
  if( n < -999 ) n = -999;

  lcd_printd(n, 4);
}

void lcd_position( uint8_t position )
{
  lcd_command( position | 0x80 );
}

void lcd_clear()
{
  lcd_command( 0x01 );
  lcd_delay( 1600 );
}

void lcd_pulse() {
  HAL_GPIO_WritePin( GPIOA, E_LINE, GPIO_PIN_SET );
  asm( "nop \n nop \n nop \n nop \n nop \n nop \n nop \n nop" );
  HAL_GPIO_WritePin( GPIOA, E_LINE, GPIO_PIN_RESET );
}
