  MODULE lcd_delay
  PUBLIC lcd_delay
  SECTION CODE:CODE

#define CPU_FREQ    27120000

lcd_delay:          // R0 - delay, microseconds
  push {R0-R1, LR}
  
  ldr R1, multiplier
  muls R0, R0, R1
  lsrs R0, R0, #8
  
loop:
  subs R0, #1
  bne loop
  
  pop {R0-R1, PC}

  DATA
multiplier:     dc32    CPU_FREQ / 4 * 256 / 1000000

  END