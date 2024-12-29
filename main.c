/*
MAIN FOR VORTICULAR SOVEREIGNTY 

Author: Luke Phillips
*/


#include "bldc_lib.c"


int main(void){

   // initialize chip 
   __init__();
   tim3_ch_en(1, 0, 0);
   tim3_pwm_freq(0x0AFF);
   tim3_pwm_duty(0xFFFE, 0x7FFF, 0x7FFF);

    // main loop
    while(1){      

      tim3_ch_en(0, 0, 0);
      delay(1000);
      tim3_ch_en(1, 0, 0);
      delay(1000);

      /* implement FOC controls  outer loop -> Position: PID with input approximated from fused hall and encoder data
                                 middle loop -> Speed: PID with input of derivative of position approximation
                                 inner loop -> torque: following steps
                                 
      
      
        1. read each phase hall sensors / voltage and encoder position
           use kalman filter to fuse hall and encoder data and approximate theta

        2. clarke transform
 
            i_alpha = ia - 1/2 ib - 1/2 ic  
            i_beta  = 0 + sqrt(3)/2 ib - sqrt(3)/2 ic 

        3. parke transform

            id = i_alpha cos(theta) + i_beta sin(theta)
            iq = -i_alpha sin(theta) + i_beta cos(theta)  

        4. Torque PI controller use id and iq as input 
           id_ref should be 0 
           iq_ref comes from speed controller

           output of torque controller: uq and ud

        5. inverse parke transform
           input from uq, ud, and theta

           u_alpha = id cos(theta) - iq sin(theta)
           u_beta = id sin(theta) + iq cos(theta)

        6. inverse clarke transform 
           
           ua = u_alpha
           ub = -1/2 u_alpha + sqrt(3)/2 u_beta
           uc = -1/2 u_alpha - sqrt(3)/2 u_beta 

        6. 3phase pwm voltage inverter

           use ua, ub, and uc as the inputs
             
      */

    }

    return 0;
}


//_____________________________________________________________________________________________//


// Startup code
__attribute__((naked, noreturn)) void _reset(void) {
  // memset .bss to zero, and copy .data section to RAM region
  extern long _sbss, _ebss, _sdata, _edata, _sidata;
  for (long *dst = &_sbss; dst < &_ebss; dst++) *dst = 0;
  for (long *dst = &_sdata, *src = &_sidata; dst < &_edata;) *dst++ = *src++;

  main();             // Call main()
  for (;;) (void) 0;  // Infinite loop in the case if main() returns
}

extern void _estack(void);  // Defined in link.ld

// 16 standard and 32 STM32-specific handlers
__attribute__((section(".vectors"))) void (*const tab[16 + 32])(void) = {
    _estack, _reset, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, SysTick_Handler};
