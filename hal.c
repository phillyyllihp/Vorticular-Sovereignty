/*
HARDWARE ABSTRACTION LAYER FOR STM32G070RB NUCLEO BOARD
SHOULD WORK WITH OTHER STM32 CHIPS WITH A LITTLE MODIFICATION

Autor: Luke Phillips
*/

#include <inttypes.h>
#include <stdbool.h>

#define BIT(x) (1UL << (x))
#define PIN(bank, num) ((((bank) - 'A') << 0x8) | (num))
#define PINNO(pin) (pin & 0xFF)
#define PINBANK(pin) (pin >> 0x8)

/*_____________________________________________________________________________________________

                                GPIO INTERACTIONS

-----------------------------------------------------------------------------------------------*/

// Define GPIO Peripheral Registers
struct gpio {
    volatile uint32_t MODER, OTYPER, OSSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFRL, AFRH, BRR;
};
// memory location and offset for gpio banks 
#define GPIO(bank) ((struct gpio *) (0x50000000 + 0x0400 * (bank)))

// Enumerate GPIO Mode values as per the data sheet 
enum {GPI_MODE, GPO_MODE, AF_MODE, ANALOG_MODE};

// function to set GPIO modes
static inline void gpio_set_mode(uint16_t pin, uint8_t gpio_mode) {
  struct gpio *gpio = GPIO(PINBANK(pin));           // GPIO Bank
  int n = PINNO(pin);                               // Pin Number
  gpio->MODER &= ~(3U << (n * 2));                  // Clear existing setting
  gpio->MODER |= (gpio_mode & 3) << (n * 2);        // Set new mode
}

// enumerate alternate functions
enum {AF0, AF1, AF2, AF3, AF4, AF5, AF6, AF7};

// function to set GPIO alternate function 
static inline void gpio_set_af(uint16_t pin, uint8_t af) {
  struct gpio *gpio = GPIO(PINBANK(pin));
  int n = PINNO(pin);
  if (n <= 7) {
    gpio->AFRL &= ~(15U << (n * 4));          // clear previous setting 
    gpio->AFRL |= (af & 7) << (n * 4);        // write new setting 
  }
  else {
    gpio->AFRH &= ~(15U << ((n-8) * 4));      // clear previous setting 
    gpio->AFRH |= (af & 7) << ((n-8) * 4);    // write new setting 
  }
}

// function to write gpio logic level to ODR thru BSRR
static inline void gpio_write(uint16_t pin, bool val) {
  struct gpio *gpio = GPIO(PINBANK(pin));                     // select gpio bank
  gpio->BSRR = (1U << PINNO(pin)) << (val ? 0 : 16);          // shift 1 left to pin number in reset side. if value is 1 shift left another 16 to set side
}

// function to get input value of a pin through IDR
bool gpio_read(uint16_t pin) {
  struct gpio *gpio = GPIO(PINBANK(pin));
  bool pin_val = gpio->IDR & BIT(PINNO(pin));
  return pin_val;
}

/*_____________________________________________________________________________________________

                            Control and setup RCC

------------------------------------------------------------------------------------------------*/


// Define RCC (Reset and Clock Control) registers 
struct rcc {
    volatile uint32_t CR, ICSCR, CFGR, PLLCFGR, RESERVED0[2], CIER, CIFR, CICR, IOPRSTR,
        AHBRSTR, APBRSTR1, APBRSTR2, IOPENR, AHBENR, APBENR1, APBENR2, IOPSMENR, AHBSMENR, 
        APBSMENR1, APBSMENR2, CCIPR, CCIPR2, BDCR, CSR;
};
// memory location of RCC
#define RCC ((struct rcc *) 0x40021000)

// enum gpio ports
enum {GPIO_A, GPIO_B, GPIO_C, GPIO_D, GPIO_E, GPIO_F};

// function to enable or disable IO port clock
static inline void io_port_en(uint16_t bank, bool val) {
  RCC->IOPENR &= ~(1U << bank);           // clear existing setting
  RCC->IOPENR |= (val << bank);           // apply new setting
}

/*_____________________________________________________________________________________________

                        SysTick and Timing Functions

-----------------------------------------------------------------------------------------------*/


// create a structure for the systick register
struct stk {
  volatile uint32_t CSR, RVR, CVR, CALIB;
};
#define STK ((struct stk *) 0xE000E010)           // location of the systick register

// function to initiate the ARM systick timer PM0223 datasheet
// set ticks to 16000000 / 1000 for 1kHz ticks
static inline void SysTick_init(uint32_t ticks) {
  if ((ticks-1) > 0xFFFFFF) return;               // limited to 24 bit timer
  STK->RVR = ticks-1;                             // set the value to be loaded 
  STK->CVR = 0;                                   // reset the current value to 0
  STK->CSR = BIT(0) | BIT(1) | BIT(2);            // set the enable bit to 1
  RCC->APBENR2 |= BIT(0);                         // Enable the SYSCFG clock
}

// function to handle SysTick interupt
static volatile uint32_t s_ticks;
void SysTick_Handler(void) {
  s_ticks++;
}

// depends on init ticks input
void delay(unsigned ms) {
  uint32_t wait = s_ticks + ms;                   // time to wait 
  while(s_ticks < wait) (void) 0;                 // wait until its time    
}

// t: expiration time, prd: period, now: current time. Return true if expired
bool timer_expired(uint32_t *t, uint32_t prd, uint32_t now) {
  if (now + prd < *t) *t = 0;                    // Time wrapped? Reset timer
  if (*t == 0) *t = now + prd;                   // First poll? Set expiration
  if (*t > now) return false;                    // Not expired yet, return
  *t = (now - *t) > prd ? now + prd : *t + prd;  // Next expiration time
  return true;                                   // Expired, return true
}

/*_____________________________________________________________________________________________

                                Timer 1 Setup 

-----------------------------------------------------------------------------------------------*/

struct tim1 {
  volatile uint32_t CR1, CR2, SMCR, DIER, SR, EGR, CCMR1, CCMR2, CCER, CNT, PSC,
      ARR, RCR, CCR1, CCR2, CCR3, CCR4, BDTR, DCR, DMAR, RES, CCMR3, CCR5, CCR6, AF1, AF2, TISEL;
};
#define TIM1 ((struct tim1 *) 0x40012C00)       // TIM1 register address

// function to change the Auto reload register (ARR) - used for setting PWM frequency 
static inline void tim1_pwm_freq(uint16_t freq) {
  TIM1->ARR = freq;                        // pass new setting into register
  TIM1->EGR |= BIT(0);                     // generate update event
  TIM1->EGR &= ~BIT(0);                    // clear UG bit 
}

// funtion to change duty cycles for pwm channels 1, 2, & 3
// OCxPE bit in the CCMRx register must be set for these to update at update events
static inline void tim1_pwm_duty(uint16_t c1_duty, uint16_t c2_duty, uint16_t c3_duty) {
  TIM1->CCR1 = c1_duty;                    // pass new setting into register
  TIM1->CCR2 = c2_duty;                    // pass new setting into register
  TIM1->CCR3 = c3_duty;                    // pass new setting into register
  TIM1->EGR |= BIT(0);                     // generate update event
  TIM1->EGR &= ~BIT(0);                    // clear UG bit 
}

// function to change the output states of each channel
static inline void tim1_ch_en(bool a, bool b, bool c) {
  uint16_t ccer = 0x0000;                // variable to store the enable bits for the ccer register
  ccer |= (a << 0);                      // shift the values for a b and c into the dummy variable
  ccer |= (b << 4);
  ccer |= (c << 8);
  TIM1->CCER &= 0xEEEE;                   // clear exisiting 
  TIM1->CCER |= ccer;                     // set new enable values
}


// function to initialize PMW output on channels 1, 2 & 3
// channel ports are set when initializing the gpio's 
static inline void __tim1_pwm_init__(void) {
  uint16_t max_psc = 16000000/250000-1;    // internal oscillator runs at 16MHz / desired switching speed for transistors in H-Bridge
  RCC->APBENR2 |= BIT(11);                 // enable timer1 peripheral clock 
  RCC->APBRSTR2 |= BIT(11);                // reset timer1 
  RCC->APBRSTR2 &= ~BIT(11);               // clear timer1 reset bit
  TIM1->CR1 |= BIT(7);                     // enable auto reload preload register
  TIM1->CCMR1 |= (0b1101 << 3);            // set channel 1 to OC PWM1 mode and enable the OC1 preload register
  TIM1->CCMR1 |= (0b1101 << 11);           // set channel 2 to OC PWM1 mode enable the OC2 preload register
  TIM1->CCMR2 |= (0b1101 << 3);            // set channel 3 to OC PWM1 mode and enable the OC3 preload register
  TIM1->PSC = max_psc;                     // set prescaler to 64 (250kHz)  
  TIM1->CR1 |= BIT(0);                     // enable counter
  tim1_pwm_freq(0x7FFF);                   // set default frequency to 1/2 of prescaled clock
  tim1_pwm_duty(0x7FFF, 0x7FFF, 0x7FFF);   // set default duty cycle to 50%
}


/*_____________________________________________________________________________________________

                                Timer 3 Setup 

-----------------------------------------------------------------------------------------------*/

struct tim3 {
  volatile uint32_t CR1, CR2, SMCR, DIER, SR, EGR, CCMR1, CCMR2, CCER, CNT, PSC,
      ARR, BLANK1, CCR1, CCR2, CCR3, CCR4, BLANK2, DCR, DMAR, AF1, BLANK3, TISEL;
};
#define TIM3 ((struct tim3 *) 0x40000400)

// function to change the Auto reload register (ARR) - used for setting PWM frequency 
static inline void tim3_pwm_freq(uint16_t freq) {
  TIM3->ARR = freq;                       // pass new setting into register
  TIM3->EGR |= BIT(0);                     // generate update event
  TIM3->EGR &= ~BIT(0);                    // clear UG bit 
}

// funtion to change duty cycles for pwm channels 1, 2, & 3
// OCxPE bit in the CCMRx register must be set for these to update at update events
static inline void tim3_pwm_duty(uint16_t c1_duty, uint16_t c2_duty, uint16_t c3_duty) {
  TIM3->CCR1 = c1_duty;                    // pass new setting into register
  TIM3->CCR2 = c2_duty;                    // pass new setting into register
  TIM3->CCR3 = c3_duty;                    // pass new setting into register
  TIM3->EGR |= BIT(0);                     // generate update event
  TIM3->EGR &= ~BIT(0);                    // clear UG bit 
}

// function to change the output states of each channel
static inline void tim3_ch_en(bool a, bool b, bool c) {
  uint16_t ccer = 0x0000;                // variable to store the enable bits for the ccer register
  ccer |= (a << 0);                      // shift the values for a b and c into the dummy variable
  ccer |= (b << 4);
  ccer |= (c << 8);
  TIM3->CCER &= 0xEEEE;                   // clear exisiting 
  TIM3->CCER |= ccer;                     // set new enable values
}

// function to initialize PMW output on channels 1, 2 & 3
// channel ports are set when initializing the gpio's 
static inline void __tim3_pwm_init__(void) {
  uint16_t max_psc = 16000000/250000-1;    // internal oscillator runs at 16MHz / desired switching speed for transistors in H-Bridge
  RCC->APBENR1 |= BIT(1);                  // enable timer3 peripheral clock 
  RCC->APBRSTR1 |= BIT(1);                 // reset timer3 
  RCC->APBRSTR1 &= ~BIT(1);                // clear timer3 reset bit
  TIM3->CR1 |= BIT(7);                     // enable auto reload preload register
  TIM3->CCMR1 |= (0b1101 << 3);            // set channel 1 to OC PWM1 mode and enable the OC1 preload register
  TIM3->CCMR1 |= (0b1101 << 11);           // set channel 2 to OC PWM1 mode enable the OC2 preload register
  TIM3->CCMR2 |= (0b1101 << 3);            // set channel 3 to OC PWM1 mode and enable the OC3 preload register
  TIM3->PSC = max_psc;                     // set prescaler to 64 (250kHz)  
  TIM3->CR1 |= BIT(0);                     // enable counter
  tim3_pwm_freq(0x7FFF);                   // set default frequency to 1/2 of prescaled clock
  tim3_pwm_duty(0x7FFF, 0x7FFF, 0x7FFF);   // set default duty cycle to 50%
}

/*_____________________________________________________________________________________________

                                ADC

------------------------------------------------------------------------------------------------*/

// create a structure for the adc registers
struct adc {
  volatile uint32_t ISR, IER, CR, CFGR1, CFGR2, SMPR, BLANK1[2], AWD1TR, AWD2TR, CHSELR0,
      CHSELR1, AWD3TR, BLANK2[4], DR, BLANK3[24], AWD2CR, AWD3CR, BLANK4[4], CALFACT, BLANK4[149], CCR
};
#define ADC ((struct adc *) 0x40010080)

// function to calibrate the adc
// the adc must have the voltage regulator enabled and stable, the adc dissabled, and the DMA dissabled
static inline void adc_calibrate(bool* calibrated, uint8_t* CAL_fact) {
  bool ADEN_bit, ADVREGEN_bit, DMAEN_bit, EOCAL;
  ADC->ISR |= BIT(11);                              // clear the end of callibration flag bit - EOCAL 
  ADEN_bit = ADC->CR & BIT(0);                     // read the adc en bit
  ADVREGEN_bit = ADC->CR & BIT(28);                 // read the adc voltage regulator en bit
  DMAEN_bit = ADC->CFGR1 & BIT(0);                  // read the DMA en bit
  EOCAL = ADC->ISR & BIT(11);                       // read the EOCAL bit

  // check that the adc is not enabled, the regulator is on, and the DMAEN is not enabled
  if (~ADEN_bit & ADVREGEN_bit & ~DMAEN_bit) {
    ADC->CR |= BIT(31);                             // begin the calibration 
    // wait until end of calibration flag bit is set
    while(~EOCAL){
      EOCAL = ADC->ISR & BIT(11);                   // read the EOCAL bit
    }
    ADC->ISR |= BIT(11);                            // clear the end of callibration flag bit - EOCAL
    *CAL_fact = ADC->CALFACT & 0x3F;                // return the 6lsb's of calfact reg. 
    *calibrated = true;                             // return that the calibration was complete
  }
  else {
    *calibrated = false;                            // return that the calibration was incomplete
  }
}

// function to initialize and calibrate the adc
static inline void __adc_init__(void) {
  bool adc_cal_check;
  uint8_t adc_cal_fact;
  ADC->CR |= BIT(28);                               // enable the internal voltage regulator in the cr register
  delay(1);                                         //delay 1ms - minimum 20us spec'd in datasheet
  adc_calibrate(&adc_cal_check, &adc_cal_fact);     //calibrate (finished when adc cal bit is cleared by hardware)
}

// funtion to enable the adc 
// the adc should have the voltage regulator enabled and stable as well as be calibrated before using this function 
static inline void adc_enable(void) {
  bool ADRDY;                            
  ADC->ISR |= BIT(0);                             // clear the ADRDY bit
  ADC->CR |= BIT(0);                              // set the adc enable bit
  ADRDY = ADC->ISR & BIT(0);                      // read the ADC ready flag
  // wait for the ADC ready flag to be set
  while(~ADRDY) {
    ADRDY = ADC->ISR & BIT(0);                    // read the adc ready flag until set
  }
}

// function to disable the adc
// the adc should have the voltage regulator enabled and stable as well as be calibrated before usign this function 
static inline void adc_disable(void) {
  bool ADSTART, ADSTP, ADDIS;
  ADSTART = ADC->CR & BIT(2);
  if (ADSTART) {
    ADC->CR |= BIT(4);                            // stop the adc conversion process
    ADSTP = ADC->CR & BIT(4);                     // read the ADSTP bit
    // wait for hardware to clear the ADSTP bit
    while(ADSTP) {
      ADSTP = ADC->CR & BIT(4);                     // read the ADSTP bit
    }
  }
  ADC->CR |= BIT(1);                              // set the ADC dissable bit
  ADDIS = ADC->CR & BIT(1);                       // read the ADC dissable bit
  while (ADDIS) {
    ADDIS = ADC->CR & BIT(1);                       // read the ADC dissable bit unit cleared
  }
}


/*_____________________________________________________________________________________________

                                INTERRUPTS

------------------------------------------------------------------------------------------------*/

// create a structure for the NVIC registers
struct nvic {
  volatile uint32_t ISER, ICER, ISPR, ICPR, IABR, IPR[8];   // there is also a priority register but it has a different size
};
#define NVIC ((struct nvic *) 0xE000E100)

// create a structure for the Extended Interrupt and event controller (EXTI)
struct exti {
  volatile uint32_t RTSR1, FTSR1, SWIER1, RPR1, FPR1, RESERVED[19], 
      EXTICR[4], RESERVED2[4], IMR1, EMR1, RESERVED3[2];
};
#define EXTI ((struct exti *) 0x40021800)

// function to set a gpio pin to trigger an interrupt on a rising or falling edge event
void gpio_interrupt_init(uint16_t pin) {
  int x, m;                                         // variables for selecting EXTICR x and m
  
  RCC->APBENR2 |= BIT(0);                           // Enable the SYSCFG clock              
  
  x = PINNO(pin)/4;                                 // is index of EXTICRx register, (x-1)
  m = PINNO(pin)%4;                                 // multiplier to shift into EXTICRx
  EXTI->RTSR1 |= BIT(PINNO(pin));                   // select line to be a rising edge trigger
  EXTI->FTSR1 |= BIT(PINNO(pin));                   // select line to be a falling edge trigger too
  EXTI->EXTICR[x] |= (PINBANK(pin) << m*8);         // select pin port "pin" as interrupt source
  EXTI->IMR1 |= BIT(PINNO(pin));                    // unmask interrupt on selected line

  NVIC->ISER |= BIT(7);                             // enable EXTI line 4-15 interrupt
}


/*_____________________________________________________________________________________________

                        Hardware Delay (Doesnt depend on systick)

-----------------------------------------------------------------------------------------------*/

// define spin function for crude delay
static inline void spin(volatile uint32_t count) {
  while(count--) (void) 0;
}

//_____________________________________________________________________________________________//