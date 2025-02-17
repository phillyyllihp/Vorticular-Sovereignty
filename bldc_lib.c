/*
FUNCTIONS LIBRARY FOR BLDC MOTOR PROJECT

Author: Luke Phillips 
*/

#include <inttypes.h>
#include <stdbool.h>
#include "hal.c"


/*_____________________________________________________________________________________________

                                Initialization

-----------------------------------------------------------------------------------------------*/

// Global Variables to define motor characteristics
uint16_t R_armature = 1.5;                  // armature winding resistance in ohms
uint16_t motor_poles = 8;                   // number of poles


// create global variables for gpio
// variables for hall sensor inputs
uint16_t hall_A = PIN('A', 6);              // TIM3_CH1 on PIN 13  
uint16_t hall_B = PIN('A', 7);              // TIM3_CH2 on PIN 14
uint16_t hall_C = PIN('B', 0);              // TIM3_CH3 on PIN 15

// pins for interfacing with IFX007 IN pin (1 activates HIGH side, 0 activates LOW side when INH is enabled)
uint16_t A_IN = PIN('A', 8);                // TIM1_CH1 on PIN 18
uint16_t B_IN = PIN('B', 3);                // TIM1_CH2 on PIN 27
uint16_t C_IN = PIN('B', 6);                // TIM1_CH3 on PIN 30

// pins for interfacing with IFX007 INH pin (1 activates outputs, 0 disables device) 
uint16_t A_EN = PIN('B', 2);                // GPIO on PIN 17
uint16_t B_EN = PIN('A', 15);               // GPIO on PIN 26
uint16_t C_EN = PIN('B', 5);                // GPIO on PIN 29

// pins for interfacing with the IS pin of IFX007 (5V tolerant ADC pins)
uint16_t A_IS = PIN('B', 1);                // ADC_IN9 on PIN 16
uint16_t B_IS = PIN('A', 1);                // ADC_IN1 on PIN 8
uint16_t C_IS = PIN('A', 0);                // ADC_IN0 on PIN 7

//____________________________________________________________________________________________________//

// function for initialization of gpio and other peripherals 
static inline void __init__(void) {

    // initialize systick 
    SysTick_init(16000000 / 1000);          // intialize the SysTick to count every 1ms | 1khz

    // initialize 3 channels on timer 1 to be used for PWM 
    __tim1_pwm_init__();

    // initialize the adc
    __adc_init__();

    // enable IO port clocks 
    io_port_en(('A'-'A'), 1);
    io_port_en(('B'-'A'), 1);

    // Hall sensor encoder set up on timer 3
    gpio_set_mode(hall_A, AF_MODE);
    gpio_set_af(hall_A, AF1);
    //gpio_interrupt_init(hall_A);
    gpio_set_mode(hall_B, AF_MODE);
    gpio_set_af(hall_B, AF1);
    //gpio_interrupt_init(hall_B); 
    gpio_set_mode(hall_C, AF_MODE);
    gpio_set_af(hall_C, AF1);
    //gpio_interrupt_init(hall_C);

    // Set IN pins to be on timer1 to utilize PWM 
    // see about synchronizing with timer3 hall encoder?
    gpio_set_mode(A_IN, AF_MODE);
    gpio_set_af(A_IN, AF2);
    gpio_set_mode(B_IN, AF_MODE);
    gpio_set_af(B_IN, AF1);
    gpio_set_mode(C_IN, AF_MODE);
    gpio_set_af(C_IN, AF1);

    // GPIO to enable and disable IFX007 IC's for each phase
    gpio_set_mode(A_EN, GPO_MODE);
    gpio_set_mode(B_EN, GPO_MODE);
    gpio_set_mode(C_EN, GPO_MODE);

    // set current sense pins to adc mode
    gpio_set_mode(A_IS, ANALOG_MODE);
    gpio_set_mode(B_IS, ANALOG_MODE);
    gpio_set_mode(C_IS, ANALOG_MODE);
}


/*_____________________________________________________________________________________________

                                Functions for driving the motors

-----------------------------------------------------------------------------------------------*/

// select which IFX007 s to enable
static inline void hbridge_en(bool a, bool b, bool c) {
    gpio_write(A_EN, a);
    gpio_write(B_EN, b);
    gpio_write(C_EN, c);
}

// function to set phase and duty 
static inline void set_phases(bool a_en, bool a_in, uint16_t duty_a, bool b_en, bool b_in, uint16_t duty_b, bool c_en, bool c_in, uint16_t duty_c) {
    hbridge_en(a_en, b_en, c_en);
    tim1_pwm_duty(duty_a, duty_b, duty_c);
    tim1_ch_en(a_in, b_in, c_in);
}

// homes the motor to winding A given the desired direction of commutation to follow
static inline void find_home(bool forwards) {
    if (forwards) {
        set_phases(1, 1, 0x7FFF, 1, 0, 0x0000, 0, 0, 0x0000);
    }
    else {
        set_phases(1, 1, 0x7FFF, 0, 0, 0x0000, 1, 0, 0x0000);
    }
    delay(5);                                                           // delay for 5ms to orient motor
}


/*_____________________________________________________________________________________________

                                Functions for Hall Effect Sensor 

-----------------------------------------------------------------------------------------------*/

// function to get states of hall effect sensors
static inline void get_hall_states(bool* a, bool* b, bool* c) {
    *a = gpio_read(hall_A);
    *b = gpio_read(hall_B);
    *c = gpio_read(hall_C);
}



/*_____________________________________________________________________________________________


                        Interrupt Service Routines (ISR)

-----------------------------------------------------------------------------------------------*/

// Global Variables for tracking hall effect sensor data
uint16_t steps_a, steps_b, steps_c;                 // global variables for tracking steps on each hall sensor line
bool dir_a, dir_b, dir_c;                           // create a variable for direction from data from the hall sensor


// function to handle interrupt on EXTI lines 4 to 15
void EXTI4_15_Handler(void) {

    bool a, b, c;                                   // create a variable for each of the hall sensor line states            
    get_hall_states(&a, &b, &c);                    // store the value of each hall state in its corresponding variable

    // if hall sensor A sets rising edge
    if(EXTI->RPR1 & BIT(PINNO(hall_A))) {

        // get the direction -- 1 = forwards , 0 = reverse
        if (a==1 & b==0 & c==1) {
            dir_a = 1;                               // forwards
            steps_a ++;                              // increase steps counted on line a 
        }       
        else if (a==1 & b==1 & c==0) {
            dir_a = 0;                               // reverse
            steps_a--;                               // decrease steps seen on line a
        }   
        
        EXTI->RPR1 |= BIT(PINNO(hall_A));           // clear pending interupt bit
    }

    // if hall sensor A sets a falling edge 
    /*
    if (EXTI->FPR1 & BIT(PINNO(hall_A)) {

        
    }
    */
        
    NVIC->ICPR |= BIT(7);                             //clear EXTI 4-15 interupt flag

}



/*_____________________________________________________________________________________________

                                "Lookup Table" for phase states wrt Hall effect sensors

-----------------------------------------------------------------------------------------------*/
// may need some tending to depending on what input structure should look like
static inline int from_table(uint16_t pos, bool dir, int* a, int* b, int* c) {

    int A, B, C, temp;

    switch (pos) {

        // not in motion
        case 000:
            A = 0;
            B = 0;
            C = 0;
            break;

        // 0 - 60
        case 101:
            A =  1;
            B = -1;
            C =  0;
            break;

        // 60 - 120
        case 100:
            A =  1;
            B =  0;
            C = -1;
            break;

        // 120 - 180
        case 110:
            A =  0;
            B =  1;
            C = -1;
            break;

        // 180 - 240
        case 010:
            A = -1;
            B =  1;
            C =  0;
            break;

        // 240 - 300
        case 011:
            A = -1;
            B =  0;
            C =  1;
            break;

        // 300 - 360
        case 001:
            A =  0;
            B = -1;
            C =  1;
            break;
    }

    if (dir){
        temp = A;
        A = C;
        C = temp;
    }

    *a = A;
    *b = B;
    *c = C;

}

//_____________________________________________________________________________________________//