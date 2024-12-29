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

// create global variables for gpio

// variables for hall sensor inputs
// change these to be on same gpio as timer 3 channels 
uint16_t hall_A = PIN('A', 1);
uint16_t hall_B = PIN('A', 2);
uint16_t hall_C = PIN('A', 3);

// pins for interfacing with H-bridge high side
// change these to be on timer1 channels
uint16_t A_high = PIN('A', 6);        // timer 3 channel 1 
uint16_t B_high = PIN('A', 7);        // timer 3 channel 2
uint16_t C_high = PIN('B', 0);        // timer 3 channel 3

// pins for interfacing with H-bridge low side
// set these to the same gpio as timer 1 complementary channels 
uint16_t A_low = PIN('B', 3);
uint16_t B_low = PIN('B', 4);
uint16_t C_low = PIN('B', 5);


static inline void __init__(void) {
    // enable IO port clocks 
    io_port_en(('A'-'A'), 1);
    io_port_en(('B'-'A'), 1);

    // define pin mode for hall sensor interface 
    // change these to be on same gpio as timer 3 channels 
    gpio_set_mode(hall_A, GPI_MODE);
    gpio_set_mode(hall_B, GPI_MODE); 
    gpio_set_mode(hall_C, GPI_MODE);

    // initialize 3 channels on timer 3 to be used for PWM 
    __tim3_pwm_init__();

    // initialize systick 
    SysTick_init(16000000 / 1000);          // intialize the SysTick to count every 1ms | 1khz

    // define pin mode for H-Bridge interface (set up pins with timers properly)
    // change these to be timer 1
    
    gpio_set_mode(A_high, AF_MODE);
    gpio_set_af(A_high, AF1);
    gpio_set_mode(B_high, AF_MODE);
    gpio_set_af(B_high, AF1);
    gpio_set_mode(C_high, AF_MODE);
    gpio_set_af(C_high, AF1);

    // set these to the same gpio as timer 1 complementary channels 
    gpio_set_mode(A_low, GPO_MODE);
    gpio_set_mode(B_low, GPO_MODE);
    gpio_set_mode(C_low, GPO_MODE);
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

