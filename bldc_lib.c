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
static inline void __init__(void) {
    // define pins and mode for hall sensor interface 
    uint16_t hall_A = PIN('A', 1);
    gpio_set_mode(hall_A, GPI_MODE);
    uint16_t hall_B = PIN('A', 2);
    gpio_set_mode(hall_B, GPI_MODE);
    uint16_t hall_C = PIN('A', 3); 
    gpio_set_mode(hall_C, GPI_MODE);

    // initialize 3 channels on timer 3 to be used for PWM 
    __tim3_pwm_init__();

    // define pins for H-Bridge interface (set up pins with timers properly)
    uint16_t A_high = PIN('A', 6);        // timer 3 channel 1 
    gpio_set_mode(A_high, AF_MODE);
    gpio_set_af(A_high, AF1);
    uint16_t B_high = PIN('A', 7);        // timer 3 channel 2
    gpio_set_mode(B_high, AF_MODE);
    gpio_set_af(B_high, AF1);
    uint16_t C_high = PIN('B', 0);        // timer 3 channel 3
    gpio_set_mode(C_high, AF_MODE);
    gpio_set_af(C_high, AF1);

    uint16_t A_low = PIN('B', 3);
    gpio_set_mode(A_low, GPO_MODE);
    uint16_t B_low = PIN('B', 4);
    gpio_set_mode(B_low, GPO_MODE);
    uint16_t C_low = PIN('B', 5);
    gpio_set_mode(C_low, GPO_MODE);

    // enable IO port clocks 
    io_port_en('A', 1);
    io_port_en('B', 1);
}




/*_____________________________________________________________________________________________

                                "Lookup Table" for phase states wrt Hall effect sensors

-----------------------------------------------------------------------------------------------*/
// input a 3 bit number where the first 3 MSB's are the hall sensor levels and the LSB denotes the direction
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

