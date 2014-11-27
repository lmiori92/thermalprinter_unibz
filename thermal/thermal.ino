/**
 * 
 *  IlLorenz Software Engineering Division
 * 
 *      *** RPi Thermal Printer Driver ***
 * 
 *  November 2014
 * 
 */

/*
 * Various resources
 * 
 * http://www.lintech.org/comp-per/16THERM.pdf
 * 
 */

#include <PinChangeInt.h>

/*
 * These are the pins that are controlling a set of
 * 7 dots. Line select pins will define if they are
 * of the first, second or third batch...
 */

#define DOT_1_PIN       2
#define DOT_2_PIN       3
#define DOT_3_PIN       4
#define DOT_4_PIN       5
#define DOT_5_PIN       6
#define DOT_6_PIN       7
#define DOT_7_PIN       8

/*
 * Line select pins. They let to select the destination
 * driver(s) on which performing the actions.
 */
#define LINE_1_PIN      10
#define LINE_2_PIN      13
#define LINE_3_PIN      13

/**
 * Motor pins
 */
#define MOTOR_ENABLE_PIN    9

/**
 * Sensors: - x2 for sensing head position
 *          - x1 for motor sensing (aka crude stepper motor)
 *  The analog output seems to be active low, pullup resistor needed
 */
#define SENSE_HEAD_L    11
#define SENSE_HEAD_R    12
#define SENSE_MOTOR     13

#define MOTOR_STEP_PER_REVOLUTION   10
//#define MOTOR_STEP_HOME     50 // TODO explain better
#define MOTOR_STEP_HORIZ_LINE       2

/*
 * Timings (ms)
 */
#define PAPER_LOAD_TIME     1500
#define DOT_HEATER_TIME     10

/*
 * Constants
 */
#define DOT_1   (1 << 0)
#define DOT_2   (1 << 1)
#define DOT_3   (1 << 2)
#define DOT_4   (1 << 3)
#define DOT_5   (1 << 4)
#define DOT_6   (1 << 5)
#define DOT_7   (1 << 6)

#define PRINTHEAD_STATE_UNKNOWN     -1
#define PRINTHEAD_STATE_LEFT        0
#define PRINTHEAD_STATE_RIGHT       1

/*
 * 
 * Driver idea:
 * 
 * - low level -> pin management
 * - middle level -> logical operations
 * - high level -> dots to ouput (dots will be created via some printing interface?)s
 * 
 */

/* Custom interrupts */
void (* volatile motor_intr_custom)(void) = NULL;
void (* volatile sense_r_intr_custom)(void) = NULL;
void (* volatile sense_l_intr_custom)(void) = NULL;

/* Status variables */
volatile long motor_step_count = 0;
volatile int printhead_line_count = 0;
volatile int printhead_current_home = PRINTHEAD_STATE_UNKNOWN; // 0 Left, 1 Right

void shit_happens(char* msg) {
    printf(msg);
    // call the interrupt handler, it will do the job
    emergency();
//    sigint_handler(0);
}

/**
 * When in doubt, set all used pins to LOW
 */
void emergency(void) {
//    printf("reseting lines\n");
    //lines
    digitalWrite(DOT_1_PIN, LOW);
    digitalWrite(DOT_2_PIN, LOW);
    digitalWrite(DOT_3_PIN, LOW);
    digitalWrite(DOT_4_PIN, LOW);
    digitalWrite(DOT_5_PIN, LOW);
    digitalWrite(DOT_6_PIN, LOW);
    digitalWrite(DOT_7_PIN, LOW);
    //drivers
    digitalWrite(LINE_1_PIN, LOW);
    digitalWrite(LINE_2_PIN, LOW);
    digitalWrite(LINE_3_PIN, LOW);
    //motors
    digitalWrite(MOTOR_ENABLE_PIN, LOW);
}

void driver_toggle(int number, int state) {

    switch(number) {
        case 0:
            digitalWrite(LINE_1_PIN, state);
            break;
        case 1:
            digitalWrite(LINE_2_PIN, state);
            break;
        case 2:
            digitalWrite(LINE_3_PIN, state);
            break;
        default:
            shit_happens("Bummer, out of range!\n");
    }
}

void dot_toggle(int dot) {
    digitalWrite(dot, HIGH);
    delay(DOT_HEATER_TIME);
    digitalWrite(dot, LOW);
}

void printer_init(void) {
    //lines
    pinMode(DOT_1_PIN, OUTPUT);
    pinMode(DOT_2_PIN, OUTPUT);
    pinMode(DOT_3_PIN, OUTPUT);
    pinMode(DOT_4_PIN, OUTPUT);
    pinMode(DOT_5_PIN, OUTPUT);
    pinMode(DOT_6_PIN, OUTPUT);
    pinMode(DOT_7_PIN, OUTPUT);
    //driver
    pinMode(LINE_1_PIN, OUTPUT);
    pinMode(LINE_2_PIN, OUTPUT);
    pinMode(LINE_3_PIN, OUTPUT);
    //motor
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    //sensor
    pinMode(SENSE_HEAD_L, INPUT);
    pinMode(SENSE_HEAD_R, INPUT);
    pinMode(SENSE_MOTOR, INPUT);

    // reset all the lines
    emergency();
}

void paper_load() {
    printhead_home();
    printhead_home();
//    digitalWrite(MOTOR_ENABLE_PIN, HIGH);
//    delay(PAPER_LOAD_TIME);
//    digitalWrite(MOTOR_ENABLE_PIN, LOW);
}

// middle-level stuff

void dot_write(int driver, int value) {

    // prepare outputs
    digitalWrite(DOT_1_PIN, !!(value & DOT_1));
    digitalWrite(DOT_2_PIN, !!(value & DOT_2));
    digitalWrite(DOT_3_PIN, !!(value & DOT_3));
    digitalWrite(DOT_4_PIN, !!(value & DOT_4));
    digitalWrite(DOT_5_PIN, !!(value & DOT_5));
    digitalWrite(DOT_6_PIN, !!(value & DOT_6));
    digitalWrite(DOT_7_PIN, !!(value & DOT_7));

    // open the gate ! and then close it after a fraction of a second
//    driver_toggle(driver, HIGH);
    digitalWrite(LINE_1_PIN, HIGH);
//    printf("driver 1 state: %i\n", digitalRead(LINE_1_PIN));
    delay(100);
//    driver_toggle(driver, LOW);
    digitalWrite(LINE_1_PIN, LOW);
}

/*
 * Motor related routines
 */



void motor_on() {
    digitalWrite(MOTOR_ENABLE_PIN, HIGH);
//  analogWrite(MOTOR_ENABLE_PIN, 254);
//      OCR1B = 50;
//analogWrite(A5, 255);
}

void motor_off() {
    digitalWrite(MOTOR_ENABLE_PIN, LOW);
//analogWrite(A5, 0);
}

int motor_state() {
    return !!digitalRead(MOTOR_ENABLE_PIN);
}

long motor_get_revolutions() {
    return motor_step_count / MOTOR_STEP_PER_REVOLUTION;
}

long motor_get_steps() {
    return motor_step_count;
}

void motor_wait_stop() {
    while (motor_state()) {
        delay(1);
    }
}

/*
 * Routines to stop the motor after n steps
 */

long _motor_call_at_stop = -1;
void(*_motor_call_at_intr_func)() = NULL;

void _motor_call_at_intr() {
    if (motor_get_steps() >= _motor_call_at_stop) {
        _motor_call_at_intr_func();
        //printf("motor: call at function called. done.\n");
    } else {
        //printf("motor: call at function running.\n");
        motor_intr_custom = &_motor_call_at_intr;
    }
}

void motor_call_at(long steps, void(*callback)()) {
    printf("motor call at\n");
    _motor_call_at_stop = motor_get_steps() + steps;
    _motor_call_at_intr_func = callback;
    motor_intr_custom = &_motor_call_at_intr;
//    _motor_call_at_intr();
    motor_on();
    printf("motor call at end\n");
}

// long motor_stop_at = -1;
// 
// void _motor_count_intr() {
//         if (motor_step_count >= motor_stop_at) {
//             motor_off();
//             printf("needda stop\n");
//         } else {
//             printf("needda go on\n");
//             motor_on();
//             motor_intr_custom = &_motor_count_intr;
//         }
//     }

void _calibration_home_intr() {

//    motor_off();
    // reset all the interrupts since we accept both edges...
    sense_r_intr_custom = NULL;
    sense_l_intr_custom = NULL;
    printf("calibration: home position reached\n");
    motor_off();
//    motor_intr_custom = &_motor_count_intr;
//    motor_stop_at = motor_step_count + 50;
//    _motor_count_intr();

}

/*
 * Printhead routines
 */

void printhead_home() {
    // set the custom handler, triggered when the head reaches its home position...stop it!
    // NOTE: home position can be both edges, thus we accept one of the interrupt
    // important is to reset them in any case after one is triggered
    sense_r_intr_custom = &_calibration_home_intr;
    sense_l_intr_custom = &_calibration_home_intr;
    // turn the motor on
    motor_on();
    motor_wait_stop();
    motor_call_at(50, &motor_off);
    motor_wait_stop();
    // busy wait until the motor stops
//     while (1) {
//        if (!motor_state()) break;
//     }
    printf("calibration: home position adjusted. done.\n");
}

void printhead_move_line_horizontal() {
    // move the motor for the needed steps and then stop it
    motor_call_at(MOTOR_STEP_HORIZ_LINE, &motor_off);

    motor_wait_stop();
//     while (1) {
//         if (!motor_state()) break;
//     }
    printf("printhead: 1 horizontal line shifted. done.\n");
}

void printhead_move_line_verical() {
    // to move vertically by one line, it is sufficient to place
    // the printhead in the home position of either edge
    printhead_home();
}

void sensor_head_l_enter() {
    printhead_current_home = PRINTHEAD_STATE_LEFT;

    if (sense_l_intr_custom != NULL) {
        void(*old_intr)() = sense_l_intr_custom;
        sense_l_intr_custom = NULL;
        old_intr();
    }

    //printf("interrupt: SENSE_HEAD_L enter\n");
}

// void sensor_head_l_exit() {
//     printf("interrupt: SENSE_HEAD_L exit\n");
// }

void sensor_head_r_enter() {

    printhead_current_home = PRINTHEAD_STATE_RIGHT;

    if (sense_r_intr_custom != NULL) {
        void(*old_intr)() = sense_r_intr_custom;
        sense_r_intr_custom = NULL;
        old_intr();
    }
    //printf("interrupt: SENSE_HEAD_R enter\n");
}

// void sensor_head_r_exit() {
//     printf("interrupt: SENSE_HEAD_R exit\n");
// }

void sensor_motor_intr() {
    motor_step_count++;
    if (motor_intr_custom != NULL) {
        void(*old_intr)() = motor_intr_custom;
        motor_intr_custom = NULL;
        old_intr();
    }
    //printf("interrupt: SENSE_MOTOR enter: %li times\n", motor_step_count);
}

// #if PLATFORM == PLATFORM_RPI
// 
// int main(void)
// {
// 
//     wiringPiSetup();
// 
//     // try to set the maximum priority to reach a real-time-like env
//     piHiPri(99);
// 
//     init();
// 
//     // let's set up some interrupt functions, mainly for sensing
//     wiringPiISR(SENSE_HEAD_L, INT_EDGE_FALLING, &sensor_head_l_enter);
// //    wiringPiISR(SENSE_HEAD_L, INT_EDGE_RISING, &sensor_head_l_exit);
// 
//     wiringPiISR(SENSE_HEAD_R, INT_EDGE_FALLING, &sensor_head_r_enter);
// //    wiringPiISR(SENSE_HEAD_R, INT_EDGE_RISING, &sensor_head_r_exit);
// 
//     wiringPiISR(SENSE_MOTOR, INT_EDGE_FALLING, &sensor_motor_intr);
// //    wiringPiISR(SENSE_MOTOR, INT_EDGE_FALLING, &sensor_motor_enter);
// //    wiringPiISR(SENSE_MOTOR, INT_EDGE_RISING, &sensor_motor_exit);
// 
//     // set custom interrupt handler to reset lines if interrupted
//     // unknown states can be dangerous: printing head might burn out
// 
//     struct sigaction action;
//     memset(&action, 0, sizeof(action));
//     action.sa_handler = &sigint_handler;
//     sigaction(SIGINT, &action, &old_action);
// //    signal(SIGINT, sigint_handler);
// 
//     //use delay()?
//     printhead_home();
//     paper_load();
//     int old_current_pos = printhead_current_home;
//     while (1) {
//         printhead_move_line_horizontal();
//         if (old_current_pos != printhead_current_home) {
//             printhead_home();
//             old_current_pos = printhead_current_home;
//         }
// //        dot_write(LINE_1_PIN, 255);
//         driver_toggle(0, HIGH);
//     dot_toggle(DOT_1_PIN);
//     dot_toggle(DOT_2_PIN);
//     dot_toggle(DOT_3_PIN);
//     dot_toggle(DOT_4_PIN);
//     dot_toggle(DOT_5_PIN);
//     dot_toggle(DOT_6_PIN);
//     dot_toggle(DOT_7_PIN);
//         driver_toggle(0, LOW);
//         delay(1);
//     }
// //            printhead_home();
// //    int z = 0;
// //    for (z = 0; z < 10; z++) {
// 
// //        printhead_move_line_horizontal();
// //        delay(1000);
// //    }
//     
//     pause();
//     
//     driver_toggle(0, HIGH);
// 
//     paper_load();
// 
// //    digitalWrite(MOTOR_ENABLE_PIN, HIGH);
// //    pause();
// 
//     int i = 0;
//     while (1) {
//         emergency();
// 
//         digitalWrite(MOTOR_ENABLE_PIN, LOW);
// 
//         //driver_toggle(0, HIGH);
// 
// 
//         dot_write(0, 128);
// //     dot_toggle(DOT_1_PIN);
// //     dot_toggle(DOT_2_PIN);
// //     dot_toggle(DOT_3_PIN);
// //     dot_toggle(DOT_4_PIN);
// //     dot_toggle(DOT_5_PIN);
// //     dot_toggle(DOT_6_PIN);
// //     dot_toggle(DOT_7_PIN);
// 
// //    driver_toggle(0, LOW);
// 
// //         delay(10);
// //         digitalWrite(DOT_1_PIN, HIGH);
// //         delay(10);
// //         digitalWrite(DOT_2_PIN, HIGH);
// //         delay(10);
// //         digitalWrite(DOT_3_PIN, HIGH);
// //         delay(10);
// //         digitalWrite(DOT_4_PIN, HIGH);
// //         delay(10);
//         digitalWrite(MOTOR_ENABLE_PIN, HIGH);
//         delay(150);
//         if (i > 10) {
//             paper_load(); // una botta per il momento che non abbiamo i sensori...
//             i = 0;
//         }
//         i++;
//     }
// 
//     return 0;
// }
// 
// #elif PLATFORM == PLATFORM_ARDUINO

// TODO to be later placed in a separate file

/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>
 
 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */
 
/**
 * @file printf.h
 *
 * Setup necessary to direct stdout to the Arduino Serial library, which
 * enables 'printf'
 */

#ifndef __PRINTF_H__
#define __PRINTF_H__

#ifdef ARDUINO

int serial_putc( char c, FILE * ) 
{
  Serial.write( c );

  return c;
} 

void printf_begin(void)
{
  fdevopen( &serial_putc, 0 );
}

#else
#error This example is only for use on Arduino.
#endif // ARDUINO

#endif // __PRINTF_H__

void setup() {
  Serial.begin(9600);
    printf_begin();
    printf("Starting\n");
    printer_init();

//      (PIN, burpcount,RISING); // attach a PinChange Interrupt to our pin on the rising edge
// (RISING, FALLING and CHANGE all work with this library)
// and execute the function burpcount when that pin changes
    PCintPort::attachInterrupt(SENSE_HEAD_L, sensor_head_l_enter, FALLING);
    PCintPort::attachInterrupt(SENSE_HEAD_R, sensor_head_r_enter, FALLING);
    PCintPort::attachInterrupt(SENSE_MOTOR, sensor_motor_intr, FALLING);
//    pinMode(A5, OUTPUT);
//    TCCR1B = TCCR1B & 0b11111000 | 0x03;

}

void loop() {
    printhead_home();
//motor_call_at(1, &motor_off);

    printf("Intr count %li\n", motor_step_count);
    printf("Current head position %i\n", printhead_current_home);
    delay(1000);
//    printhead_home();
}
