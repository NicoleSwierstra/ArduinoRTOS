#include <Servo.h>
#include "rtos.h"
#include "lookup.c"

/*
 * Speeds:
 * +- 90  = 26cm/s
 * +- 70  = 26cm/s
 * +- 60  = 26cm/s
 * +- 45  = 25cm/s
 * +- 30  = 20cm/s
 * +- 15  = 11cm/s
 * 
 * Speeds (9v):
 * +- 90  = 60cm/s
 * +- 70  = 26cm/s
 * +- 60  = 26cm/s
 * +- 45  = 25cm/s
 * +- 30  = 20cm/s
 * +- 15  = 11cm/s
 */

#define LEFT_SERVO        3
#define RIGHT_SERVO       4

#define INPUT_PORT_Pos    5

#define LEFT_Pos          0
#define CENTER_Pos        1
#define RIGHT_Pos         2

#define LEFT_IR           (1 << LEFT_Pos)
#define CENTER_IR         (1 << CENTER_Pos)
#define RIGHT_IR          (1 << RIGHT_Pos)

#define BUFFER_POW 5
const int BUFFER_LEN = (2 << BUFFER_POW); /* Int so it's not recalculated */
#define CALIBRATION_MODE 0

Servo leftServo;
Servo rightServo;

kernel rtos_scheduler;

void task();

struct _state {
    uint8_t buffer[BUFFER_LEN * 2]; /* doubling up the buffer and copying the data means that we can do a continous read w/o a mod function */
    uint16_t bufferptr;

    uint8_t lookahead[3];
    uint8_t lookbehind[3];
} state;


void collapse() {
    uint16_t la_left = 0,
             la_right = 0, 
             la_center = 0;

    uint16_t lb_left = 0,
             lb_center = 0,
             lb_right = 0;

    for (int i = state.bufferptr; i < state.bufferptr + BUFFER_LEN; i++){
        uint16_t multiplier = ((state.bufferptr + BUFFER_LEN) - i);

        la_left += ((state.buffer[i] >> LEFT_Pos) & 0x1) * multiplier;
        la_right += ((state.buffer[i] >> RIGHT_Pos) & 0x1) * multiplier;
        la_center += ((state.buffer[i] >> CENTER_Pos) & 0x1) * multiplier;

        multiplier = (i - state.bufferptr);
        lb_left += ((state.buffer[i] >> LEFT_Pos) & 0x1) * multiplier;
        lb_right += ((state.buffer[i] >> RIGHT_Pos) & 0x1) * multiplier;
        lb_center += ((state.buffer[i] >> CENTER_Pos) & 0x1) * multiplier;
    }

    state.lookahead[0] = la_left >> BUFFER_POW;
    state.lookahead[1] = la_center >> BUFFER_POW;
    state.lookahead[2] = la_right >> BUFFER_POW;

    state.lookbehind[0] = lb_left >> BUFFER_POW;
    state.lookbehind[1] = lb_center >> BUFFER_POW;
    state.lookbehind[2] = lb_right >> BUFFER_POW;
}

void fastRead(){
    state.bufferptr++;
    state.bufferptr = state.bufferptr % BUFFER_LEN;

    uint8_t read = PIND >> INPUT_PORT_Pos;
    state.buffer[state.bufferptr] = read;
    state.buffer[state.bufferptr + BUFFER_LEN] = read;
}

void task(){
    collapse();

    uint8_t p0 = (state.lookahead[0]  *   4) / BUFFER_LEN;
    uint8_t p1 = (state.lookahead[1]  *   4) / BUFFER_LEN;
    uint8_t p2 = (state.lookahead[2]  *   4) / BUFFER_LEN;
    uint8_t p3 = (state.lookbehind[0] *   4) / BUFFER_LEN;
    uint8_t p4 = (state.lookbehind[1] *   4) / BUFFER_LEN;
    uint8_t p5 = (state.lookbehind[2] *   4) / BUFFER_LEN;

    uint8_t cmd_l = lookuptable[p0][p1][p2][p3][p4][p5 * 2] + 90;
    uint8_t cmd_r = -lookuptable[p0][p1][p2][p3][p4][p5 * 2 + 1] + 90;

    leftServo.write(cmd_l);
    rightServo.write(cmd_r);
}

void setup() {
    Serial.begin(9600);
    delay(1000);
    Serial.println("Hello World!");

    DDRD = 0xFF & ~(0b111 << INPUT_PORT_Pos);

    leftServo.attach(LEFT_SERVO);
    leftServo.write(90); /* Idle left motor */
    rightServo.attach(RIGHT_SERVO);
    rightServo.write(90); /* Idle right motor */

    /* TODO: CLOCK SETUP FOR 20MHz SYSTEM CLOCK */

#if CALIBRATION_MODE == 1
    while(1);
#endif

    RTOS_init();
    uint32_t stateHandle = RTOS_addState(NULL, NULL);
    uint32_t taskHandle; 

    /* runs motors every 20ms*/
    taskHandle = RTOS_scheduleTask(stateHandle, task, 20);
    Serial.println(taskHandle);
    if(taskHandle == -1) return;

    /* reads data every ms, approximately every mm */
    taskHandle = RTOS_scheduleTask(stateHandle, fastRead, 2);
    Serial.println(taskHandle);
    if(taskHandle == -1) return;

    cli(); /* disable interrupts */
    /* Sets a timer to update the rtos every ms */
    TCCR0A = 0;
    TCCR0B = 0;
    TCNT0  = 0; /* Set counter to be 0 */
    OCR0A = 250; /* What value to time - 250 = interrupt every 250 timer cycles, 1 timer cycle = 64 clock cycles */
    TCCR0A |= 0b10; /* Clear interrupt on compare match */
    TCCR0B |= 0b11; /* Set prescaler at 64x */
    TIMSK0 |= (1 << OCIE0A); /* Start timer cmp interrupt */
    sei(); /* Enable interrupts */
    
    while (1){
        RTOS_ExecuteTasks();
    }
}

/* This is a goofy macro lol */
ISR(TIMER0_COMPA_vect){
    RTOS_Update();
}

void loop(){}
