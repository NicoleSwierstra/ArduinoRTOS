#include <Servo.h>
#include "rtos.h"

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
        int16_t multiplier = (state.bufferptr - i) + BUFFER_LEN;

        la_left += (state.buffer[i] & LEFT_IR) * multiplier;
        la_right += (state.buffer[i] & RIGHT_IR) * multiplier;
        la_center += (state.buffer[i] & CENTER_IR) * multiplier;

        multiplier = (i - state.bufferptr);
        lb_left += (state.buffer[i] & LEFT_IR) * multiplier;
        lb_right += (state.buffer[i] & RIGHT_IR) * multiplier;
        lb_center += (state.buffer[i] & CENTER_IR) * multiplier;
    }

    state.lookahead[0] = la_left >> BUFFER_POW;
    state.lookahead[1] = la_right >> BUFFER_POW;
    state.lookahead[2] = la_center >> BUFFER_POW;

    state.lookbehind[0] = lb_left >> BUFFER_POW;
    state.lookbehind[1] = lb_right >> BUFFER_POW;
    state.lookbehind[2] = lb_center >> BUFFER_POW;
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

    if(state.lookahead[LEFT_Pos] < 30 && state.lookahead[RIGHT_Pos] < 30 && 0){
        leftServo.write(180);
        rightServo.write(0);
    }
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

    /* reads data every ms */
    taskHandle = RTOS_scheduleTask(stateHandle, fastRead, 1);
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
        //Serial.print("todo: ");
        //Serial.println(((uint32_t)rtos_scheduler.taskQue));
        RTOS_ExecuteTasks();
    }
}

/* This is a goofy macro lol */
ISR(TIMER0_COMPA_vect){
    RTOS_Update();
    rtos_scheduler.taskQue = 0b11;
}

void loop(){}
