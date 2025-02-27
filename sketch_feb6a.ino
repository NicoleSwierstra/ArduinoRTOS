#include <Servo.h>
#include "rtos.h"

#define LEFT_SERVO        3
#define RIGHT_SERVO       4

#define INPUT_PORT_Pos    0

#define LEFT_IR           (1 << 0)
#define CENTER_IR         (1 << 1)
#define RIGHT_IR          (1 << 2)

#define BUFFER_POW 5
#define CALIBRATION_MODE 0
const int BUFFER_LEN = (2 << BUFFER_POW); /* Int so it's not recalculated */

Servo leftServo;
Servo rightServo;

kernel rtos_scheduler = {0};

void task();

struct _state {
    uint8_t buffer[BUFFER_LEN];
    uint16_t bufferptr;

    uint8_t lookahead[3];
    uint8_t lookbehind[3];
} state;


uint32_t collapse() {
    uint8_t la_left = 0,
            la_right = 0, 
            la_center = 0;

    uint8_t lb_left = 0,
            lb_center = 0,
            lb_right = 0;

    for (int i = 0; i < BUFFER_LEN; i++){
        la_left += (state.buffer[i] & LEFT_IR);
        la_right += (state.buffer[i] & LEFT_IR);
        la_center += (state.buffer[i] & LEFT_IR);

        lb_left += (state.buffer[i] & LEFT_IR);
        lb_right += (state.buffer[i] & LEFT_IR);
        lb_center += (state.buffer[i] & LEFT_IR);
    }
}

void fastRead(){
    state.bufferptr++;
    state.buffer[state.bufferptr] = PIND >> INPUT_PORT_Pos;
}

void task(){
  switch(PIND >> INPUT_PORT_Pos) {
    case LEFT_IR | CENTER_IR | RIGHT_IR:
    case LEFT_IR | RIGHT_IR:
      leftServo.write(90);
      rightServo.write(90); 
      break;
    case LEFT_IR | CENTER_IR:
    case LEFT_IR:
      leftServo.write(90);
      rightServo.write(0); 
      break;
    case CENTER_IR:
    case 0:
      leftServo.write(180);
      rightServo.write(0);
      break;
    case RIGHT_IR | CENTER_IR:
    case RIGHT_IR:
      leftServo.write(180);
      rightServo.write(90);
      break;
  }
}

void setup() {
    RTOS_init();

    DDRD = 0xFF & ~(0b111 << INPUT_PORT_Pos);

    leftServo.attach(LEFT_SERVO);
    leftServo.write(90); /* Idle left motor */
    rightServo.attach(RIGHT_SERVO);
    rightServo.write(90); /* Idle right motor */

    /* TODO: CLOCK SETUP FOR 20MHz SYSTEM CLOCK */

#if CALIBRATION_MODE == 1
    while(1);
#endif

    cli();
    TCCR0A = 0;
    TCCR0B = 0;
    TCNT0  = 0; /* Set counter to be 0 */
    OCR0A = 250; /* What value to time - 250 = interrupt every 250 timer cycles, 1 timer cycle = 64 clock cycles */
    TCCR0A |= 0b10; /* Clear interrupt on compare match */
    TCCR0B |= 0b11; /* Set prescaler at 64x */
    TIMSK0 |= (1 << OCIE0A); /* Start timer cmp interrupt */
    sei(); /* Enable interrupts */
}

/* This is a goofy macro lol */
ISR(TIMER0_COMPA_vect){
    RTOS_Update();
}


void loop() {
    RTOS_ExecuteTasks();
}
