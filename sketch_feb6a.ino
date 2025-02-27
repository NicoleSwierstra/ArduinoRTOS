#include <Servo.h>
/* Pins 4 and 5 for some reason??????? */
#define LEFT_SERVO        3
#define RIGHT_SERVO       4

#define INPUT_PORT_Pos    0

#define LEFT_IR           (1 << 0)
#define CENTER_IR         (1 << 1)
#define RIGHT_IR          (1 << 2)

#define BUFFER_LEN 128
#define CALIBRATION_MODE 0

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
        left += (buffer[i] & LEFT_IR);
        right += (buffer[i] & LEFT_IR);
        center += (buffer[i] & LEFT_IR);
    }
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
    DDRD = B11111000; /* set first 3 pins as input pins */

    leftServo.attach(LEFT_SERVO);
    leftServo.write(90); /* Idle left motor */
    rightServo.attach(RIGHT_SERVO);
    rightServo.write(90); /* Idle right motor */

#if CALIBRATION_MODE == 1
  while(1)
#endif
}

void loop() {
    RTOS_ExecuteTasks();
}
