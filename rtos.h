/*
 * Nicole Swierstra
 * My custom RTOS, written for a tiny footprint
 */

#include <stdint.h>
#ifndef _RTOS_
#define _RTOS_

#if defined(__cplusplus)
extern "C" {
#endif
    
#define RTOS_maxTaskNum  32
#define RTOS_maxStateNum 16

typedef struct {
    void (*callback)();
    uint16_t reset_ms;
    uint16_t counter;
} rtos_task;

typedef struct {
    void (*entry)();
    void (*exit)();
    uint32_t taskMask;
} rtos_state;

typedef struct {
    volatile uint32_t taskQue; 
    volatile uint8_t numberOfTasks;
    volatile rtos_task tasks[RTOS_maxTaskNum];

    volatile uint8_t updateLock;
    volatile uint8_t numberOfStates;
    volatile uint8_t state;
    volatile rtos_state states[RTOS_maxStateNum];
} kernel;

extern kernel rtos_scheduler;

/**
*  Inits and zeros the RTOS, just in case
*/
int RTOS_init(void);

/**
*  Adds a state to the car, and then returns the index of the state
*/
int RTOS_addState(void (*start), void (*stop));

/**
* switches the cars state to the state in the pool at index of @state
*/
int RTOS_switchState(uint8_t state);

/**
* Checks if you are in that state
*/
int RTOS_inState(uint8_t state);

/**
* schedules a task to be performed in the referenced state
*/
int RTOS_scheduleTask(uint8_t state, void (*function)(), uint16_t period);

/**
* In interrupt, calls events and queues tasks
*/
int RTOS_Update(void);

/**
* In non-rt context, calls tasks that have been queued
*/
int RTOS_ExecuteTasks(void);
#if defined(__cplusplus)
}
#endif
#endif