#ifndef APPLICATION_STATEMACHINE_H_
#define APPLICATION_STATEMACHINE_H_
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include "F28x_Project.h"

typedef enum
{
    SM_PLL = 0,
    SM_SHUNT,
    SM_BYPASS,
    SM_DCBUS,
    SM_WORKING,
    SM_ERROR
} States;

typedef enum
{
    SM_NO_CMD = 0U,
    SM_GOTO_SHUNT = 1U,
    SM_GOTO_BYPASS,
    SM_GOTO_DCBUS,
    SM_GOTO_WORKING,
    SM_GOTO_ERROR
} SmCommands;

typedef void (*TransitionFunc)(void *funcRef);

typedef struct
{
    States currentState;
    States nextState;
    SmCommands command;
    TransitionFunc transPLL2SHUNT;
    TransitionFunc transSHUNT2BYPASS;
    TransitionFunc transBYPASS2DCBUS;
    TransitionFunc transDCBUS2WORKING;
    TransitionFunc transAnyToError;
    void *transRef;
} StateMachine;

void SM_init(StateMachine *);
void SM_main(StateMachine *);
void SM_processCommand(StateMachine *, const SmCommands);
void SM_autoStateChange(StateMachine *);
static void SM_clearError(StateMachine *);
#endif /* APPLICATION_STATEMACHINE_H_ */