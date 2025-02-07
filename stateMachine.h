#ifndef APPLICATION_STATEMACHINE_H_
#define APPLICATION_STATEMACHINE_H_
#include <stdint.h>
//#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include "F28x_Project.h"
#include "Utils.h"

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

typedef struct
{
    States currentState;
    SmCommands command;
} StateMachine;

void SM_init(StateMachine *);
int SM_changeState(StateMachine *, const SmCommands);
#endif /* APPLICATION_STATEMACHINE_H_ */
