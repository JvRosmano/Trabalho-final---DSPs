#include "stateMachine.h"

// local function prototype
void SM_emptyFunc(void *);
/*****************************************************************************/
/**
* This function initializes the app instance.
*
* @param   self is the
StateMachine instance we are working on.
*
* @return  None.
*
* @note    The transition functions must be assigned externally, e.g.
* sm.transInitToOff = ((TransitionFunc)((void*)APP_transInitToOff));
*
*****************************************************************************/
void SM_init(StateMachine *self)
{
    // check parameters
    assert(NULL != self);
    self->currentState = SM_PLL;
    self->nextState = SM_PLL;
    self->transPLL2SHUNT = ((TransitionFunc)((void *)SM_emptyFunc));
    self->transSHUNT2BYPASS = ((TransitionFunc)((void *)SM_emptyFunc));
    self->transBYPASS2DCBUS = ((TransitionFunc)((void *)SM_emptyFunc));
    self->transDCBUS2WORKING = ((TransitionFunc)((void *)SM_emptyFunc));
    self->transAnyToError = ((TransitionFunc)((void *)SM_emptyFunc));
}

void SM_main(StateMachine *self)
{
    // check parameters
    assert(NULL != self);
    assert(NULL != self->transRef);
    /* Nothing to do, leave function*/
    if (self->currentState == self->nextState)
        return;
    /* Change state if allowed*/
    switch (self->currentState)
    {
    case SM_PLL:
        if (SM_SHUNT == self->nextState)
        {
            DINT;
            /* Atomic call of transisiton*/
            self->transPLL2SHUNT(self->transRef);
            self->currentState = self->nextState;
            EINT;
        }
        else if (SM_ERROR == self->nextState)
        {
            DINT;
            /* Atomic call of transisiton*/
            self->transAnyToError(self->transRef);
            self->currentState = self->nextState;
            EINT;
        }
        break;
    case SM_SHUNT:
        if (SM_BYPASS == self->nextState)
        {
            DINT;
            /* Atomic call of transisiton*/
            self->transSHUNT2BYPASS(self->transRef);
            self->currentState = self->nextState;
            EINT;
        }
        else if (SM_ERROR == self->nextState)
        {
            DINT;
            /* Atomic call of transisiton*/
            self->transAnyToError(self->transRef);
            self->currentState = self->nextState;
            EINT;
        }
        break;

    case SM_BYPASS:
        if (SM_DCBUS == self->nextState)
        {
            DINT;
            /* Atomic call of transisiton*/
            self->transBYPASS2DCBUS(self->transRef);
            self->currentState = self->nextState;
            EINT;
        }
        else if (SM_ERROR == self->nextState)
        {
            DINT;
            /* Atomic call of transisiton*/
            self->transAnyToError(self->transRef);
            self->currentState = self->nextState;
            EINT;
        }
        break;

    case SM_DCBUS:
        if (SM_WORKING == self->nextState)
        {
            DINT;
            /* Atomic call of transisiton*/
            self->transDCBUS2WORKING(self->transRef);
            self->currentState = self->nextState;
            EINT;
        }
        else if (SM_ERROR == self->nextState)
        {
            DINT;
            /* Atomic call of transisiton*/
            self->transAnyToError(self->transRef);
            self->currentState = self->nextState;
            EINT;
        }
        break;
    }
}

void SM_processCommand(StateMachine *self, const SmCommands cmd)
{
    static SmCommands cmdOld = SM_NO_CMD;
    // check parameters
    assert(NULL != self);
    if (cmdOld == cmd)
    {
        return;
    }
    self->command = cmd;
    switch (self->currentState)
    {
    case SM_PLL:
        if (SM_GOTO_SHUNT == cmd)
        {
            self->nextState = SM_SHUNT;
        }
        break;
    case SM_SHUNT:
        if (SM_GOTO_BYPASS == cmd)
        {
            self->nextState = SM_BYPASS;
        }
        break;
    case SM_BYPASS:
        if (SM_GOTO_DCBUS == cmd)
        {
            self->nextState = SM_DCBUS;
        }
        break;
    case SM_DCBUS:
        if (SM_GOTO_WORKING == cmd)
        {
            self->nextState = SM_WORKING;
        }
        break;
    }
}

void SM_emptyFunc(void *params)
{
}
