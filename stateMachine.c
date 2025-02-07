#include "stateMachine.h"

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
    self->currentState = SM_PLL;
}

int SM_changeState(StateMachine *self, const SmCommands cmd)
{
    int success = 0;
        self->command = cmd;
        switch (self->currentState)
        {
        case SM_PLL:
            if (SM_GOTO_SHUNT == cmd)
            {
                self->currentState = SM_SHUNT;
                success = 1;
            }
            break;
        case SM_SHUNT:
            if (SM_GOTO_BYPASS == cmd)
            {
                self->currentState = SM_BYPASS;
                success = 1;
            }
            break;
        case SM_BYPASS:
            if (SM_GOTO_DCBUS == cmd)
            {
                self->currentState = SM_DCBUS;
                success = 1;
            }
            break;
        case SM_DCBUS:
            if (SM_GOTO_WORKING == cmd)
            {
                self->currentState = SM_WORKING;
                success = 1;
            }
            break;
        }
        if (SM_GOTO_ERROR == cmd)
        {
            self->currentState = SM_ERROR;
            success = 1;
        }
    return success;
}
