/**
 ******************************************************************************
 * @file    State_Machine.h
 * @brief   Formal integrated application state machine
 ******************************************************************************
 */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

typedef enum
{
    STATE_INIT = 0,
    STATE_IDLE,
    STATE_UART_PARSE,
    STATE_ERROR
} System_State_t;

void State_Machine_Init(void);
void State_Machine_Update(void);
System_State_t State_Machine_GetCurrentState(void);

#endif
