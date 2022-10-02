/*
*********************************************************************************************************
*                                                uC/OS-II
*                                          The Real-Time Kernel
*                                     ATMEL  AVR32 UC3 Specific code
*
*                                 (c) Copyright 2007; Micrium; Weston, FL
*                                           All Rights Reserved
*
* File    : OS_CPU_C.C
* By      : Fabiano Kovalski
*
* LICENSING TERMS:
* ---------------
*   uC/OS-II is provided in source form for FREE evaluation, for educational use or for peaceful research.
* If you plan on using  uC/OS-II  in a commercial product you need to contact Micrium to properly  license
* its use in your product.  We provide ALL the source code for your convenience and to help you experience
* uC/OS-II.   The fact that the  source  is provided does  NOT  mean that you can use it without  paying a
* licensing fee.
*********************************************************************************************************
*/

#define   OS_CPU_GLOBALS
#include  <ucos_ii.h>


/*
*********************************************************************************************************
*                                          LOCAL VARIABLES
*********************************************************************************************************
*/

#if      (OS_VERSION >= 281) && (OS_TMR_EN > 0)
  static  INT16U  OSTmrCtr;
#endif                                                                  /* #if (OS_VERSION >= 281) && (OS_TMR_EN > 0)               */

/*
**************************************************************************************************************
*                                         STATUS REGISTER MASKS
**************************************************************************************************************
*/

#define  OS_CPU_SR_M0_MASK              0x00400000                      /* Status Register, Supervisor Execution Mode Mask          */

/*
*********************************************************************************************************
*                                       OS INITIALIZATION HOOK
*                                            (BEGINNING)
*
* Description: This function is called by OSInit() at the beginning of OSInit().
*
* Arguments  : none
*
* Note(s)    : 1) Interrupts should be disabled during this call.
*********************************************************************************************************
*/
#if (OS_CPU_HOOKS_EN > 0) && (OS_VERSION > 203)
void  OSInitHookBegin (void)
{
#if (OS_VERSION >= 281) && (OS_TMR_EN > 0)
    OSTmrCtr = 0;
#endif
}
#endif

/*
*********************************************************************************************************
*                                       OS INITIALIZATION HOOK
*                                               (END)
*
* Description: This function is called by OSInit() at the end of OSInit().
*
* Arguments  : none
*
* Note(s)    : 1) Interrupts should be disabled during this call.
*********************************************************************************************************
*/
#if (OS_CPU_HOOKS_EN > 0) && (OS_VERSION > 203)
void  OSInitHookEnd (void)
{
}
#endif

/*$PAGE*/
/*
*********************************************************************************************************
*                                          TASK CREATION HOOK
*
* Description: This function is called when a task is created.
*
* Arguments  : ptcb   is a pointer to the task control block of the task being created.
*
* Note(s)    : 1) Interrupts are disabled during this call.
*********************************************************************************************************
*/
#if OS_CPU_HOOKS_EN > 0
void  OSTaskCreateHook (OS_TCB *ptcb)
{
#if OS_APP_HOOKS_EN > 0
    App_TaskCreateHook(ptcb);
#else
    (void)ptcb;                                                         /* Prevent compiler warning                                 */
#endif
}
#endif


/*
*********************************************************************************************************
*                                           TASK DELETION HOOK
*
* Description: This function is called when a task is deleted.
*
* Arguments  : ptcb   is a pointer to the task control block of the task being deleted.
*
* Note(s)    : 1) Interrupts are disabled during this call.
*********************************************************************************************************
*/
#if OS_CPU_HOOKS_EN > 0
void  OSTaskDelHook (OS_TCB *ptcb)
{
#if OS_APP_HOOKS_EN > 0
    App_TaskDelHook(ptcb);
#else
    (void)ptcb;                                                         /* Prevent compiler warning                                 */
#endif
}
#endif

/*
*********************************************************************************************************
*                                             IDLE TASK HOOK
*
* Description: This function is called by the idle task.  This hook has been added to allow you to do
*              such things as STOP the CPU to conserve power.
*
* Arguments  : none
*
* Note(s)    : 1) Interrupts are enabled during this call.
*********************************************************************************************************
*/
#if (OS_CPU_HOOKS_EN > 0) && (OS_VERSION >= 251)
void  OSTaskIdleHook (void)
{
#if OS_APP_HOOKS_EN > 0
    App_TaskIdleHook();
#endif
}
#endif

/*
*********************************************************************************************************
*                                           STATISTIC TASK HOOK
*
* Description: This function is called every second by uC/OS-II's statistics task.  This allows your
*              application to add functionality to the statistics task.
*
* Arguments  : none
*********************************************************************************************************
*/

#if OS_CPU_HOOKS_EN > 0
void  OSTaskStatHook (void)
{
#if OS_APP_HOOKS_EN > 0
    App_TaskStatHook();
#endif
}
#endif

/*$PAGE*/
/*
**********************************************************************************************************
*                                       INITIALIZE A TASK'S STACK
*
* Description: This function is called by either OSTaskCreate() or OSTaskCreateExt() to initialize the
*              stack frame of the task being created. This function is highly processor specific.
*
* Arguments  : task          is a pointer to the task code
*
*              p_arg         is a pointer to a user supplied data area that will be passed to the task
*                            when the task first executes.
*
*              ptos          is a pointer to the top of stack. It is assumed that 'ptos' points to the
*                            highest valid address on the stack.
*
*              opt           specifies options that can be used to alter the behavior of OSTaskStkInit().
*                            (see uCOS_II.H for OS_TASK_OPT_???).
*
* Returns    : Always returns the location of the new top-of-stack' once the processor registers have
*              been placed on the stack in the proper order.
*
* Note(s)    : 1) Avoid AVR32Studio warning "Target request failed: cannot access memory at address #" at debug mode.
*
*              2) Prevent AVR32Studio prematurely exiting from debug mode at OSCtxRestore due to invalid memory access.
**********************************************************************************************************
*/

OS_STK  *OSTaskStkInit (void (*task)(void *pd), void *p_arg, OS_STK *ptos, INT16U opt)
{
    OS_STK *stk;


    (void)opt;                                                          /* 'opt' is not used, prevent warning                       */

     stk   =          ptos;
    *stk-- = (INT32U) 0x08080808;                                       /* R8                                                       */
    *stk-- = (INT32U) 0x09090909;                                       /* R9                                                       */
    *stk-- = (INT32U) 0x10101010;                                       /* R10                                                      */
    *stk-- = (INT32U) 0x11111111;                                       /* R11                                                      */
    *stk-- = (INT32U) p_arg;                                            /* R12                                                      */
    *stk-- = (INT32U) 0x80000000;                                       /* R14/LR (reset vector for GNU C) (note 1)                 */
    *stk-- = (INT32U) task;                                             /* R15/PC                                                   */
    *stk-- = (INT32U) OS_CPU_SR_M0_MASK;                                /* SR: Supervisor mode                                      */
    *stk-- = (INT32U) 0xFF0000FF;                                       /* R0                                                       */
    *stk-- = (INT32U) 0x01010101;                                       /* R1                                                       */
    *stk-- = (INT32U) 0x02020202;                                       /* R2                                                       */
    *stk-- = (INT32U) 0x03030303;                                       /* R3                                                       */
    *stk-- = (INT32U) 0x04040404;                                       /* R4                                                       */
    *stk-- = (INT32U) 0x05050505;                                       /* R5                                                       */
    *stk-- = (INT32U) 0x06060606;                                       /* R6                                                       */
    *stk   = (INT32U) 0x00000000;                                       /* R7 (note 2)                                              */

    return (stk);
 }

/*
*********************************************************************************************************
*                                           TASK SWITCH HOOK
*
* Description: This function is called when a task switch is performed.  This allows you to perform other
*              operations during a context switch.
*
* Arguments  : none
*
* Note(s)    : 1) Interrupts are disabled during this call.
*              2) It is assumed that the global pointer 'OSTCBHighRdy' points to the TCB of the task that
*                 will be 'switched in' (i.e. the highest priority task) and, 'OSTCBCur' points to the
*                 task being switched out (i.e. the preempted task).
*********************************************************************************************************
*/
#if (OS_CPU_HOOKS_EN > 0) && (OS_TASK_SW_HOOK_EN > 0)
void  OSTaskSwHook (void)
{
#if OS_APP_HOOKS_EN > 0
    App_TaskSwHook();
#endif
}
#endif

/*$PAGE*/
/*
*********************************************************************************************************
*                                           OS_TCBInit() HOOK
*
* Description: This function is called by OS_TCBInit() after setting up most of the TCB.
*
* Arguments  : ptcb    is a pointer to the TCB of the task being created.
*
* Note(s)    : 1) Interrupts may or may not be ENABLED during this call.
*********************************************************************************************************
*/
#if (OS_CPU_HOOKS_EN > 0) && (OS_VERSION > 203)
void  OSTCBInitHook (OS_TCB *ptcb)
{
#if OS_APP_HOOKS_EN > 0
    App_TCBInitHook(ptcb);
#else
    (void)ptcb;                                                         /* Prevent compiler warning                                 */
#endif
}
#endif


/*
*********************************************************************************************************
*                                               TICK HOOK
*
* Description: This function is called every tick.
*
* Arguments  : none
*
* Note(s)    : 1) Interrupts may or may not be ENABLED during this call.
*********************************************************************************************************
*/
#if (OS_CPU_HOOKS_EN > 0) && (OS_TIME_TICK_HOOK_EN > 0)
void  OSTimeTickHook (void)
{
#if OS_APP_HOOKS_EN > 0
    App_TimeTickHook();
#endif

#if (OS_VERSION >= 281) && (OS_TMR_EN > 0)
    OSTmrCtr++;
    if (OSTmrCtr >= (OS_TICKS_PER_SEC / OS_TMR_CFG_TICKS_PER_SEC)) {
        OSTmrCtr = 0;
        OSTmrSignal();
    }
#endif
}
#endif


/*$PAGE*/
/*
*********************************************************************************************************
*                                    INTERRUPT LEVEL CONTEXT SWITCH
*
* Description: This function is called by OSIntExit() to perform a context switch from an ISR.
*
* Arguments  : none
*
* Note(s)    : 1) The stack frame is assumed to look as follows:
*
*                  OSTCBHighRdy->OSTCBStkPtr --> SP   -->
*                                                R7   ----/   (Low memory)
*                                                .
*                                                .
*                                                R0
*                                                SR
*                                                PC
*                                                LR
*                                                R12
*                                                .
*                                                .
*                                                R8   ----\   (High memory)
*
*                  where the stack pointer points to the task start address.
*
*              2) OSIntCtxSw() MUST:
*                      a) Call OSTaskSwHook() then,
*                      b) Set OSTCBCur = OSTCBHighRdy,
*                      c) Set OSPrioCur = OSPrioHighRdy,
*                      d) Switch to the highest priority task.
*********************************************************************************************************
*/

void  OSIntCtxSw (void)
{
    OSTaskSwHook();

    OSTCBCur  =  OSTCBHighRdy;
    OSPrioCur =  OSPrioHighRdy;

    OSIntCtxRestore(OSTCBHighRdy->OSTCBStkPtr);
}


/*
*********************************************************************************************************
*                              START HIGHEST PRIORITY TASK READY-TO-RUN
*
* Description: This function is called by OSStart() to start the highest priority task that was created
*              by your application before calling OSStart().
*
* Arguments  : none
*
* Note(s)    : 1) The stack frame is assumed to look as follows:
*
*                  OSTCBHighRdy->OSTCBStkPtr --> SP   -->
*                                                R7   ----/   (Low memory)
*                                                .
*                                                .
*                                                R0
*                                                SR
*                                                PC
*                                                LR
*                                                R12
*                                                .
*                                                .
*                                                R8   ----\   (High memory)
*
*                  where the stack pointer points to the task start address.
*
*              2) OSStartHighRdy() MUST:
*                      a) Call OSTaskSwHook() then,
*                      b) Set OSRunning to TRUE,
*                      c) Switch to the highest priority task.
*********************************************************************************************************
*/

void  OSStartHighRdy (void)
{
    OSTaskSwHook();
    OSRunning =  1;
    OSCtxRestore(OSTCBHighRdy->OSTCBStkPtr);
}
