/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

#include <FreeRTOSConfig.h>

	RSEG    CODE:CODE(2)
	thumb

	EXTERN pxCurrentTCB
	EXTERN vTaskSwitchContext
        EXTERN vSVCHandler

	PUBLIC xPortPendSVHandler
	PUBLIC vPortSVCHandler
	PUBLIC vPortStartFirstTask
        PUBLIC vRestoreContextOfFirstTask
	PUBLIC vPortEnableVFP
        PUBLIC xPortRaisePrivilege
        PUBLIC vPortSwitchToUserMode
        


/*-----------------------------------------------------------*/

xPortPendSVHandler:
	mrs r0, psp
	isb
	/* Get the location of the current TCB. */
	ldr	r3, =pxCurrentTCB
	ldr	r2, [r3]

	/* Is the task using the FPU context?  If so, push high vfp registers. */
	tst r14, #0x10
	it eq
	vstmdbeq r0!, {s16-s31}

	mrs r1, control
        /* Save the core registers. */
	stmdb r0!, {r1, r4-r11, r14}

	/* Save the new top of stack into the first member of the TCB. */
	str r0, [r2]

	stmdb sp!, {r3}
	mov r0, #configMAX_SYSCALL_INTERRUPT_PRIORITY
  cpsid i
	msr basepri, r0
	dsb
	isb
  cpsie i
	bl vTaskSwitchContext
	mov r0, #0
	msr basepri, r0
	ldmia sp!, {r3}

	/* The first item in pxCurrentTCB is the task top of stack. */
	ldr r1, [r3]
	ldr r0, [r1]
        
 	add r1, r1, #4					/* Move onto the second item in the TCB... */
	ldr r2, =0xe000ed9c				/* Region Base Address register. */
	ldmia r1!, {r4-r11}				/* Read 4 sets of MPU registers. */
	stmia r2!, {r4-r11}				/* Write 4 sets of MPU registers. */

	/* Pop the core registers. */
	ldmia r0!, {r3-r11, r14}
        msr control, r3

	/* Is the task using the FPU context?  If so, pop the high vfp registers
	too. */
	tst r14, #0x10
	it eq
	vldmiaeq r0!, {s16-s31}

	msr psp, r0
	isb
	#ifdef WORKAROUND_PMU_CM001 /* XMC4000 specific errata */
		#if WORKAROUND_PMU_CM001 == 1
			push { r14 }
			pop { pc }
		#endif
	#endif

	bx r14


/*-----------------------------------------------------------*/

vPortSVCHandler:
	/* Assumes psp was in use. */
	#ifndef USE_PROCESS_STACK	/* Code should not be required if a main() is using the process stack. */
		tst lr, #4
		ite eq
		mrseq r0, msp
		mrsne r0, psp
	#else
		mrs r0, psp
	#endif
		b vSVCHandler

/*-----------------------------------------------------------*/

vPortStartFirstTask:
	/* Use the NVIC offset register to locate the stack. */
	ldr r0, =0xE000ED08
	ldr r0, [r0]
	ldr r0, [r0]
	/* Set the msp back to the start of the stack. */
	msr msp, r0
	/* Call SVC to start the first task. */
	cpsie i
	cpsie f
	dsb
	isb
	svc 0	/* System call to start first task. */
  
vRestoreContextOfFirstTask:

	ldr r0, =0xE000ED08				/* Use the NVIC offset register to locate the stack. */
	ldr r0, [r0]
	ldr r0, [r0]
	msr msp, r0						/* Set the msp back to the start of the stack. */
	ldr	r3, =pxCurrentTCB			/* Restore the context. */
	ldr r1, [r3]
	ldr r0, [r1]					/* The first item in the TCB is the task top of stack. */
	add r1, r1, #4					/* Move onto the second item in the TCB... */
	ldr r2, =0xe000ed9c				/* Region Base Address register. */
	ldmia r1!, {r4-r11}				/* Read 4 sets of MPU registers. */
	stmia r2!, {r4-r11}				/* Write 4 sets of MPU registers. */
	ldmia r0!, {r3-r11, r14}	/* Pop the registers that are not automatically saved on exception entry. */
	msr control, r3
	msr psp, r0						/* Restore the task stack pointer. */
	mov r0, #0
	msr	basepri, r0
	bx r14
  
/*-----------------------------------------------------------*/

vPortEnableVFP:
	/* The FPU enable bits are in the CPACR. */
	ldr.w r0, =0xE000ED88
	ldr	r1, [r0]

	/* Enable CP10 and CP11 coprocessors, then save back. */
	orr	r1, r1, #( 0xf << 20 )
	str r1, [r0]
	bx	r14

xPortRaisePrivilege:
	mrs r0, control
	tst r0, #1						/* Is the task running privileged? */
	itte ne
	movne r0, #0					/* CONTROL[0]!=0, return false. */
	svcne 2                                  	/* Switch to privileged. */
	moveq r0, #1					/* CONTROL[0]==0, return true. */
	bx lr
        
/*-----------------------------------------------------------*/



vPortSwitchToUserMode:
	
	mrs r0, control
	orr r0, r0, #1
	msr control, r0
	bx r14

	END

