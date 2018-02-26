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

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE. 
 *
 * See http://www.freertos.org/a00110.html.
 *----------------------------------------------------------*/

#define configUSE_PREEMPTION								1	//1使用抢占式内核，0使用协程
#define configUSE_TIME_SLICING							1	//1使能时间片调度
#define configUSE_PORT_OPTIMISED_TASK_SELECTION	1 //1启用特殊方法来选择下一个要运行的任务
                                                  //一般是硬件计算前导零指令，如果所使用的
																									//MCU没有这些硬件指令的话此宏应该设置为0！
#define configUSE_TICKLESS_IDLE							0	//1启用低功耗tickless模式
#define configUSE_QUEUE_SETS								1	//1启用队列
#define configUSE_IDLE_HOOK									0	//不使用空闲任务钩子
#define configUSE_TICK_HOOK									0	//不使用时间片钩子
#define configCPU_CLOCK_HZ									((unsigned long )72000000)//系统时钟
#define configTICK_RATE_HZ									((TickType_t)1000)//系统节拍
#define configMAX_PRIORITIES								(10)//可用的最大优先级
#define configMINIMAL_STACK_SIZE						((unsigned short )128)//空闲人物堆栈大小
#define configSUPPORT_DYNAMIC_ALLOCATION 		1		//支持动态内存申请
#define configTOTAL_HEAP_SIZE								((size_t)(12*1024))//总堆大小 12k
#define configMAX_TASK_NAME_LEN							(16)//最大任务名字节数
#define configUSE_TRACE_FACILITY						0		//关闭跟踪调试
#define configUSE_16_BIT_TICKS							0		//使用32为Ticks				
#define configIDLE_SHOULD_YIELD							1		
#define configUSE_TASK_NOTIFICATIONS				1		//信号量支持
#define configUSE_RECURSIVE_MUTEXES					1		//使用互斥信号量
#define configUSE_MUTEXES										1		//使用互斥信号量

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES 		0
#define configMAX_CO_ROUTINE_PRIORITIES (2)
/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */
#define INCLUDE_vTaskPrioritySet			1
#define INCLUDE_uxTaskPriorityGet			1
#define INCLUDE_vTaskDelete						1
#define INCLUDE_vTaskCleanUpResources	1
#define INCLUDE_vTaskSuspend					1
#define INCLUDE_vTaskDelayUntil				1
#define INCLUDE_vTaskDelay						1

/* This is the raw value as per the Cortex-M3 NVIC.  Values can be 255
(lowest) to 0 (1?) (highest). */
#define configKERNEL_INTERRUPT_PRIORITY 		255
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
//STM32使用了Cortex M中断优先级位中的4位（高4位） 所以191 = 0b10111111 >> 4 = 0b1011 = 0xb0 = 11
//配置后FreeRTOS将能屏蔽所有中断优先级大于11的中断
//使用的优先级位数可以在 CMSIS中的__NVIC_PRIO_BITS查询
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	191 /* equivalent to 0xb0, or priority 11. */


/* This is the value being used as per the ST library which permits 16
priority values, 0 to 15.  This must correspond to the
configKERNEL_INTERRUPT_PRIORITY setting.  Here 15 corresponds to the lowest
NVIC value of 255. */
#define configLIBRARY_KERNEL_INTERRUPT_PRIORITY	15

#endif /* FREERTOS_CONFIG_H */

