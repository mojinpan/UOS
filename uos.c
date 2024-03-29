/** ****************************************************************************
* @file 	uos.c
* @author 	mojinpan
* @copyright (c) 2018 - 2019 BRC Tech. Co., Ltd.
* @brief 		微型嵌入式操作系统
* 
* @version 	V0.1 
* @date 	2019-05-18 
* @details
* 1.开始对UCOSII进行精简
* 2.将汇编整合到C文件中
* 
* @version 	V0.2 
* @date 	2019-05-26 
* @details
* 1.实现最基本的任务调度
* 
* @version 	V0.3 
* @date 	2019-05-27 
* @details
* 1.时钟节拍,实现OS级的延时支持
* 
* @version 	V0.4 
* @date 	2019-05-28 
* @details
* 1.增加中断任务切换支持
* 
* @version 	V0.5 
* @date 	2019-06-09 
* @details
* 1.增加系统时间的支持 
* 2.增加软定时器的支持
* 
* @version 	V0.6 
* @date 	2019-10-22
* @details
* 1.重写任务队列部分代码,支持动态加载,并简化操作
*
* @version 	V0.7 
* @date 	2020-07-31
* @details
* 1.修改OSTaskStkInit()和PendSV_Handler(),调整浮点寄出去出栈和入栈的操作,修正开启FPU后会出现硬件错误的bug
*
* @version 	V0.8 
* @date 	2021-01-30
* @details
* 1.将优先级改为从高到低,并启用CLZ(Count Leading Zeros)指令进行优先级查找,加速优先级查找并保证查找时间一致

* @version 	V0.9 
* @date 	2021-02-08
* @details
* 1.改写嵌汇编部分代码,使其能在AC6编译模式下耐受大部分编译等级(-O0除外)
*******************************************************************************/
#include "stdint.h"
#include "stdbool.h"

#ifdef CPU_MIMXRT1052CVL5B
#include "MIMXRT1052.h"
#endif

#ifdef STM32F767xx
#include "stm32f767xx.h"
#endif

#ifdef STM32F401xC
#include "stm32f401xc.h"
#endif

#ifdef STM32H750xx
#include "stm32h750xx.h"
#endif


#include "uos.h"

/*******************************************************************************
							Macro Definition 宏定义
*******************************************************************************/
#define NVIC_INT_CTRL       0xE000ED04      // 中断控制寄存器
#define NVIC_PENDSVSET      0x10000000      // 触发PendSV异常的值

#define NVIC_SYSPRI14       0xE000ED22      // 系统优先级寄存器（优先级14）
#define NVIC_PENDSV_PRI     0xFF            // PendSV优先级值（最低）

#define OSIntCtxSw()        OSCtxSw() 
#define OS_TASK_SW()        OSCtxSw() 

#if defined(__CC_ARM)
#define GET_HIGH_PRIO(HighRdy, RdyGrp)    HighRdy = (31UL - (uint32_t) __clz((RdyGrp)))
#elif  defined(__GNUC__) || defined(__CLANG_ARM)
#define GET_HIGH_PRIO(HighRdy, RdyGrp)    HighRdy = (31UL - (uint32_t)CountLeadingZeros((RdyGrp)))
#endif


void OS_SchedNew(void);
/*******************************************************************************
                            Global Variables    全局变量
*******************************************************************************/
bool     OSRunning;
uint8_t  OSIntNesting;
uint8_t  OSPrioCur;                         // 当前任务的优先级 
uint8_t  OSRdyGrp;                          // 任务就绪列表
uint8_t  OSPrioHighRdy;                     // 优先级最高的任务
volatile  unsigned int  OSTime;                 // 系统时间的当前值（以ticks表示）

OS_TCB  *OSTCBCur;                          // 当前任务控制块指针 
OS_TCB  *OSTCBHighRdy;                      // 优先级最高的任务控制块
OS_TCB  OSTCBTbl[OS_MAX_TASKS];             // 任务控制块列表
#if OS_TMR_CFG_MAX > 0
OS_TMR  OSTmrTbl[OS_TMR_CFG_MAX];           // 软定时器列表
#endif

TQUE_TaskNode *taskList[TQUE_MAX_TASK_NODE_NUM];   //任务列表
uint16_t taskRunNo;                                //任务队列中正在运行的任务

#if  defined(__GNUC__) || defined(__CLANG_ARM)
/** ****************************************************************************
* @brief 前导零数量计算
*******************************************************************************/
__attribute__((always_inline)) static inline uint8_t CountLeadingZeros(uint32_t RdyGrp)
{
    uint8_t Zeros;
    __asm volatile ( "clz %0, %1" : "=r" ( Zeros ) : "r" (RdyGrp) : "memory" );
    return Zeros;
}

/** ****************************************************************************
* @brief 保存中断状态,并关中断
* @return 返回关中断前的中断状态
*******************************************************************************/
inline __attribute__( ( always_inline ) ) uint32_t OS_CPU_SR_Save( void )
{
    uint32_t res, new;
    __asm volatile
    (
        "	mrs %0, basepri											\n"\
        "	mov %1, %2												\n"\
        "	msr basepri, %1											\n"\
        "	isb														\n"\
        "	dsb														\n"\
        : "=r" ( res ), "=r" ( new ) : "i" ( OS_INT_PRIO<<4 ) : "memory"
    );
    return res;
}
/** ****************************************************************************
* @brief 还原中断状态
* @param cpu_sr 中断状态
* @note 在多个带临界保护的函数嵌套使用时,如果直接开中断会导致上层临界保护失效,所以此处不直接开中断
*******************************************************************************/
inline __attribute__( ( always_inline ) ) void OS_CPU_SR_Restore( uint32_t new )
{
    __asm volatile
    (
        "	msr basepri, %0	"::"r" ( new ) : "memory"
    );
}
/** ****************************************************************************
* @brief 触发PendSV中断,通过PendSV中断进行任务切换
*******************************************************************************/
#define OSCtxSw()                                   \
{                                                   \
    ( *( ( volatile uint32_t * ) NVIC_INT_CTRL ) ) = NVIC_PENDSVSET;                 \
    __asm volatile ( "dsb" ::: "memory" );          \
    __asm volatile ( "isb" );                       \
}
/** ****************************************************************************
* @brief  任务切换
*******************************************************************************/
void PendSV_Handler(void)
{
    __asm volatile (
    
    "CPSID   I                                   \n"// OS_ENTER_CRITICAL();
    "MRS     R0, PSP                             \n"// R0 = PSP;
    "CBZ     R0, PendSV_Handler_nosave           \n"// if(R0 == 0) goto PendSV_Handler_NoSave;
    // 入栈(PUSH)
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    //入栈浮点寄存器
    "SUBS    R0, R0, #0X40                       \n"
    "VSTM    R0, {S16-S31}                       \n"
#endif  
    "SUBS    R0, R0, #0x20                       \n"// R0 = R0 - 0x20;栈顶减32，即8Word,跳过中断自动保存的XPSR,PC,LR,R12,R13,R2,R1,R0
    "STM     R0, {R4-R11}                        \n"// 压栈R4~R11,8个寄存器  
    // 保存PSP指针
    "LDR     R1, pOSTCBCur                       \n"// R1 = OSTCBCur
    "LDR     R1, [R1]                            \n"// R1 = *R1;(R1 = OSTCBCur->OSTCBStkPtr)
    "STR     R0, [R1]                            \n"// *R1 = R0;(*(OSTCBCur->OSTCBStkPrt) = R0)

    // At this point, entire context of process has been saved
    "PendSV_Handler_nosave:                      \n"
 
    "LDR     R0, pOSPrioCur                      \n"// R0 =  &OSPrioCur                   
    "LDR     R1, pOSPrioHighRdy                  \n"// R1 =  &OSPrioHighRdy
    "LDRB    R2, [R1]                            \n"// R2 = *R1
    "STRB    R2, [R0]                            \n"// *R0 = R2  --->  OSPrioCur = OSPrioHighRdy                            
    // OSTCBCur  = OSTCBHighRdy;
    "LDR     R0, pOSTCBCur                              \n"// R0 = &OSTCBCur                       
    "LDR     R1, pOSTCBHighRdy                              \n"// R1 = &OSTCBHighRdy
    "LDR     R2, [R1]                            \n"// R2 = OSTCBHighRdy->OSTCBStkPtr;
    "STR     R2, [R0]                            \n"// *R0 = R2;(OSTCBCur->OSTCBStkPtr = OSTCBHighRdy->OSTCBStkPtr)
    // 出栈(POP)
    "LDR     R0, [R2]                            \n"// R0 is new process SP; SP = OSTCBHighRdy->OSTCBStkPtr;

    "LDM     R0, {R4-R11}                        \n"// Restore r4-11 from new process stack
    "ADDS    R0, R0, #0x20                       \n"// 堆栈指针(R0)向下0x20;栈顶加32,指向保存R0-xPSR段
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    //出栈浮点寄存器 
    "VLDM    R0, {S16-S31}                       \n"
    "ADDS    R0, R0, #0X40                       \n"
#endif
    "MSR     PSP, R0                             \n"// PSP = R0;(PSP = OSPrioCur->OSTCBStkPtr)
    "ORR     LR, LR, #0x04                       \n"// LR = LR | 0x04;中断退出后进入任务模式，使用PSP栈
    "CPSIE   I                                   \n"// OS_EXIT_CRITICAL();
    "BX      LR                                  \n"// return;
    ".align 4                                    \n"
    "pOSTCBCur:  .word OSTCBCur                  \n"
    "pOSPrioCur: .word OSPrioCur                 \n"
    "pOSPrioHighRdy:  .word OSPrioHighRdy        \n"
    "pOSTCBHighRdy:  .word OSTCBHighRdy          \n"
	);
}
/** ****************************************************************************
* @brief  OS启动并切换至最高优先级任务
*******************************************************************************/
void OSStartHighRdy(void)
{
    __asm volatile (
        "LDR     R0, =0xE000ED22            \n"   // Set the PendSV exception priority
        "LDR     R1, =0xFF                  \n"
        "STRB    R1, [R0]                   \n"   //*R0 = R1 即将PendSV的优先级设置为最低

        "MOVS    R0, #0                     \n"   // Set the PSP to 0 for initial context switch call
        "MSR     PSP, R0                    \n"   // PSP = R0 栈指针归零初始化

        "LDR     R0, =0xE000ED04            \n"   // Trigger the PendSV exception (causes context switch)
        "LDR     R1, =0x10000000            \n"
        "STR     R1, [R0]                   \n"   // *R0 = R1 触发PendSV中断
        " nop					            \n"
        "	.align 4						\n"
        );
}

#endif

#ifdef __CC_ARM
/** ****************************************************************************
* @brief 保存中断状态,并关中断
* @return 返回关中断前的中断状态
*******************************************************************************/
__asm uint32_t OS_CPU_SR_Save(void)
{
    PRESERVE8
    
    MRS     R0, PRIMASK //保存 PRIMASK的值
    CPSID   I           //关中断（除了fault 和 NMI）
    BX      LR          //return R0
}

/** ****************************************************************************
* @brief 还原中断状态
* @param cpu_sr 中断状态
* @note 在多个带临界保护的函数嵌套使用时,如果直接开中断会导致上层临界保护失效,所以此处不直接开中断
*******************************************************************************/
__asm void OS_CPU_SR_Restore(uint32_t cpu_sr)
{
    PRESERVE8
    
    MSR     PRIMASK, R0 //还原PRIMASK的值 传参cpu_sr会默认保存到R0上
    BX      LR          //return 
}

/** ****************************************************************************
* @brief 触发PendSV中断,通过PendSV中断进行任务切换
*******************************************************************************/
__asm void OSCtxSw(void)
{
    PRESERVE8
    
    LDR     R0, =NVIC_INT_CTRL                     // Trigger the PendSV exception (causes context switch)
    LDR     R1, =NVIC_PENDSVSET                     // Value to trigger PendSV exception.              
    STR     R1, [R0]
    BX      LR
    ALIGN
}

/** ****************************************************************************
* @brief  OS启动并切换至最高优先级任务
*******************************************************************************/
__asm void OSStartHighRdy(void)
{
    PRESERVE8

    LDR     R0, =NVIC_SYSPRI14                                  // Set the PendSV exception priority
    LDR     R1, =NVIC_PENDSV_PRI
    STRB    R1, [R0]                                            //*R0 = R1 即将PendSV的优先级设置为最低

    MOVS    R0, #0                                              // Set the PSP to 0 for initial context switch call
    MSR     PSP, R0                                             // PSP = R0 栈指针归零初始化

    LDR     R0, =NVIC_INT_CTRL                                  // Trigger the PendSV exception (causes context switch)
    LDR     R1, =NVIC_PENDSVSET
    STR     R1, [R0]                                            // *R0 = R1 触发PendSV中断

    CPSIE   I                                                   // Enable interrupts at processor level

OSStartHang
    B       OSStartHang                                         // while(1); 
    ALIGN
}
/** ****************************************************************************
* @brief  任务切换
*******************************************************************************/
__asm void PendSV_Handler(void)
{
    IMPORT  OSPrioCur
    IMPORT  OSPrioHighRdy
    IMPORT  OSTCBCur
    IMPORT  OSTCBHighRdy
    
    PRESERVE8
    
    CPSID   I                                   // OS_ENTER_CRITICAL();
    MRS     R0, PSP                             // R0 = PSP;
    CBZ     R0, PendSV_Handler_nosave           // if(R0 == 0) goto PendSV_Handler_NoSave;
    // 入栈(PUSH)
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
//入栈浮点寄存器
    SUBS    R0, R0, #0X40 
    VSTM    R0, {S16-S31} 
#endif  
    SUBS    R0, R0, #0x20                       // R0 = R0 - 0x20;栈顶减32，即8Word,跳过中断自动保存的XPSR,PC,LR,R12,R13,R2,R1,R0
    STM     R0, {R4-R11}                        // 压栈R4~R11,8个寄存器  
    // 保存PSP指针
    LDR     R1, =OSTCBCur                       // R1 = OSTCBCur
    LDR     R1, [R1]                            // R1 = *R1;(R1 = OSTCBCur->OSTCBStkPtr)
    STR     R0, [R1]                            // *R1 = R0;(*(OSTCBCur->OSTCBStkPrt) = R0)

                                                // At this point, entire context of process has been saved
PendSV_Handler_nosave
 
    LDR     R0, =OSPrioCur                      // R0 =  &OSPrioCur                   
    LDR     R1, =OSPrioHighRdy                  // R1 =  &OSPrioHighRdy
    LDRB    R2, [R1]                            // R2 = *R1
    STRB    R2, [R0]                            // *R0 = R2  --->  OSPrioCur = OSPrioHighRdy                            
    // OSTCBCur  = OSTCBHighRdy;
    LDR     R0, =OSTCBCur                       // R0 = &OSTCBCur                       
    LDR     R1, =OSTCBHighRdy                   // R1 = &OSTCBHighRdy
    LDR     R2, [R1]                            // R2 = OSTCBHighRdy->OSTCBStkPtr;
    STR     R2, [R0]                            // *R0 = R2;(OSTCBCur->OSTCBStkPtr = OSTCBHighRdy->OSTCBStkPtr)
    // 出栈(POP)
    LDR     R0, [R2]                            // R0 is new process SP; SP = OSTCBHighRdy->OSTCBStkPtr;

    LDM     R0, {R4-R11}                        // Restore r4-11 from new process stack
    ADDS    R0, R0, #0x20                       // 堆栈指针(R0)向下0x20;栈顶加32,指向保存R0-xPSR段
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    //出栈浮点寄存器 
    VLDM    R0, {S16-S31} 
    ADDS    R0, R0, #0X40 
#endif
    MSR     PSP, R0                             // PSP = R0;(PSP = OSPrioCur->OSTCBStkPtr)
    ORR     LR, LR, #0x04                       // LR = LR | 0x04;中断退出后进入任务模式，使用PSP栈
    CPSIE   I                                   // OS_EXIT_CRITICAL();
    BX      LR                                  // return;
    ALIGN
}

#endif


/** ****************************************************************************
* @brief  任务堆栈初始化
* @param	void (*task)(void *p_arg)	函数指针变量
* @param	*p_arg	                    任务参数指针
* @param	*ptos	                    堆栈栈顶指针
*******************************************************************************/
uint32_t *OSTaskStkInit(void (*task)(void *p_arg), void *p_arg, uint32_t *ptos)
{
    volatile uint32_t *p_stk;

    p_stk      = ptos + 1u;                                       // Load stack pointer                                   
                                                                    // Align the stack to 8-bytes.                          
    p_stk      = (uint32_t *)((uint32_t)(p_stk) & 0xFFFFFFF8u);   // Registers stacked as if auto-saved on exception   
    #if(__FPU_PRESENT == 1)&&(__FPU_USED == 1) 
    *--p_stk = (uint32_t)0x00000000u;                        /*unknow register                                         */ 
    *--p_stk = (uint32_t)0x00000000u;                        /*FPSCR                                                   */ 
    *--p_stk = (uint32_t)0x15151515u;                        /*S15                                                     */ 
    *--p_stk = (uint32_t)0x14141414u;                        /*S14                                                     */ 
    *--p_stk = (uint32_t)0x13131313u;                        /*S13                                                     */ 
    *--p_stk = (uint32_t)0x12121212u;                        /*S12                                                     */ 
    *--p_stk = (uint32_t)0x11111111u;                        /*S11                                                     */ 
    *--p_stk = (uint32_t)0x10101010u;                        /*S10                                                     */ 
    *--p_stk = (uint32_t)0x09090909u;                        /*S09                                                     */ 
    *--p_stk = (uint32_t)0x08080808u;                        /*S08                                                     */ 
    *--p_stk = (uint32_t)0x07070707u;                        /*S07                                                     */ 
    *--p_stk = (uint32_t)0x06060606u;                        /*S06                                                     */ 
    *--p_stk = (uint32_t)0x05050505u;                        /*S05                                                     */ 
    *--p_stk = (uint32_t)0x04040404u;                        /*S04                                                     */ 
    *--p_stk = (uint32_t)0x03030303u;                        /*S03                                                     */ 
    *--p_stk = (uint32_t)0x02020202u;                        /*S02                                                     */ 
    *--p_stk = (uint32_t)0x01010101u;                        /*S01                                                     */ 
    *--p_stk = (uint32_t)0x00000000u;                        /*S00                                                     */ 
    #endif   
   *(--p_stk) = (uint32_t)0x01000000uL;                          // xPSR                                                 
   *(--p_stk) = (uint32_t)task;                                  // R15 (PC)                                       
   *(--p_stk) = (uint32_t)0xFFFFFFFEuL;                          // R14 (LR)                                             
   *(--p_stk) = (uint32_t)0x12121212uL;                          // R12                                                  
   *(--p_stk) = (uint32_t)0x03030303uL;                          // R3                                                   
   *(--p_stk) = (uint32_t)0x02020202uL;                          // R2                                                   
   *(--p_stk) = (uint32_t)0x01010101uL;                          // R1                                                   
   *(--p_stk) = (uint32_t)p_arg;                                 // R0 : argument                                        
                                                                 // Remaining registers saved on process stack   
    #if(__FPU_PRESENT == 1)&&(__FPU_USED == 1) 
    *--p_stk = (uint32_t)0x31313131u;                        /*S31                                                     */ 
    *--p_stk = (uint32_t)0x30303030u;                        /*S30                                                     */ 
    *--p_stk = (uint32_t)0x29292929u;                        /*S29                                                     */ 
    *--p_stk = (uint32_t)0x28282828u;                        /*S28                                                     */ 
    *--p_stk = (uint32_t)0x27272727u;                        /*S27                                                     */ 
    *--p_stk = (uint32_t)0x26262626u;                        /*S26                                                     */ 
    *--p_stk = (uint32_t)0x25252525u;                        /*S25                                                     */ 
    *--p_stk = (uint32_t)0x24242424u;                        /*S24                                                     */ 
    *--p_stk = (uint32_t)0x23232323u;                        /*S23                                                     */ 
    *--p_stk = (uint32_t)0x22222222u;                        /*S22                                                     */ 
    *--p_stk = (uint32_t)0x21212121u;                        /*S21                                                     */ 
    *--p_stk = (uint32_t)0x20202020u;                        /*S20                                                     */ 
    *--p_stk = (uint32_t)0x19191919u;                        /*S19                                                     */ 
    *--p_stk = (uint32_t)0x18181818u;                        /*S18                                                     */ 
    *--p_stk = (uint32_t)0x17171717u;                        /*S17                                                     */ 
    *--p_stk = (uint32_t)0x16161616u;                        /*S16                                                     */ 
    #endif      
   *(--p_stk) = (uint32_t)0x11111111uL;                          // R11                                                  
   *(--p_stk) = (uint32_t)0x10101010uL;                          // R10                                                  
   *(--p_stk) = (uint32_t)0x09090909uL;                          // R9                                                   
   *(--p_stk) = (uint32_t)0x08080808uL;                          // R8                                                   
   *(--p_stk) = (uint32_t)0x07070707uL;                          // R7                                                   
   *(--p_stk) = (uint32_t)0x06060606uL;                          // R6                                                   
   *(--p_stk) = (uint32_t)0x05050505uL;                          // R5                                                   
   *(--p_stk) = (uint32_t)0x04040404uL;                          // R4   
   return (uint32_t*)(p_stk);
}
/** ****************************************************************************
* @brief	初始化任务控制块
* @param	prio	    任务优先级
* @param	*ptos	    堆栈栈顶指针
* @param	stk_size	堆栈大小
*******************************************************************************/
void  OS_TCBInit(uint8_t prio,uint32_t *ptos,uint32_t stk_size)
{
    OS_ENTER_CRITICAL();
    OSTCBTbl[prio].OSTCBStkPtr  = ptos;
    OSTCBTbl[prio].OSTCBStkSize = stk_size;
    OSTCBTbl[prio].OSTCBDly     = 0;
    OSTCBTbl[prio].OSTCBStat    = OS_STAT_RDY;
    OSTCBTbl[prio].OSTCBPrio    = prio;
    OSRdyGrp                   |= (1 << prio);
    OS_EXIT_CRITICAL();   
}
/** ****************************************************************************
* @brief  优先级查找
*******************************************************************************/
void OS_SchedNew(void)
{
#if OS_HW_PRIO_SEEK == 0
 	int8_t i;
    for (i = OS_MAX_TASKS - 1; i >=0 ; i--)
    {
        if (OSRdyGrp & (1 << i))
        {
            OSPrioHighRdy = i;
            break;
        }
    }
#else
    GET_HIGH_PRIO(OSPrioHighRdy,OSRdyGrp);
#endif
    OSTCBHighRdy = &OSTCBTbl[OSPrioHighRdy];
    // OSTCBCur = OSTCBHighRdy;
}
/** ****************************************************************************
* @brief  任务切换
*******************************************************************************/
void OS_Sched(void)
{
    if (OSIntNesting == 0u) 
    {    
        OS_ENTER_CRITICAL();   
         OS_SchedNew();
        if (OSPrioHighRdy != OSPrioCur) 
        {          
            OSCtxSw();   //优先级调度
        }
        OS_EXIT_CRITICAL();
    }
}
/** ****************************************************************************
* @brief  任务恢复
* @param	prio    任务序号
* @return	操作结果
*******************************************************************************/
uint8_t OSTaskResume(uint8_t prio)
{
    if(prio >= OS_MAX_TASKS)return OS_ERR_TASK_PRIO;

    OS_ENTER_CRITICAL();
	if (OSTCBTbl[prio].OSTCBDly == 0U)
	{
		OSRdyGrp |= (1 << prio);
		OS_EXIT_CRITICAL();
		OS_Sched();
	}
    OS_EXIT_CRITICAL();
    return OS_ERR_NONE;
}
/** ****************************************************************************
* @brief  任务挂起
* @param	prio    任务序号
* @return	操作结果
*******************************************************************************/
uint8_t OSTaskSuspend(uint8_t prio)
{
    if(prio >= OS_MAX_TASKS)return OS_ERR_TASK_PRIO;
    OS_ENTER_CRITICAL();
    OSRdyGrp &= ~(1 << prio);
    OS_EXIT_CRITICAL();
    if (prio == OSPrioCur) //如果挂起的是当前任务,则进行任务调度
	{
		OS_Sched();
	}
    return OS_ERR_NONE;
}

/** ****************************************************************************
* @brief  创建任务
* @param  *task	    任务函数指针
* @param  *p_arg	    传递给任务的参数
* @param  *ptos	    任务堆栈栈顶的指针
* @param  stk_size	任务堆栈大小
* @param  prio	    任务优先级
*******************************************************************************/
uint8_t OSTaskCreate(void (*task)(void *p_arg),void *p_arg,uint32_t *ptos,uint32_t stk_size,uint8_t prio)
{
    uint32_t *psp;

    psp = OSTaskStkInit(task,p_arg,ptos);
    OS_TCBInit(prio,psp,stk_size);

    if (OSRunning)
	{
		OS_Sched();
	}
    return 0 ;
}

/** ****************************************************************************
* @brief  OS初始化
*******************************************************************************/
void OSInit(void)
{
#if OS_TMR_CFG_MAX > 0   
    uint8_t i;
    for(i=0;i<OS_TMR_CFG_MAX; i++)  //更新定时器
    {
        OSTmrTbl[i].OSTmrDly = 0;
        OSTmrTbl[i].OSTmrUsed = false;
    }
#endif    
    OSRunning       = false;
    OSIntNesting    = 0;
    OSRdyGrp        = 0;
    OSPrioCur       = 0;
    OSPrioHighRdy   = 0;
    OSTCBCur        = (OS_TCB*)0;
    OSTCBHighRdy    = (OS_TCB*)0;
    OSTime          = 0;
    

}
/** ****************************************************************************
* @brief  启动OS
*******************************************************************************/
void  OSStart (void)
{
	int8_t i;
	
	if (!OSRunning)
	{
		for (i = OS_MAX_TASKS - 1; i >=0 ; i--)
		{
			if (OSRdyGrp & (1 << i))
			{				
				OSPrioHighRdy = i;
				OSPrioCur = OSPrioHighRdy;
				OSTCBHighRdy = &OSTCBTbl[i];
				OSTCBCur = OSTCBHighRdy;
                OSRunning = true;
				break;
			}
		}
        OS_ENTER_CRITICAL();
		OSStartHighRdy();
        OS_EXIT_CRITICAL();
	}
}
/** ****************************************************************************
* @brief  时钟节拍函数
*******************************************************************************/
void OSTimeTick(void)
{
    int8_t i;   
    if (OSRunning == true) 
    {
        OS_ENTER_CRITICAL();
        OSTime++;   //更新系统时间
        for (i = OS_MAX_TASKS - 1; i >=0 ; i--)  //更新任务延时
        {
            if (OSTCBTbl[i].OSTCBDly != 0U)
            {
                OSTCBTbl[i].OSTCBDly--;
                if (OSTCBTbl[i].OSTCBDly == 0U)
                {
                    OSRdyGrp |= (1 << i);
                }
            }
        }
#if OS_TMR_CFG_MAX > 0
        for(i = OS_MAX_TASKS - 1 ; i >=0 ; i--)  //更新定时器
        {
            if(OSTmrTbl[i].OSTmrDly != 0U)
            {
                OSTmrTbl[i].OSTmrDly--; 
            }
        }
#endif
        OS_EXIT_CRITICAL();
    }
}
/** ****************************************************************************
* @brief  延时函数
* @param	ticks    系统节拍数
*******************************************************************************/
void OSTimeDly(uint32_t ticks)
{
	OS_ENTER_CRITICAL();
	OSRdyGrp &= ~(1 << OSPrioCur);
	OSTCBTbl[OSPrioCur].OSTCBDly = ticks;
    OS_EXIT_CRITICAL();
    OS_Sched(); 
}

/** ****************************************************************************
* @brief  取消任务延时
* @param	prio    任务序号
*******************************************************************************/
int8_t  OSTimeDlyResume (int8_t prio)
{
    if(prio >= OS_MAX_TASKS)return OS_ERR_TASK_PRIO;
	OS_ENTER_CRITICAL();
	if (OSTCBTbl[prio].OSTCBDly != 0U)
	{
		OSTCBTbl[prio].OSTCBDly = 0U;
		OSRdyGrp |= (1 << prio);
		OS_EXIT_CRITICAL();
		OS_Sched();
	}
	else
	{
		OS_EXIT_CRITICAL();
	}
    return OS_ERR_NONE ;
}
/** ****************************************************************************
* @brief  进中断
*******************************************************************************/
void OSIntEnter(void)
{
    if (OSRunning == true) 
    {
        if (OSIntNesting < 255u) 
        {
            OSIntNesting++;                      //中断嵌套层数增加
        }
    }
}
/** ****************************************************************************
* @brief  出中断
*******************************************************************************/
void OSIntExit(void)
{
    if (OSRunning == true) 
    {
        OS_ENTER_CRITICAL();
        if (OSIntNesting > 0u) 
        {                           
            OSIntNesting--; //中断嵌套层数减少
        }
        OS_Sched();
        OS_EXIT_CRITICAL();
    }
}
/** ****************************************************************************
* @brief  获取系统时间
*******************************************************************************/
uint32_t OSTimeGet(void)
{
    uint32_t     ticks;
    OS_ENTER_CRITICAL();
    ticks = OSTime;
    OS_EXIT_CRITICAL();
    return (ticks);
}
/** ****************************************************************************
* @brief  设置系统时间
*******************************************************************************/
void OSTimeSet(uint32_t ticks)
{
    OS_ENTER_CRITICAL();
    OSTime = ticks;
    OS_EXIT_CRITICAL();
}
#if OS_TMR_CFG_MAX > 0
/** ****************************************************************************
* @brief  设置软定时器
* @param	ticks    计时节拍数
return  0 - 无定时器可用 n - 定时器序号
*******************************************************************************/
uint8_t OSTmrSet(uint32_t ticks)
{
    uint8_t i;
    OS_ENTER_CRITICAL();
    for(i=0;i<OS_TMR_CFG_MAX; i++)
    {
        if(OSTmrTbl[i].OSTmrUsed == false)
        {
            OSTmrTbl[i].OSTmrUsed = true;
            OSTmrTbl[i].OSTmrDly = ticks;
            OS_EXIT_CRITICAL();
            return i + 1;
        }
    }
    OS_EXIT_CRITICAL();
    return OS_ERR_NO_TMR;
}
/** ****************************************************************************
* @brief  获取软定时器值
* @param	idx    定时器序号
* @return 延时剩余时间,序号错误时错误时返回OS_ERR_TMR_IDX
*******************************************************************************/
uint32_t OSTmrGet(uint8_t idx)
{
    uint32_t dly;
    if(idx>OS_TMR_CFG_MAX||idx==OS_ERR_NO_TMR)return OS_ERR_TMR_IDX;
    OS_ENTER_CRITICAL();
    dly = OSTmrTbl[idx-1].OSTmrDly; //取走延时值
    if(OSTmrTbl[idx-1].OSTmrDly == 0)
    {
        OSTmrTbl[idx-1].OSTmrUsed = false; //释放定时器
    }
    OS_EXIT_CRITICAL();
    return dly;
}
#endif

#ifdef OS_USE_TQUE
/** ****************************************************************************
* @brief  向任务队列里添加任务
* @param	task    任务指针
* @param	no      任务队列序号
* @return 是否成功
*******************************************************************************/
bool TqueTaskAdd(TQUE_TaskNode *task)
{
    uint16_t i;
    if(task->TaskNo >TQUE_MAX_TASK_NO - 1) 
    {
        ULOG_E("Task No must be less than %d!",TQUE_MAX_TASK_NO-1);
        return false;
    }

    for(i=0;i<TQUE_MAX_TASK_NODE_NUM;i++)
    {
        if(taskList[i] == task)
        {
            ULOG_E("Task already exists!");
            return false;
        }
    }    

    for(i=0;i<TQUE_MAX_TASK_NODE_NUM;i++)
    {
        if(taskList[i] == 0)
        {
            taskList[i] = task;
            return true;
        }
    }

    ULOG_E("Task list full!");
    return false;   
}

/** ****************************************************************************
* @brief  从任务队列里删除任务
* @param	task    任务指针
* @param	no      任务队列序号
* @return 是否成功
*******************************************************************************/
bool TqueTaskDel(TQUE_TaskNode *task)
{
    uint16_t i;
    OS_ENTER_CRITICAL();
    for(i=0;i<TQUE_MAX_TASK_NODE_NUM;i++)
    {
        if(taskList[i] == task)
        {          
            taskList[i] = 0;
			OS_EXIT_CRITICAL();
            return true;  
        }
    }
    OS_EXIT_CRITICAL();
    ULOG_E("Task not found!");
    return false;   
}

/** ****************************************************************************
* @brief  执行任务队列中的任务
* @param	no      任务队列序号
*******************************************************************************/
void TqueTaskRun(uint8_t no)
{
    uint16_t i,temp;
   
    for(i=0;i<TQUE_MAX_TASK_NODE_NUM;i++)
    {
        if(taskList[i] != 0 && taskList[i]->TaskNo == no &&taskList[i]->active == true )
        { 
            if(taskList[i]->initOk == false && taskList[i]->initFunc != 0)
            {
                taskList[i]->initFunc(taskList[i]->initArg);
                taskList[i]->initOk = true;
            }

            if(taskList[i]->execFunc != 0)
            {               
                OS_ENTER_CRITICAL();
                temp = taskRunNo;
                taskRunNo = i;
                OS_EXIT_CRITICAL();
                taskList[i]->execFunc(taskList[i]->ExecArg);
                taskRunNo = temp;
            }
        }
    }
}

#endif
