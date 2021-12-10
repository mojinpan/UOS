/** ****************************************************************************
* @file 	uos.h
* @author 	mojinpan
* @copyright (c) 2018 - 2020 mojinpan
* @brief 	微型嵌入式操作系统
* @version 	V0.9
* @date 	2021-02-08
* 
* @par 功能说明
* @details
* 1.提供至多8个任务的优先级的抢占式任务调度,实现OS的进程支持(基于UCOS极限精简)
* 2.提供任务队列的支持,便于快速动态加载任务
* 
*******************************************************************************/
#ifndef     __UOS_H__
#define     __UOS_H__

#include "stdint.h"
#include "stdbool.h"

#ifdef __cplusplus
extern  "C" {
#endif
/*******************************************************************************
							Macro Definition 宏定义
*******************************************************************************/
#define OS_MAX_TASKS                8          //最大任务数量[1 - 8]
#define OS_TMR_CFG_MAX              0          //可用软定时器数量
#define OS_INT_PRIO                 5          //OS中断优先级,高于此优先级的中断可强占OS

#define OS_USE_TQUE                            //开启任务队列的支持
#define TQUE_MAX_TASK_NO            8          //任务队列数量
#define TQUE_MAX_TASK_NODE_NUM      64         //任务节点数量
/*******************************************************************************
							Macro Definition 宏定义
*******************************************************************************/
#define OS_STAT_RDY                 0x00u      //任务就绪

#define OS_ERR_NONE                 0x00u      //无错误
#define OS_ERR_TASK_PRIO            0x01u      //任务序号错误
#define OS_ERR_NO_TMR               0x00u      //无可用定时器
#define OS_ERR_TMR_IDX              0xFFFFFFFFu//定时器序号错误

#define OS_ENTER_CRITICAL()     uint32_t cpu_sr;{cpu_sr = OS_CPU_SR_Save();}
#define OS_EXIT_CRITICAL()      {OS_CPU_SR_Restore(cpu_sr);}
           
/*******************************************************************************
						    Type declaration 类型声明
********************************************************************************/       
///任务控制块(Task Control Block)
typedef struct os_tcb{
    uint32_t          *OSTCBStkPtr;             ///< 任务栈顶指针
    uint32_t           OSTCBStkSize;            ///< 任务堆栈大小
    uint32_t           OSTCBDly;                ///< 任务延时
    uint8_t            OSTCBStat;               ///< 任务状态
    uint8_t            OSTCBPrio;               ///< 任务优先级
} OS_TCB;

typedef struct os_tmr{
    uint32_t           OSTmrDly;                ///< 延时时间,每个系统节拍后递减1
    bool               OSTmrUsed;               ///< 定时器已使用
} OS_TMR;

///任务节点
typedef struct tque_task_node {
    const char *name;                       	///< 任务名称
    uint8_t TaskNo;                             ///< 任务序号
    bool initOk;                                ///< 任务初始状态
    bool active;                                ///< 任务执行状态
    void (*initFunc)(void *);               	///< 初始化函数
    void *initArg;                              ///< 初始化参数
    void (*execFunc)(void *);                   ///< 执行函数
    void *ExecArg;                              ///< 执行参数
} TQUE_TaskNode;


extern volatile  unsigned int  OSTime;
/*******************************************************************************
							Function declaration 函数声明
*******************************************************************************/
uint32_t OS_CPU_SR_Save(void);
void OS_CPU_SR_Restore(uint32_t cpu_sr);
void OSInit(void);
void OS_Sched(void);
void  OSStart (void);
uint8_t OSTaskResume(uint8_t prio);
uint8_t OSTaskSuspend(uint8_t prio);
uint8_t OSTaskCreate(void (*task)(void *p_arg),void *p_arg,uint32_t *ptos,uint32_t stk_size,uint8_t prio);
void OSTimeTick(void);
void OSTimeDly(uint32_t ticks);
void OSIntEnter(void);
void OSIntExit(void);
uint32_t OSTimeGet(void);
void OSTimeSet(uint32_t ticks);

bool TqueTaskAdd(TQUE_TaskNode *task);
bool TqueTaskDel(TQUE_TaskNode *task);
void TqueTaskRun(uint8_t no);

#ifdef __cplusplus
}
#endif
  
#endif

