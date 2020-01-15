# UOS: 基于UCOS的精简OS

![version](https://img.shields.io/badge/version-0.6-brightgreen.svg)
![build](https://img.shields.io/badge/build-2019.10.22-brightgreen.svg)
![build](https://img.shields.io/badge/license-MIT-brightgreen.svg)

## 1. 介绍

这是一个基于UCOS的精简OS,极致的精简,只保留了任务调度等非常核心的功能,适用于任务相对固定且对资源要求非常苛刻的工控环境.

## 2. 功能说明

- 提供至多8个任务的优先级的抢占式任务调度,实现OS的进程支持
- 提供任务队列的支持,便于快速动态加载任务
- 提供软件定时器和系统时间输出支持
- 汇编部分嵌入到C中,接口尽可能精简,使移植的复杂度降低到最低
- 支持Cortex-M3/Cortex-M4/Cortex-M7,目前已STM32F103/SMT32F767上测试过(基于keil)


## 3. 移植说明

- Cortex-M3/Cortex-M4/Cortex-M7环境下无需移植,直接使用(基于keil)

## 4. 使用说明

### 4.1 宏配置

- 按需求配置宏

    | 宏                            | 意义                        |
    | --------------------------    | --------------------------- |
    | OS_MAX_TASKS                  | 最大任务数量[1 - 8]          |
    | OS_TMR_CFG_MAX                | 定时器数量[0 - 255]          |
    | OS_USE_TQUE                   | 开启任务队列的支持            |
    | TQUE_MAX_TASK_NO              | 任务队列数量                 |
    | TQUE_MAX_TASK_NODE_NUM        | 任务节点数量                 |

### 4.2 OS部分

- 先定义任务栈

```
//任务栈大小定义
#define LV1_STK_SIZE                (1024U)                
#define LV2_STK_SIZE                (1024U)   
#define IDLE_STK_SIZE               (1024U) 
//任务优先级定义
#define LV1_TASK_PRIO               (0U)
#define LV2_TASK_PRIO               (1U)
#define IDLE_TASK_PRIO              (OS_MAX_TASKS - 1)
//任务队列定义
#define LEVEL1  1U
#define LEVEL2  2U
//任务栈
uint32_t Lv1TaskStk[LV1_STK_SIZE];
uint32_t Lv2TaskStk[LV2_STK_SIZE];
uint32_t IdleTaskStk[IDLE_STK_SIZE];
```

- 定义任务函数

```
/** ****************************************************************************
* @brief  LV1进程,执行周期1ms
*******************************************************************************/
void Lv1Task(void)
{
    while(1)
    {
		TqueTaskRun(LEVEL1);
		OSTimeDly(1);
    }
}

/** ****************************************************************************
* @brief  LV2进程,执行周期10ms
*******************************************************************************/
void Lv2Task(void)
{
    while(1)
    {
		TqueTaskRun(LEVEL2);
        OSTimeDly(10);
    }
}
/** ****************************************************************************
* @brief 空闲进程
*******************************************************************************/
void IdleTask(void)
{
	while(1)
    {

    }
}
```

- 在main函数中加初始化os并创建任务

```
int main(void)
{

	BspInit();
	SysInfo();
	BasInit();
	AppInit();
	
    OSInit();
    OSTaskCreate((void(*)(void *))Lv1Task,(void *)0, &Lv1TaskStk[LV1_STK_SIZE - 1], LV1_STK_SIZE, LV1_TASK_PRIO);
    OSTaskCreate((void(*)(void *))Lv2Task,(void *)0, &Lv2TaskStk[LV2_STK_SIZE - 1], LV2_STK_SIZE, LV2_TASK_PRIO);
    OSTaskCreate((void(*)(void *))IdleTask,(void *)0, &IdleTaskStk[IDLE_STK_SIZE - 1], IDLE_STK_SIZE, IDLE_TASK_PRIO);
	OSStart();


  while (1)
  {

  }

}
```

- 在SysTick_Handler中添加任务调度函数

```
void SysTick_Handler(void)
{   
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();

    OSIntExit();
    OSTimeTick();
    OSIntExit();

}
```
### 4.2 任务队列的使用

- 定义任务节点信息

```
 TQUE_TaskNode tcpServer = {
 	.name       = "TcpServer",                      //任务名称
 	.TaskNo     = LEVEL2,                           //任务队列序号,通过任务队列序号指定执行任务的队列                              
    .active     = true,                             //是否处于活跃状态,非活跃状态任务无法执行
    .initFunc   = (void (*)(void *))TcpServerInit,  //指定该任务的初始化函数
    .initArg    = (void*)&tcp,                      //指定该任务的初始化参数
 	.execFunc   = (void (*)(void *))TcpServer,      //指定该任务的执行函数
    .ExecArg    = (void*)&tcp,                      //指定该任务的执行参数
 };
 ```

- 在对应的进程中加载任务任务队列函数

```
/** ****************************************************************************
* @brief  LV2进程,执行周期10ms
*******************************************************************************/
void Lv2Task(void)
{
    while(1)
    {
		TqueTaskRun(LEVEL2);
        OSTimeDly(10);
    }
}
```

- 在运行的代码中调用任务添加函数添加任务

```
.....

    TqueTaskAdd(&tcpServer);
.....
```

## 5. 更新说明

- V0.1 2019-05-18
  
  - 1.开始对UCOSII进行精简
  - 2.将汇编整合到C文件中。

- V0.2 2019-05-26 
  - 1.实现最基本的任务调度

- V0.3 2019-05-27  
  - 1.时钟节拍,实现OS级的延时支持

- V0.4 2019-05-28
  - 1.增加中断任务切换支持

- V0.5 2019-06-09
  - 1.增加系统时间的支持
  - 2.增加软定时器的支持

- V0.6 2019-10-22
  - 1.重写任务队列部分代码,支持动态加载,并简化操作
