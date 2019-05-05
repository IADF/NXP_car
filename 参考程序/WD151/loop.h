#ifndef _LOOP_H_
#define _LOOP_H_

#include "common.h"
extern volatile uint32_t sysTickUptime;
typedef struct
{
	void(*task_func)(void);
	uint16_t interval_ticks;//ִ��ʱ����
	uint32_t last_run;//���һ������ʱ��
}sched_task_t;

extern void Loop_Run(void);
void Scheduler_Setup(void);
#endif

