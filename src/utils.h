#pragma once
#ifndef __UTILS_H
#define __UTILS_H

#include <string>

#define NUM_CPU_STATES 10
#define NUM_CPU_LOAD_STATES 3

enum CPUStates
{
	S_USER = 0,
	S_NICE,
	S_SYSTEM,
	S_IDLE,
	S_IOWAIT,
	S_IRQ,
	S_SOFTIRQ,
	S_STEAL,
	S_GUEST,
	S_GUEST_NICE
};

typedef struct CPUData
{
	std::string cpu;
	size_t times[NUM_CPU_STATES];
} CPUData;

enum CPULoadStates
{
	S_AVG_1_MIN = 0,
	S_AVG_5_MIN,
	S_AVG_15_MIN
};

typedef struct CPULoadData
{
	float times[NUM_CPU_LOAD_STATES];
} CPULoadData;

extern size_t GetCPUIdle();
extern size_t GetCPUUsage();
extern float GetCPUUsagePercents();
extern size_t GetCPUAverageLoad();

#endif /* __UTILS_H */
