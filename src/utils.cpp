#include "utils.h"
#include <fstream>
#include <sstream>

extern CPUData ReadStatsCPU();
extern CPULoadData ReadAvgStatsCPU();

CPUData ReadStatsCPU()
{
	std::ifstream fileStat("/proc/stat");

	std::string line;

	const std::string STR_CPU("cpu");
	const std::size_t LEN_STR_CPU = STR_CPU.size();
	const std::string STR_TOT("tot");

	CPUData entry;

	while (std::getline(fileStat, line))
	{
		// cpu stats line found
		if (!line.compare(0, LEN_STR_CPU, STR_CPU))
		{
			std::istringstream ss(line);

			// read cpu label
			ss >> entry.cpu;

			if (entry.cpu.size() > LEN_STR_CPU)
				entry.cpu.erase(0, LEN_STR_CPU);
			else
				entry.cpu = STR_TOT;

			// read times
			for (int i = 0; i < NUM_CPU_STATES; ++i)
				ss >> entry.times[i];
		}
	}

	return entry;
}

CPULoadData ReadAvgStatsCPU()
{
	std::ifstream fileLoadAvgStat("/proc/loadavg");

	std::string line;

	CPULoadData entry;

	std::getline(fileLoadAvgStat, line);
	std::istringstream iss(line);

	// read times
	for (int i = 0; i < NUM_CPU_LOAD_STATES; ++i)
		iss >> entry.times[i];

	return entry;
}



size_t GetCPUIdle()
{
	const CPUData entry = ReadStatsCPU();
	return  entry.times[S_IDLE] + entry.times[S_IOWAIT];
}

size_t GetCPUUsage()
{
	const CPUData entry = ReadStatsCPU();
	return	entry.times[S_USER] +
		entry.times[S_NICE] +
		entry.times[S_SYSTEM] +
		entry.times[S_IRQ] +
		entry.times[S_SOFTIRQ] +
		entry.times[S_STEAL] +
		entry.times[S_GUEST] +
		entry.times[S_GUEST_NICE];
}

float GetCPUUsagePercents()
{
	const size_t idle = GetCPUIdle();
	const size_t active = GetCPUUsage();
	const size_t total = idle + active;
	return 100.f * active / total;
}


size_t GetCPUAverageLoad()
{
	const CPULoadData entry = ReadAvgStatsCPU();
	return  entry.times[S_AVG_1_MIN] * 100 / 4;
}
