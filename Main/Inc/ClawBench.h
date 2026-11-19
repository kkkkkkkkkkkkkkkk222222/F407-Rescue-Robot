#ifndef CLAW_BENCH_H
#define CLAW_BENCH_H

#include <stdbool.h>
#include <stdint.h>

/* Manual calibration console. An unrecognised command returns NULL. */
const char *ClawBench_Command(const char *command, bool available);
bool ClawBench_IsRunning(void);
void ClawBench_Stop(void);
void ClawBench_Process(uint32_t now_ms);

#endif
