#ifndef ROUTE_DEMO_H
#define ROUTE_DEMO_H

#include <stdbool.h>
#include <stdint.h>

void RouteDemo_Init(void);
void RouteDemo_Process(uint32_t now_ms);
bool RouteDemo_IsRunning(void);

#endif
