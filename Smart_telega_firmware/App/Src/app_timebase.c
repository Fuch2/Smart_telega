#include "app_timebase.h"
#include "stm32f1xx_hal.h"


void app_timebase_set_now_ms(uint32_t now_ms)
{
	(void)now_ms;
}

uint32_t app_timebase_now_ms(void)
{
	return HAL_GetTick();
}
