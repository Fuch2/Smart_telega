#include "main.h"
#include "hw_gpio_if.h"

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} hw_gpio_map_entry_t;

static const hw_gpio_map_entry_t g_map[HW_GPIO_LOGICAL_COUNT] = {
    {SW_00_GPIO_Port, SW_00_Pin},  // 0
    {SW_01_GPIO_Port, SW_01_Pin},  // 1
    {SW_02_GPIO_Port, SW_02_Pin},  // 2
    {SW_03_GPIO_Port, SW_03_Pin},  // 3
    {SW_04_GPIO_Port, SW_04_Pin},  // 4
    {SW_05_GPIO_Port, SW_05_Pin},  // 5
    {SW_06_GPIO_Port, SW_06_Pin},  // 6
    {SW_07_GPIO_Port, SW_07_Pin},  // 7
    {SW_08_GPIO_Port, SW_08_Pin},  // 8
    {SW_09_GPIO_Port, SW_09_Pin},  // 9
    {SW_10_GPIO_Port, SW_10_Pin},  // 10
    {SW_11_GPIO_Port, SW_11_Pin},  // 11
    {SW_12_GPIO_Port, SW_12_Pin},  // 12
    {SW_13_GPIO_Port, SW_13_Pin},  // 13
    {SW_14_GPIO_Port, SW_14_Pin},  // 14
    {SW_15_GPIO_Port, SW_15_Pin},  // 15
    {SW_16_GPIO_Port, SW_16_Pin},  // 16
    {SW_17_GPIO_Port, SW_17_Pin},  // 17
    {SW_18_GPIO_Port, SW_18_Pin},  // 18
    {SW_19_GPIO_Port, SW_19_Pin},  // 19
    {SW_20_GPIO_Port, SW_20_Pin},  // 20
    {SW_21_GPIO_Port, SW_21_Pin},  // 21
    {SW_22_GPIO_Port, SW_22_Pin},  // 22
    {SW_23_GPIO_Port, SW_23_Pin},  // 23
};

bool hw_gpio_if_read_logical(uint8_t logical_index, bool *out_level_high)
{
    if ((logical_index >= HW_GPIO_LOGICAL_COUNT) || (out_level_high == NULL)) {
        return false;
    }
    *out_level_high = (HAL_GPIO_ReadPin(g_map[logical_index].port,
                                        g_map[logical_index].pin) == GPIO_PIN_SET);
    return true;
}

void hw_gpio_if_init(void)
{
    /* GPIO уже инициализированы CubeMX в MX_GPIO_Init — ничего делать не нужно */
}

