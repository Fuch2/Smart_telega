#include "hw_gpio_if.h"

#include <string.h>

/* ============================================================
 * Compile-time mapping skeleton (logical channel -> hw line id)
 * При реальной интеграции сюда подставляются конкретные GPIO порты/пины.
 * ============================================================ */
typedef struct
{
    uint8_t hw_line_id;
} hw_gpio_map_entry_t;

static const hw_gpio_map_entry_t g_map[HW_GPIO_LOGICAL_COUNT] = {
    {0u},  {1u},  {2u},  {3u},  {4u},  {5u},
    {6u},  {7u},  {8u},  {9u},  {10u}, {11u},
    {12u}, {13u}, {14u}, {15u}, {16u}, {17u},
    {18u}, {19u}, {20u}, {21u}, {22u}, {23u}
};

/* ============================================================
 * Compileable skeleton:
 * пока без прямых HAL вызовов — храним уровни в статическом mock-массиве.
 * Это позволяет тестировать этап 3 на host без железа.
 * ============================================================ */
static bool g_levels[HW_GPIO_LOGICAL_COUNT];

void hw_gpio_if_init(void)
{
    (void)memset(g_levels, 0, sizeof(g_levels));
}

bool hw_gpio_if_read_logical(uint8_t logical_index, bool *out_level_high)
{
    uint8_t hw_line_id;

    if ((logical_index >= HW_GPIO_LOGICAL_COUNT) || (out_level_high == 0)) {
        return false;
    }

    hw_line_id = g_map[logical_index].hw_line_id;
    if (hw_line_id >= HW_GPIO_LOGICAL_COUNT) {
        return false;
    }

    *out_level_high = g_levels[hw_line_id];
    return true;
}

bool hw_gpio_if_debug_set_level(uint8_t logical_index, bool level_high)
{
    uint8_t hw_line_id;

    if (logical_index >= HW_GPIO_LOGICAL_COUNT) {
        return false;
    }

    hw_line_id = g_map[logical_index].hw_line_id;
    if (hw_line_id >= HW_GPIO_LOGICAL_COUNT) {
        return false;
    }

    g_levels[hw_line_id] = level_high;
    return true;
}
