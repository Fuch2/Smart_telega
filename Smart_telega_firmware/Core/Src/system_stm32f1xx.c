#include "stm32f1xx.h"

uint32_t SystemCoreClock = 8000000U;

const uint8_t AHBPrescTable[16U] = {
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    1U, 2U, 3U, 4U, 6U, 7U, 8U, 9U
};

const uint8_t APBPrescTable[8U] = {
    0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U
};

void SystemInit(void)
{
    RCC->CR |= 0x00000001U;
    RCC->CFGR &= 0xF8FF0000U;
    RCC->CR &= 0xFEF6FFFFU;
    RCC->CR &= 0xFFFBFFFFU;
    RCC->CFGR &= 0xFF80FFFFU;
    RCC->CIR = 0x009F0000U;
#ifdef VECT_TAB_SRAM
    SCB->VTOR = SRAM_BASE | 0x00000000U;
#else
    SCB->VTOR = FLASH_BASE | 0x00000000U;
#endif
}

void SystemCoreClockUpdate(void)
{
    uint32_t tmp, pllmull, pllsource;

    tmp = RCC->CFGR & RCC_CFGR_SWS;

    switch (tmp)
    {
        case 0x00U:
            SystemCoreClock = 8000000U;
            break;
        case 0x04U:
            SystemCoreClock = 8000000U;
            break;
        case 0x08U:
            pllmull   = (RCC->CFGR & RCC_CFGR_PLLMULL) >> 18U;
            pllsource =  RCC->CFGR & RCC_CFGR_PLLSRC;
            pllmull   = pllmull + 2U;
            if (pllsource == 0x00U)
                SystemCoreClock = (8000000U >> 1U) * pllmull;
            else
                SystemCoreClock = 8000000U * pllmull;
            break;
        default:
            SystemCoreClock = 8000000U;
            break;
    }

    tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4U)];
    SystemCoreClock >>= tmp;
}
