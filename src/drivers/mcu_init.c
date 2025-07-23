#include "mcu_init.h"
#include "io.h"
#include <stm32f4xx.h>

void clock_init(void)
{
    // 1. Enable Power controller (needed for voltage scaling)
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS; // VOS = Scale 1, good for high speed

    // 2. Enable HSE and wait for it
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY))
        ; // Wait until HSE ready

    // 3. Set PLL: PLLM=8, PLLN=336, PLLP=2, PLLQ=7
    // PLL input  = HSE / PLLM = 8/8 = 1 MHz
    // VCO output = 1 MHz * PLLN = 336 MHz
    // PLL output = VCO / PLLP = 336/2 = 168 MHz
    RCC->PLLCFGR = (8 << RCC_PLLCFGR_PLLM_Pos) | // PLLM = 8
        (336 << RCC_PLLCFGR_PLLN_Pos) | // PLLN = 336
        (0 << RCC_PLLCFGR_PLLP_Pos) | // PLLP = 2
        RCC_PLLCFGR_PLLSRC_HSE | // Source = HSE
        (7 << RCC_PLLCFGR_PLLQ_Pos); // PLLQ = 7 (for USB, safe default)

    // 4. Enable PLL and wait for it
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY))
        ; // Wait until PLL ready

    // 5. Set FLASH latency/wait states (5WS for 168 MHz), enable cache/prefetch
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;

    // 6. Set AHB/APB bus dividers (AHB=168, APB1=42, APB2=84)
    RCC->CFGR = RCC_CFGR_HPRE_DIV1 | // AHB prescaler = 1 (168 MHz)
        RCC_CFGR_PPRE1_DIV4 | // APB1 prescaler = 4 (42 MHz)
        RCC_CFGR_PPRE2_DIV2; // APB2 prescaler = 2 (84 MHz)

    // 7. Select PLL as SYSCLK
    RCC->CFGR &= ~RCC_CFGR_SW; // Clear SW bits
    RCC->CFGR |= RCC_CFGR_SW_PLL; // Set SW to PLL

    // 8. Wait for switch to take effect
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
        ;

    // System now running at 168 MHz (CPU & AHB)
    // APB1 peripherals at 42 MHz, APB2 at 84 MHz
}

void mcu_init(void)
{
    // watchdog is already off by default
    io_init();
    clock_init();

    // Globally enable interrupts
    __enable_irq();
}