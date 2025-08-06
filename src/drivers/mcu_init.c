#include "mcu_init.h"
#include "io.h"
#include <stm32f4xx.h>

void clock_init(void)
{
    // 1. Enable Power controller (needed for voltage scaling)
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS; // Scale 1, good for high speed

#ifdef ARM_SLEEVE
    // === STM32F411 Black Pill ===
    // HSE: 25MHz; SYSCLK: 100MHz
    // PLL: (25/25)*200/2 = 100 MHz
    const uint32_t pll_m = 25;
    const uint32_t pll_n = 200;
    const uint32_t pll_p = 0; // 00b => PLLP=2
    const uint32_t pll_q = 4;
    const uint32_t flash_ws = FLASH_ACR_LATENCY_3WS;

    // Enable HSE (external 25 MHz crystal)
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY))
        ;

    RCC->PLLCFGR = (pll_m << RCC_PLLCFGR_PLLM_Pos) | (pll_n << RCC_PLLCFGR_PLLN_Pos)
        | (pll_p << RCC_PLLCFGR_PLLP_Pos) | RCC_PLLCFGR_PLLSRC_HSE
        | (pll_q << RCC_PLLCFGR_PLLQ_Pos);

    // Enable PLL, wait ready
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY))
        ;

    // Flash wait states, enable caches & prefetch
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | flash_ws;

    // AHB=100MHz, APB1=50MHz, APB2=100MHz
    RCC->CFGR = RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1;

#elif defined(ROBOTIC_ARM)
    // === STM32F446RE Nucleo or similar ===
    // HSE: 8MHz; SYSCLK: 168 MHz
    // PLL: (8/8)*336/2 = 168 MHz
    const uint32_t pll_m = 8;
    const uint32_t pll_n = 336;
    const uint32_t pll_p = 0; // 00b => PLLP=2
    const uint32_t pll_q = 7;
    const uint32_t flash_ws = FLASH_ACR_LATENCY_5WS;

    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY))
        ;

    RCC->PLLCFGR = (pll_m << RCC_PLLCFGR_PLLM_Pos) | (pll_n << RCC_PLLCFGR_PLLN_Pos)
        | (pll_p << RCC_PLLCFGR_PLLP_Pos) | RCC_PLLCFGR_PLLSRC_HSE
        | (pll_q << RCC_PLLCFGR_PLLQ_Pos);

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY))
        ;

    // Flash wait states, enable caches & prefetch
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | flash_ws;

    // AHB=168, APB1=42, APB2=84
    RCC->CFGR = RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;
#else
#error "You must #define ARM_SLEEVE or ROBOTIC_ARM"
#endif

    // 7. Select PLL as SYSCLK, wait for switch
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
        ;
}

void mcu_init(void)
{
    // watchdog is already off by default
    io_init();
    clock_init();

    // Globally enable interrupts
    __enable_irq();
}