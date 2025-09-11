#include "pwm.h"
#include "io.h"
#include "../common/assert_handler.h"
#include "../common/defines.h"
#include "../common/trace.h"

#include <stm32f4xx.h>
#include <stdbool.h>
#include <stddef.h>

#define PWM_FREQ_HZ 1000
#define PWM_RESOLUTION 18464
#define PWM_PERIOD_TICKS PWM_RESOLUTION

struct pwm_channel_config
{
    TIM_TypeDef *tim;
    volatile uint32_t *ccm; // Pointer to CCMR1 or CCMR2
    volatile uint32_t *cce; // Pointer to CCER
    volatile uint32_t *ccr; // Pointer to CCRx
    volatile uint32_t *egr; // Pointer to EGR
    uint32_t oc_shift; // Bit position of OCxM in CCMRx
    uint32_t ocpe_mask; // OCxPE bit mask
    uint32_t ccer_mask;
};

static struct pwm_channel_config pwm_channel_configs[NUM_PWM_CHANNELS] = {
#if defined(ROBOTIC_ARM)
    // PWM_L_WRIST_NAE_NAE: PA6 TIM3_CH1 AF2
    {
        .tim = TIM3,
        .ccr = &TIM3->CCR1,
        .ccm = &TIM3->CCMR1,
        .cce = &TIM3->CCER,
        .egr = &TIM3->EGR,
        .oc_shift = 4,
        .ocpe_mask = TIM_CCMR1_OC1PE,
        .ccer_mask = TIM_CCER_CC1E,
    },
    // PWM_R_WRIST_NAE_NAE: PB7 TIM4_CH2 AF2
    {
        .tim = TIM4,
        .ccr = &TIM4->CCR2,
        .ccm = &TIM4->CCMR1,
        .cce = &TIM4->CCER,
        .egr = &TIM4->EGR,
        .oc_shift = TIM_CCMR1_OC2M_Pos,
        .ocpe_mask = TIM_CCMR1_OC2PE,
        .ccer_mask = TIM_CCER_CC2E,
    },
    // PWM_L_ELBOW: PC7 TIM3_CH2 AF2
    {
        .tim = TIM3,
        .ccr = &TIM3->CCR2,
        .ccm = &TIM3->CCMR1,
        .cce = &TIM3->CCER,
        .egr = &TIM3->EGR,
        .oc_shift = TIM_CCMR1_OC2M_Pos,
        .ocpe_mask = TIM_CCMR1_OC2PE,
        .ccer_mask = TIM_CCER_CC2E,
    },
    // PWM_R_ELBOW: PC6 TIM8_CH1 AF3
    {
        .tim = TIM8,
        .ccr = &TIM8->CCR1,
        .ccm = &TIM8->CCMR1,
        .cce = &TIM8->CCER,
        .egr = &TIM8->EGR,
        .oc_shift = 4,
        .ocpe_mask = TIM_CCMR1_OC1PE,
        .ccer_mask = TIM_CCER_CC1E,
    },
    // PWM_L_BICEP: PB6 TIM4_CH1 AF2
    {
        .tim = TIM4,
        .ccr = &TIM4->CCR1,
        .ccm = &TIM4->CCMR1,
        .cce = &TIM4->CCER,
        .egr = &TIM4->EGR,
        .oc_shift = 4,
        .ocpe_mask = TIM_CCMR1_OC1PE,
        .ccer_mask = TIM_CCER_CC1E,
    },
    // PWM_R_BICEP: PA2 TIM2_CH2 AF1
    {
        .tim = TIM2,
        .ccr = &TIM2->CCR2,
        .ccm = &TIM2->CCMR1,
        .cce = &TIM2->CCER,
        .egr = &TIM2->EGR,
        .oc_shift = TIM_CCMR1_OC2M_Pos,
        .ocpe_mask = TIM_CCMR1_OC2PE,
        .ccer_mask = TIM_CCER_CC2E,
    },
    // PWM_L_SHOULDER_FRONT_RAISE: PB10 TIM2_CH3 AF1
    {
        .tim = TIM2,
        .ccr = &TIM2->CCR3,
        .ccm = &TIM2->CCMR2,
        .cce = &TIM2->CCER,
        .egr = &TIM2->EGR,
        .oc_shift = 4,
        .ocpe_mask = TIM_CCMR2_OC3PE,
        .ccer_mask = TIM_CCER_CC3E,
    },
    // PWM_R_SHOULDER_FRONT_RAISE: PB1 TIM3_CH4 AF2
    {
        .tim = TIM3,
        .ccr = &TIM3->CCR4,
        .ccm = &TIM3->CCMR2,
        .cce = &TIM3->CCER,
        .egr = &TIM3->EGR,
        .oc_shift = TIM_CCMR2_OC4M_Pos,
        .ocpe_mask = TIM_CCMR2_OC4PE,
        .ccer_mask = TIM_CCER_CC4E,
    },
    // PWM_L_SHOULDER_LAT_RAISE: PA3 TIM2_CH4 AF1
    {
        .tim = TIM2,
        .ccr = &TIM2->CCR4,
        .ccm = &TIM2->CCMR2,
        .cce = &TIM2->CCER,
        .egr = &TIM2->EGR,
        .oc_shift = TIM_CCMR2_OC4M_Pos,
        .ocpe_mask = TIM_CCMR2_OC4PE,
        .ccer_mask = TIM_CCER_CC4E,
    },
    // PWM_R_SHOULDER_LAT_RAISE: PA7 TIM8_CH2 AF3
    {
        .tim = TIM8,
        .ccr = &TIM8->CCR2,
        .ccm = &TIM8->CCMR1,
        .cce = &TIM8->CCER,
        .egr = &TIM8->EGR,
        .oc_shift = TIM_CCMR1_OC2M_Pos,
        .ocpe_mask = TIM_CCMR1_OC2PE,
        .ccer_mask = TIM_CCER_CC2E,
    },
#endif

#if defined(ARM_SLEEVE)
    {
        .tim = TIM3,
        .ccr = &TIM3->CCR1,
        .ccm = &TIM3->CCMR1,
        .cce = &TIM3->CCER,
        .egr = &TIM3->EGR,
        .oc_shift = 4,
        .ocpe_mask = TIM_CCMR1_OC1PE,
        .ccer_mask = TIM_CCER_CC1E,
    },

    {
        .tim = TIM3,
        .ccr = &TIM3->CCR2,
        .ccm = &TIM3->CCMR1,
        .cce = &TIM3->CCER,
        .egr = &TIM3->EGR,
        .oc_shift = TIM_CCMR1_OC2M_Pos,
        .ocpe_mask = TIM_CCMR1_OC2PE,
        .ccer_mask = TIM_CCER_CC2E,
    },

    {
        .tim = TIM2,
        .ccm = &TIM2->CCMR2,
        .cce = &TIM2->CCER,
        .ccr = &TIM2->CCR3,
        .egr = &TIM2->EGR,
        .ccer_mask = TIM_CCER_CC3E,
        .oc_shift = 4,
        .ocpe_mask = TIM_CCMR2_OC3PE,
    },

    {
        .tim = TIM4,
        .ccm = &TIM4->CCMR1,
        .cce = &TIM4->CCER,
        .ccr = &TIM4->CCR1,
        .egr = &TIM4->EGR,
        .ccer_mask = TIM_CCER_CC1E,
        .oc_shift = 4,
        .ocpe_mask = TIM_CCMR1_OC1PE,
    },
#endif
};

// Removed strict io_config assertion due to mixed AFs
static bool initialized = false;

void pwm_init(void)
{
    ASSERT(!initialized);

    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;
#if defined(ROBOTIC_ARM)
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
#endif

    // APB1 = 42MHz (timers = 84MHz), APB2 = 84MHz (timers = 168MHz)
    uint32_t tim_apb1_clk = 84000000UL; // TIM2,3,4
    uint32_t psc_val_apb1 = (tim_apb1_clk / (PWM_PERIOD_TICKS * PWM_FREQ_HZ)) - 1;
#if defined(ROBOTIC_ARM)
    uint32_t tim_apb2_clk = 168000000UL; // TIM8
    uint32_t psc_val_apb2 = (tim_apb2_clk / (PWM_PERIOD_TICKS * PWM_FREQ_HZ)) - 1;
#endif
    uint32_t arr_val = PWM_PERIOD_TICKS - 1;

    TIM2->PSC = psc_val_apb1;
    TIM3->PSC = psc_val_apb1;
    TIM4->PSC = psc_val_apb1;
#if defined(ROBOTIC_ARM)
    TIM8->PSC = psc_val_apb2;
#endif

    TIM2->ARR = arr_val;
    TIM3->ARR = arr_val;
    TIM4->ARR = arr_val;
#if defined(ROBOTIC_ARM)
    TIM8->ARR = arr_val;
#endif

    TIM2->CNT = TIM3->CNT = TIM4->CNT = 0;
#if defined(ROBOTIC_ARM)
    TIM8->CNT = 0;
#endif

    initialized = true;
}

void pwm_channel_enable(pwm_e pwm_channel)
{
    int idx = pwm_channel;
    ASSERT(idx >= 0 && idx < (int)NUM_PWM_CHANNELS);
    struct pwm_channel_config *ch = &pwm_channel_configs[idx];

    TIM_TypeDef *tim = ch->tim;

    // Disable output before changing config
    *(ch->cce) &= ~(ch->ccer_mask);

    // Enable auto-reload preload and main counter
    tim->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;

    // Configure output compare mode
    *(ch->ccm) &= ~(0b111 << ch->oc_shift); // Clear OCxM bits
    *(ch->ccm) |= (6 << ch->oc_shift) | ch->ocpe_mask; // Set PWM mode 1 + preload

    // Generate update event to latch values
    *(ch->egr) |= TIM_EGR_UG;

    // Enable channel output
    *(ch->cce) |= ch->ccer_mask;

    // For advanced-control timers like TIM8, enable main output
    if (tim == TIM1
#if defined(ROBOTIC_ARM)
        || tim == TIM8
#endif
    ) {
        tim->BDTR |= TIM_BDTR_MOE;
    }
}

static inline uint32_t pwm_scale_duty_cycle(struct pwm_channel_config *ch,
                                            uint8_t duty_cycle_percent)
{
    ASSERT(duty_cycle_percent <= 100);
    uint32_t period = ch->tim->ARR + 1;
    return (duty_cycle_percent * period) / 100;
}

void pwm_set_duty_cycle(pwm_e pwm_channel, uint8_t duty_cycle_percent)
{
    int idx = pwm_channel;
    ASSERT(idx >= 0 && idx < (int)NUM_PWM_CHANNELS);
    struct pwm_channel_config *ch = &pwm_channel_configs[idx];

    if (duty_cycle_percent > 0) {
        uint32_t ccr_val = pwm_scale_duty_cycle(ch, duty_cycle_percent);
        *(ch->ccr) = ccr_val;
        pwm_channel_enable(pwm_channel);
    } else {
        *(ch->ccr) = 0;
    }
}