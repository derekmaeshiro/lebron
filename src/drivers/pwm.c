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

const joint_e pwm_joints[] = {
    WRIST_NAE_NAE, ELBOW, BICEP, SHOULDER_FRONT_RAISE, SHOULDER_LAT_RAISE,
};

int get_pwm_channel_index(joint_e joint)
{
    for (size_t i = 0; i < (int)NUM_PWM_CHANNELS; ++i) {
        if (pwm_joints[i] == joint)
            return i;
    }
    return -1; // Not a PWM-controlled joint
}

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

static struct pwm_channel_config pwm_channel_configs[] = {
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
        .tim = TIM8,
        .ccm = &TIM8->CCMR1,
        .cce = &TIM8->CCER,
        .ccr = &TIM8->CCR1,
        .egr = &TIM8->EGR,
        .ccer_mask = TIM_CCER_CC1E,
        .oc_shift = 4,
        .ocpe_mask = TIM_CCMR1_OC1PE,
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
};

// Removed strict io_config assertion due to mixed AFs
static bool initialized = false;

void pwm_init(void)
{
    ASSERT(!initialized);

    // Enable timer clocks
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;

    TIM2->PSC = (84000000UL / (PWM_PERIOD_TICKS * PWM_FREQ_HZ)) - 1;
    TIM3->PSC = (84000000UL / (PWM_PERIOD_TICKS * PWM_FREQ_HZ)) - 1;
    TIM8->PSC = (84000000UL / (PWM_PERIOD_TICKS * PWM_FREQ_HZ)) - 1;

    // Use standardized ARR
    TIM2->ARR = TIM3->ARR = TIM8->ARR = PWM_PERIOD_TICKS - 1;

    TIM2->CNT = TIM3->CNT = TIM8->CNT = 0;

    initialized = true;
}

void pwm_channel_enable(joint_e joint)
{
    int idx = get_pwm_channel_index(joint);
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
    if (tim == TIM1 || tim == TIM8) {
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

void pwm_set_duty_cycle(joint_e joint, uint8_t duty_cycle_percent)
{
    int idx = get_pwm_channel_index(joint);
    ASSERT(idx >= 0 && idx < (int)NUM_PWM_CHANNELS);
    struct pwm_channel_config *ch = &pwm_channel_configs[idx];

    if (duty_cycle_percent > 0) {
        uint32_t ccr_val = pwm_scale_duty_cycle(ch, duty_cycle_percent);
        *(ch->ccr) = ccr_val;
        pwm_channel_enable(joint);
    } else {
        *(ch->ccr) = 0;
    }
}