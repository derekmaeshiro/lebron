#include <stm32f4xx.h>
#include "pwm.h"
#include "io.h"
#include <stdbool.h>
#include "../common/assert_handler.h"
#include "../common/defines.h"

/*
    The following macros are used to calculate the # of ticks in a defined period.
    TIMER_TICK_FREQ calculates the effective frequency of the timer given the values in defines.h.
    PWM_PERIOD_FREQ_HZ defines the length of a period (the length of the PWM cycle), 
    and PWM_PERIOD_TICKS calculates the # of timer ticks in the defined period.
    For convenience, we want PWM_PERIOD_TICKS to be equivalent to 100.
*/

#define PWM_PERIOD_FREQ_HZ (20000UL)
#define PWM_PERIOD_TICKS (TIMER_TICK_FREQ / PWM_PERIOD_FREQ_HZ)

// ASSERT(PWM_PERIOD_TICKS == 100, "Expected 100 ticks per period");

struct pwm_channel_config{
    bool enabled;
    volatile uint32_t *const ccm;
    volatile uint32_t *const cce;
    volatile uint32_t *const ccr;
};

#if defined (ROBOTIC_ARM)
static struct pwm_channel_config pwm_channel_configs[] = {
    [PWM_DISTAL_INTERPHALANGEAL_JOINT] = { 
        .enabled = false,
        .ccm = &TIM2->CCMR1,
        .cce = &TIM2->CCER,
        .ccr = &TIM2->CCR1
    },
    [PWM_PROXIMAL_INTERPHALANGEAL_JOINT] = { 
        .enabled = false,
        .ccm = &TIM2->CCMR1,
        .cce = &TIM2->CCER,
        .ccr = &TIM2->CCR2
    },
    [PWM_METACARPOPHALANGEAL_JOINT_1] = { 
        .enabled = false,
        .ccm = &TIM2->CCMR2,
        .cce = &TIM2->CCER,
        .ccr = &TIM2->CCR3
    },
    [PWM_METACARPOPHALANGEAL_JOINT_2] = { 
        .enabled = false,
        .ccm = &TIM2->CCMR2,
        .cce = &TIM2->CCER,
        .ccr = &TIM2->CCR4
    }
};
#endif

static const struct io_config pwm_io_config = {
    .used = true,
    .select = IO_SELECT_ALT,
    .io_alt_function = IO_ALT_FUNCTION_2,
    .resistor = IO_RESISTOR_DISABLED,
    .out = IO_OUT_LOW,
};

static bool initialized = false;

void pwm_init(void)
{
    ASSERT(!initialized);

    // Assert that PWM pins for ROBOTIC ARM are configured properly
    struct io_config current_config;
    #if defined(ROBOTIC_ARM)
        io_get_current_config(IO_PWM_DISTAL_INTERPHALANGEAL_JOINT, &current_config);
        ASSERT(io_config_compare(&current_config, &pwm_io_config));
        io_get_current_config(IO_PWM_PROXIMAL_INTERPHALANGEAL_JOINT, &current_config);
        ASSERT(io_config_compare(&current_config, &pwm_io_config));
        io_get_current_config(IO_PWM_METACARPOPHALANGEAL_JOINT_1, &current_config);
        ASSERT(io_config_compare(&current_config, &pwm_io_config));
        io_get_current_config(IO_PWM_METACARPOPHALANGEAL_JOINT_2, &current_config);
        ASSERT(io_config_compare(&current_config, &pwm_io_config));
    #endif 

    // Initialize base timer
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2 -> PSC = TIMER_PRESCALER;
    TIM2 -> CNT = 0;
    TIM2->ARR = PWM_PERIOD_TICKS - 1;

    initialized = true;
}

static bool pwm_all_channels_disabled(){
    for(uint8_t i=0; i < ARRAY_SIZE(pwm_channel_configs); i++){
        if(pwm_channel_configs[i].enabled){
            return false;
        }
    }
    return true;
}

static bool pwm_enabled = false;
static void pwm_enable(bool enable)
{
    if (pwm_enabled != enable) {
        if (enable) {
            // Generate an update event to load registers
            TIM2->EGR |= TIM_EGR_UG;

            // Enable auto-reload preload
            TIM2->CR1 |= TIM_CR1_ARPE;

            // Enable counter
            TIM2->CR1 |= TIM_CR1_CEN;
        } else {
            // Disable counter
            TIM2->CR1 &= ~TIM_CR1_CEN;
        }

        pwm_enabled = enable;
    }
}

void pwm_channel_enable(pwm_e pwm, bool enable) {
    struct pwm_channel_config *ch = &pwm_channel_configs[pwm];
    if(pwm_channel_configs[pwm].enabled != enable) {
        uint32_t ccmr_mask = 0;
        uint32_t ccer_mask = 0;

        switch (pwm) {
            case PWM_DISTAL_INTERPHALANGEAL_JOINT:  // CH1
                ccmr_mask = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;
                ccer_mask = TIM_CCER_CC1E;
                break;
            case PWM_PROXIMAL_INTERPHALANGEAL_JOINT:  // CH2
                ccmr_mask = TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE;
                ccer_mask = TIM_CCER_CC2E;
                break;
            case PWM_METACARPOPHALANGEAL_JOINT_1:  // CH3
                ccmr_mask = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE;
                ccer_mask = TIM_CCER_CC3E;
                break;
            case PWM_METACARPOPHALANGEAL_JOINT_2:  // CH4
                ccmr_mask = TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE;
                ccer_mask = TIM_CCER_CC4E;
                break;
            default:
                ASSERT(false);  // Invalid channel
                return;
        }

        // Disable channel before modifying registers
        *(ch->cce) &= ~ccer_mask;

        if (enable) {
            *(ch->ccm) |= ccmr_mask;   // Set PWM mode and enable preload
            *(ch->cce) |= ccer_mask;   // Enable output
        } else {
            *(ch->cce) &= ~ccer_mask;  // Disable output
        }

        ch->enabled = enable;

        if (enable) {
            pwm_enable(true);  // start timer if needed
        } else if (pwm_all_channels_disabled()) {
            pwm_enable(false);  // stop timer if no channels active
        }
    }
}

static inline uint8_t pwm_scale_duty_cycle(uint8_t duty_cycle_percent){
    // Scale up/down duty cycle depending on max voltage of battery and servos
   return duty_cycle_percent;
}

void pwm_set_duty_cycle(pwm_e pwm, uint8_t duty_cycle_percent){
    ASSERT(duty_cycle_percent <= 100);

    #if defined ROBOTIC_ARM
        const bool enable = duty_cycle_percent > 0;
        if(enable){
            *pwm_channel_configs[pwm].ccr = pwm_scale_duty_cycle(duty_cycle_percent);
        }
        pwm_channel_enable(pwm, enable);
    #endif
}