#include "ultrasound.h"
static TIM_HandleTypeDef htim12;
#define ULTRASOUND_TRIG_GPIO_Port GPIOB
#define ULTRASOUND_TRIG_Pin GPIO_PIN_13

typedef enum {
    ULTRASOUND_IDLE = 0,
    ULTRASOUND_WAIT_RISING,
    ULTRASOUND_WAIT_FALLING,
    ULTRASOUND_CAPTURE_COMPLETE
} Ultrasound_State_t;

static volatile Ultrasound_State_t ultrasound_state = ULTRASOUND_IDLE;
static volatile uint16_t echo_rising_count;
static volatile uint16_t echo_pulse_us;
static uint32_t trigger_tick;
static uint32_t last_trigger_tick;
static uint16_t result_distance_mm;
static bool result_valid;
static bool result_ready;

static void Ultrasound_Trigger(void)
{
    uint16_t pulse_start;

    __HAL_TIM_SET_CAPTUREPOLARITY(&htim12,
                                  TIM_CHANNEL_1,
                                  TIM_INPUTCHANNELPOLARITY_RISING);
    __HAL_TIM_CLEAR_FLAG(&htim12, TIM_FLAG_CC1);
    ultrasound_state = ULTRASOUND_WAIT_RISING;
    trigger_tick = HAL_GetTick();
    last_trigger_tick = trigger_tick;

    HAL_GPIO_WritePin(ULTRASOUND_TRIG_GPIO_Port,
                      ULTRASOUND_TRIG_Pin,
                      GPIO_PIN_SET);

    pulse_start = (uint16_t)__HAL_TIM_GET_COUNTER(&htim12);
    while ((uint16_t)((uint16_t)__HAL_TIM_GET_COUNTER(&htim12) - pulse_start) < 12U) {
        /* HC-SR04 requires a trigger pulse of at least 10 us. */
    }

    HAL_GPIO_WritePin(ULTRASOUND_TRIG_GPIO_Port,
                      ULTRASOUND_TRIG_Pin,
                      GPIO_PIN_RESET);
}

HAL_StatusTypeDef Ultrasound_Init(void)
{
    GPIO_InitTypeDef gpio={0};
    TIM_IC_InitTypeDef ic={0};
    if(HAL_RCC_GetPCLK1Freq()!=42000000U || HAL_RCC_GetHCLKFreq()!=168000000U) return HAL_ERROR;
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM12_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
    gpio.Pin=GPIO_PIN_13; gpio.Mode=GPIO_MODE_OUTPUT_PP;
    gpio.Pull=GPIO_NOPULL;gpio.Speed=GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB,&gpio);
    gpio.Pin=GPIO_PIN_14;gpio.Mode=GPIO_MODE_AF_PP;gpio.Alternate=GPIO_AF9_TIM12;
    gpio.Pull=GPIO_PULLDOWN;HAL_GPIO_Init(GPIOB,&gpio);
    htim12.Instance=TIM12;htim12.Init.Prescaler=83;
    htim12.Init.CounterMode=TIM_COUNTERMODE_UP;htim12.Init.Period=65535;
    htim12.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    htim12.Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_DISABLE;
    if(HAL_TIM_IC_Init(&htim12)!=HAL_OK)return HAL_ERROR;
    ic.ICPolarity=TIM_INPUTCHANNELPOLARITY_RISING;
    ic.ICSelection=TIM_ICSELECTION_DIRECTTI;ic.ICPrescaler=TIM_ICPSC_DIV1;ic.ICFilter=4;
    if(HAL_TIM_IC_ConfigChannel(&htim12,&ic,TIM_CHANNEL_1)!=HAL_OK)return HAL_ERROR;
    HAL_NVIC_SetPriority(TIM8_BRK_TIM12_IRQn,2,0);
    HAL_NVIC_ClearPendingIRQ(TIM8_BRK_TIM12_IRQn);HAL_NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
    ultrasound_state = ULTRASOUND_IDLE;
    echo_rising_count = 0U;
    echo_pulse_us = 0U;
    trigger_tick = 0U;
    last_trigger_tick = 0U;
    result_distance_mm = 0U;
    result_valid = false;
    result_ready = false;

    HAL_GPIO_WritePin(ULTRASOUND_TRIG_GPIO_Port,
                      ULTRASOUND_TRIG_Pin,
                      GPIO_PIN_RESET);
    __HAL_TIM_SET_COUNTER(&htim12, 0U);
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim12,
                                  TIM_CHANNEL_1,
                                  TIM_INPUTCHANNELPOLARITY_RISING);
    return HAL_TIM_IC_Start_IT(&htim12, TIM_CHANNEL_1);
}

void Ultrasound_Update(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t mask=__get_PRIMASK();
    __disable_irq();

    if (ultrasound_state == ULTRASOUND_CAPTURE_COMPLETE) {
        uint32_t distance_mm = ((uint32_t)echo_pulse_us * 10U + 29U) / 58U;

        if ((distance_mm >= ULTRASOUND_MIN_DISTANCE_MM) &&
            (distance_mm <= ULTRASOUND_MAX_DISTANCE_MM)) {
            result_distance_mm = (uint16_t)distance_mm;
            result_valid = true;
        } else {
            result_distance_mm = 0U;
            result_valid = false;
        }

        result_ready = true;
        ultrasound_state = ULTRASOUND_IDLE;
    } else if ((ultrasound_state == ULTRASOUND_WAIT_RISING) ||
               (ultrasound_state == ULTRASOUND_WAIT_FALLING)) {
        if ((now - trigger_tick) >= ULTRASOUND_ECHO_TIMEOUT_MS) {
            __HAL_TIM_SET_CAPTUREPOLARITY(&htim12,
                                          TIM_CHANNEL_1,
                                          TIM_INPUTCHANNELPOLARITY_RISING);
            result_distance_mm = 0U;
            result_valid = false;
            result_ready = true;
            ultrasound_state = ULTRASOUND_IDLE;
        }
    }

    __set_PRIMASK(mask);
    if ((ultrasound_state == ULTRASOUND_IDLE) &&
        ((now - last_trigger_tick) >= ULTRASOUND_TRIGGER_INTERVAL_MS)) {
        Ultrasound_Trigger();
    }
}

bool Ultrasound_TakeMeasurement(uint16_t *distance_mm, bool *valid)
{
    if (!result_ready) {
        return false;
    }

    if (distance_mm != NULL) {
        *distance_mm = result_distance_mm;
    }
    if (valid != NULL) {
        *valid = result_valid;
    }

    result_ready = false;
    return true;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if ((htim->Instance != TIM12) ||
        (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1)) {
        return;
    }

    if (ultrasound_state == ULTRASOUND_WAIT_RISING) {
        echo_rising_count =
            (uint16_t)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        __HAL_TIM_SET_CAPTUREPOLARITY(htim,
                                      TIM_CHANNEL_1,
                                      TIM_INPUTCHANNELPOLARITY_FALLING);
        ultrasound_state = ULTRASOUND_WAIT_FALLING;
    } else if (ultrasound_state == ULTRASOUND_WAIT_FALLING) {
        uint16_t echo_falling_count =
            (uint16_t)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

        echo_pulse_us = (uint16_t)(echo_falling_count - echo_rising_count);
        __HAL_TIM_SET_CAPTUREPOLARITY(htim,
                                      TIM_CHANNEL_1,
                                      TIM_INPUTCHANNELPOLARITY_RISING);
        ultrasound_state = ULTRASOUND_CAPTURE_COMPLETE;
    }
}

/* TIM8 break interrupt is unused; TIM8 encoder does not enable it. */
void TIM8_BRK_TIM12_IRQHandler(void) { HAL_TIM_IRQHandler(&htim12); }
