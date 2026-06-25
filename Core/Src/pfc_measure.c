#include "pfc_measure.h"

#include "adc.h"
#include "hrtim.h"
#include "pfc_app.h"

volatile uint16_t adc_dma_buf[PFC_ADC_CH_COUNT] = {0U, 0U, 0U, 0U};

uint16_t raw_vac = 0U;
uint16_t raw_il = 0U;
uint16_t raw_vbus = 0U;
uint16_t raw_vrefint = 0U;

float vac_adc_v = 0.0f;
float il_adc_v = 0.0f;
float vbus_adc_v = 0.0f;
float adc_vref_manual_v = PFC_ADC_VREF_MANUAL_V;
float adc_vref_est_v = 0.0f;
float adc_vref_used_v = PFC_ADC_VREF_MANUAL_V;
float il_zero_runtime_v = PFC_IL_ZERO_V;
uint8_t il_zero_calibrated = 0U;
float il_zero_cal_span_v = 0.0f;

float vac_inst_v = 0.0f;
float vac_abs_v = 0.0f;
float vbus_v = 0.0f;
float il_a = 0.0f;
float pfc_vac_fast_v = 0.0f;
float pfc_vbus_fast_v = 0.0f;
float pfc_vbus_ctrl_v = 0.0f;

float vac_rms_v = 0.0f;
float il_rms_a = 0.0f;
float pin_w = 0.0f;
float pf_est = 0.0f;

volatile uint16_t raw_il_sync = 0U;
volatile float il_sync_adc_v = 0.0f;
volatile float il_sync_a = 0.0f;
volatile uint8_t il_sync_valid = 0U;
volatile uint32_t il_sync_update_count = 0U;
volatile uint32_t il_sync_last_update_ms = 0U;
volatile uint32_t il_sync_last_count = 0U;
volatile uint32_t pfc_adc_sync_pulse_cnt = 0U;
volatile uint32_t pfc_adc_inj_eoc_cnt = 0U;

static float filt_vac_raw = 0.0f;
static float filt_il_raw = 0.0f;
static float filt_vbus_raw = 0.0f;
static float filt_vrefint_raw = 0.0f;
static uint8_t filter_seeded = 0U;
static float pfc_vbus_line_sum = 0.0f;
static uint32_t pfc_vbus_line_count = 0U;
static uint32_t pfc_vbus_line_last_ms = 0U;
static uint8_t pfc_control_filter_seeded = 0U;

static float PFC_AbsFloat(float value)
{
  return (value >= 0.0f) ? value : -value;
}

static uint16_t PFC_RawFloatToU16(float value)
{
  if (value <= 0.0f)
  {
    return 0U;
  }
  if (value >= PFC_ADC_FULL_SCALE)
  {
    return (uint16_t)PFC_ADC_FULL_SCALE;
  }
  return (uint16_t)(value + 0.5f);
}

static float PFC_RawToAdcVoltage(uint16_t raw)
{
  return ((float)raw * adc_vref_used_v) / PFC_ADC_FULL_SCALE;
}

#if (PFC_DEBUG_ADC_TIMING_PULSE_ENABLE != 0) && (PFC_DEBUG_ADC_TIMING_USE_HRTIM == 0)
static void PFC_Measure_ConfigDebugTimingPulseGpio(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* PB0 is intentionally outside PA10/PA11 PWM, ADC pins, OLED I2C, and key IO. */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(PFC_DEBUG_ADC_TIMING_GPIO_PORT,
                    PFC_DEBUG_ADC_TIMING_GPIO_PIN,
                    GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = PFC_DEBUG_ADC_TIMING_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(PFC_DEBUG_ADC_TIMING_GPIO_PORT, &GPIO_InitStruct);
}

static void PFC_Measure_DebugAdcTimingPulse(void)
{
  pfc_adc_sync_pulse_cnt++;

  /*
   * Fallback marker: this GPIO pulse is emitted at ADC injected conversion
   * complete interrupt time. It is not the exact HRTIM CMP2 trigger instant.
   */
  PFC_DEBUG_ADC_TIMING_GPIO_PORT->BSRR = (uint32_t)PFC_DEBUG_ADC_TIMING_GPIO_PIN;
  __NOP(); __NOP(); __NOP(); __NOP();
  __NOP(); __NOP(); __NOP(); __NOP();
  __NOP(); __NOP(); __NOP(); __NOP();
  __NOP(); __NOP(); __NOP(); __NOP();
  __NOP(); __NOP(); __NOP(); __NOP();
  __NOP(); __NOP(); __NOP(); __NOP();
  __NOP(); __NOP(); __NOP(); __NOP();
  __NOP(); __NOP(); __NOP(); __NOP();
  PFC_DEBUG_ADC_TIMING_GPIO_PORT->BSRR = ((uint32_t)PFC_DEBUG_ADC_TIMING_GPIO_PIN << 16U);
}
#else
static void PFC_Measure_ConfigDebugTimingPulseGpio(void)
{
}

static void PFC_Measure_DebugAdcTimingPulse(void)
{
}
#endif

static void PFC_UpdateSyncCurrentFromRaw(uint16_t raw)
{
  const float adc_v = PFC_RawToAdcVoltage(raw);

  raw_il_sync = raw;
  il_sync_adc_v = adc_v;
  if (PFC_IL_GAIN_V_PER_A > 0.000001f)
  {
    il_sync_a = ((adc_v - il_zero_runtime_v) / PFC_IL_GAIN_V_PER_A) * PFC_IL_SIGN;
  }
  else
  {
    il_sync_a = 0.0f;
  }
  il_sync_valid = 1U;
  il_sync_update_count++;
  il_sync_last_count = il_sync_update_count;
  il_sync_last_update_ms = HAL_GetTick();
}

static void PFC_UpdateAdcVref(void)
{
  uint8_t estimate_valid = 0U;

  adc_vref_manual_v = PFC_ADC_VREF_MANUAL_V;

#if (PFC_USE_VREFINT_ESTIMATE != 0)
#if defined(__HAL_ADC_CALC_VREFANALOG_VOLTAGE)
  if ((raw_vrefint > 0U) && ((float)raw_vrefint < PFC_ADC_FULL_SCALE))
  {
    const uint32_t vref_mv = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(raw_vrefint, ADC_RESOLUTION_12B);

    if ((vref_mv >= 1500UL) && (vref_mv <= 3600UL))
    {
      adc_vref_est_v = (float)vref_mv * 0.001f;
      estimate_valid = 1U;
    }
    else
    {
      adc_vref_est_v = 0.0f;
    }
  }
  else
  {
    adc_vref_est_v = 0.0f;
  }
#else
  /*
   * STM32G4 HAL normally provides __HAL_ADC_CALC_VREFANALOG_VOLTAGE(),
   * backed by VREFINT_CAL_ADDR and VREFINT_CAL_VREF. If that macro is not
   * present in a different HAL pack, keep the estimate invalid and fall back.
   */
  adc_vref_est_v = 0.0f;
#endif
#else
  adc_vref_est_v = 0.0f;
#endif

  if (estimate_valid != 0U)
  {
    adc_vref_used_v = adc_vref_est_v;
  }
  else
  {
#if (PFC_USE_MANUAL_VREF_FALLBACK != 0)
    adc_vref_used_v = adc_vref_manual_v;
#else
    adc_vref_used_v = adc_vref_manual_v;
#endif
  }
}

static uint8_t PFC_Measure_ConfigHrtimSyncTrigger(void)
{
  HRTIM_ADCTriggerCfgTypeDef adc_trigger_cfg = {0};

  /*
   * HRTIM ADC Trigger 4 is driven by Timer B Compare2. CMP2 is updated by the
   * PWM layer to sit near the low-side on-time midpoint.
   */
  adc_trigger_cfg.UpdateSource = HRTIM_ADCTRIGGERUPDATE_TIMER_B;
  adc_trigger_cfg.Trigger = HRTIM_ADCTRIGGEREVENT24_TIMERB_CMP2;
  if (HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_4, &adc_trigger_cfg) != HAL_OK)
  {
    return 0U;
  }

  if (HAL_HRTIM_ADCPostScalerConfig(&hhrtim1, HRTIM_ADCTRIGGER_4, 0U) != HAL_OK)
  {
    return 0U;
  }

  return 1U;
}

static uint8_t PFC_Measure_ConfigInjectedCurrent(void)
{
  ADC_InjectionConfTypeDef inj_config = {0};

  inj_config.InjectedChannel = ADC_CHANNEL_2;                 /* PA1 / ADC1_IN2 = IL */
  inj_config.InjectedRank = ADC_INJECTED_RANK_1;
  inj_config.InjectedSamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  inj_config.InjectedSingleDiff = ADC_SINGLE_ENDED;
  inj_config.InjectedOffsetNumber = ADC_OFFSET_NONE;
  inj_config.InjectedOffset = 0U;
  inj_config.InjectedOffsetSign = ADC_OFFSET_SIGN_NEGATIVE;
  inj_config.InjectedOffsetSaturation = DISABLE;
  inj_config.InjectedNbrOfConversion = 1U;
  inj_config.InjectedDiscontinuousConvMode = DISABLE;
  inj_config.AutoInjectedConv = DISABLE;
  inj_config.QueueInjectedContext = DISABLE;
  inj_config.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_HRTIM_TRG4;
  inj_config.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  inj_config.InjecOversamplingMode = DISABLE;

  return (HAL_ADCEx_InjectedConfigChannel(&hadc1, &inj_config) == HAL_OK) ? 1U : 0U;
}

void PFC_Measure_Start(void)
{
  uint8_t sync_config_ok;

  /*
   * Calibrate ADC1 before starting DMA. The stop call is harmless at boot and
   * keeps this safe if the measure layer is restarted during debug.
   */
  (void)HAL_ADC_Stop_DMA(&hadc1);
  (void)HAL_ADCEx_InjectedStop_IT(&hadc1);

  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }

  filter_seeded = 0U;
  pfc_vac_fast_v = 0.0f;
  pfc_vbus_fast_v = 0.0f;
  pfc_vbus_ctrl_v = 0.0f;
  pfc_vbus_line_sum = 0.0f;
  pfc_vbus_line_count = 0U;
  pfc_vbus_line_last_ms = HAL_GetTick();
  pfc_control_filter_seeded = 0U;
  raw_il_sync = 0U;
  il_sync_adc_v = 0.0f;
  il_sync_a = 0.0f;
  il_sync_valid = 0U;
  il_sync_update_count = 0U;
  il_sync_last_update_ms = 0U;
  il_sync_last_count = 0U;
  pfc_adc_sync_pulse_cnt = 0U;
  pfc_adc_inj_eoc_cnt = 0U;

  PFC_Measure_ConfigDebugTimingPulseGpio();

  sync_config_ok = PFC_Measure_ConfigInjectedCurrent();
  if (sync_config_ok != 0U)
  {
    sync_config_ok = PFC_Measure_ConfigHrtimSyncTrigger();
  }

  /*
   * Start ADC1 DMA circular scan with exactly four ranks:
   * [0] VAC/PB1, [1] IL/PA1, [2] VBUS/PC3, [3] VREFINT.
   */
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)(void *)adc_dma_buf, PFC_ADC_CH_COUNT) != HAL_OK)
  {
    Error_Handler();
  }

  /*
   * Synchronized IL sampling is a debug assist. If injected start fails, keep
   * the old DMA/IIR measurement path alive and let the current loop fall back.
   */
  if (sync_config_ok != 0U)
  {
    if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK)
    {
      il_sync_valid = 0U;
    }
  }
}

float PFC_Measure_GetCurrentZeroVoltage(void)
{
  return il_zero_runtime_v;
}

/*
 * This function must only be called while PWM is stopped and the power stage
 * has no inductor current. If power input is present and current can flow, the
 * calibration result will be wrong. Do not auto-recalibrate during runtime.
 */
void PFC_Measure_CalibrateCurrentZero(void)
{
#if (PFC_IL_ZERO_AUTO_CAL_ENABLE != 0)
  const uint32_t sample_count = PFC_IL_ZERO_CAL_SAMPLES;
  float sum_v = 0.0f;
  float min_v = 1000.0f;
  float max_v = -1000.0f;
  float avg_v;
  uint32_t i;

  il_zero_runtime_v = PFC_IL_ZERO_V;
  il_zero_calibrated = 0U;
  il_zero_cal_span_v = 0.0f;

  if (sample_count == 0U)
  {
    return;
  }

  HAL_Delay(PFC_IL_ZERO_CAL_SETTLE_MS);

  for (i = 0U; i < sample_count; i++)
  {
    const uint16_t raw = adc_dma_buf[PFC_ADC_BUF_IL];
    float sample_v;

    raw_vrefint = adc_dma_buf[PFC_ADC_BUF_VREFINT];
    PFC_UpdateAdcVref();
    sample_v = PFC_RawToAdcVoltage(raw);

    sum_v += sample_v;
    if (sample_v < min_v)
    {
      min_v = sample_v;
    }
    if (sample_v > max_v)
    {
      max_v = sample_v;
    }

    HAL_Delay(1U);
  }

  avg_v = sum_v / (float)sample_count;
  il_zero_cal_span_v = max_v - min_v;

  if ((avg_v >= PFC_IL_ZERO_CAL_MIN_V) &&
      (avg_v <= PFC_IL_ZERO_CAL_MAX_V) &&
      (il_zero_cal_span_v <= PFC_IL_ZERO_CAL_MAX_SPAN_V))
  {
    il_zero_runtime_v = avg_v;
    il_zero_calibrated = 1U;
  }
  else
  {
    il_zero_runtime_v = PFC_IL_ZERO_V;
    il_zero_calibrated = 0U;
  }
#else
  il_zero_runtime_v = PFC_IL_ZERO_V;
  il_zero_calibrated = 0U;
  il_zero_cal_span_v = 0.0f;
#endif
}

void PFC_Measure_Update(void)
{
  const uint16_t dma_vac = adc_dma_buf[PFC_ADC_BUF_VAC];
  const uint16_t dma_il = adc_dma_buf[PFC_ADC_BUF_IL];
  const uint16_t dma_vbus = adc_dma_buf[PFC_ADC_BUF_VBUS];
  const uint16_t dma_vrefint = adc_dma_buf[PFC_ADC_BUF_VREFINT];
  const uint32_t now_ms = HAL_GetTick();

  if (filter_seeded == 0U)
  {
    filt_vac_raw = (float)dma_vac;
    filt_il_raw = (float)dma_il;
    filt_vbus_raw = (float)dma_vbus;
    filt_vrefint_raw = (float)dma_vrefint;
    filter_seeded = 1U;
  }
  else
  {
    /* First-order IIR, alpha = 1/16, for a simple 16-sample-like smoothing. */
    filt_vac_raw += ((float)dma_vac - filt_vac_raw) * 0.0625f;
    filt_il_raw += ((float)dma_il - filt_il_raw) * 0.0625f;
    filt_vbus_raw += ((float)dma_vbus - filt_vbus_raw) * 0.0625f;
    filt_vrefint_raw += ((float)dma_vrefint - filt_vrefint_raw) * 0.0625f;
  }

  raw_vac = PFC_RawFloatToU16(filt_vac_raw);
  raw_il = PFC_RawFloatToU16(filt_il_raw);
  raw_vbus = PFC_RawFloatToU16(filt_vbus_raw);
  raw_vrefint = PFC_RawFloatToU16(filt_vrefint_raw);

  PFC_UpdateAdcVref();

  vac_adc_v = PFC_RawToAdcVoltage(raw_vac);
  il_adc_v = PFC_RawToAdcVoltage(raw_il);
  vbus_adc_v = PFC_RawToAdcVoltage(raw_vbus);

  vac_inst_v = (vac_adc_v - PFC_VAC_ZERO_V) * PFC_VAC_GAIN;
  vac_abs_v = PFC_AbsFloat(vac_inst_v);
  vbus_v = (vbus_adc_v - PFC_VBUS_ZERO_V) * PFC_VBUS_GAIN;

  if (pfc_control_filter_seeded == 0U)
  {
    pfc_vac_fast_v = vac_abs_v;
    pfc_vbus_fast_v = vbus_v;
    pfc_vbus_ctrl_v = vbus_v;
    pfc_control_filter_seeded = 1U;
  }
  else
  {
    pfc_vac_fast_v += PFC_VAC_FAST_ALPHA * (vac_abs_v - pfc_vac_fast_v);
    pfc_vbus_fast_v += PFC_VBUS_FAST_ALPHA * (vbus_v - pfc_vbus_fast_v);
  }

  pfc_vbus_line_sum += pfc_vbus_fast_v;
  pfc_vbus_line_count++;
  if ((now_ms - pfc_vbus_line_last_ms) >= PFC_VBUS_LINE_AVG_PERIOD_MS)
  {
    const float line_avg = pfc_vbus_line_sum / (float)pfc_vbus_line_count;

    pfc_vbus_ctrl_v += PFC_VBUS_CTRL_ALPHA * (line_avg - pfc_vbus_ctrl_v);
    pfc_vbus_line_sum = 0.0f;
    pfc_vbus_line_count = 0U;
    pfc_vbus_line_last_ms = now_ms;
  }

  if (PFC_IL_GAIN_V_PER_A > 0.000001f)
  {
    il_a = ((il_adc_v - il_zero_runtime_v) / PFC_IL_GAIN_V_PER_A) * PFC_IL_SIGN;
  }
  else
  {
    il_a = 0.0f;
  }

  /* Reserved for later PF/THD work. Keep deterministic in the safe debug build. */
  vac_rms_v = 0.0f;
  il_rms_a = 0.0f;
  pin_w = 0.0f;
  pf_est = 0.0f;
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    pfc_adc_inj_eoc_cnt++;
    PFC_Measure_DebugAdcTimingPulse();

    const uint32_t raw = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);

    PFC_UpdateSyncCurrentFromRaw((uint16_t)raw);
#if (PFC_CURRENT_LOOP_IN_ISR_ENABLE != 0)
    PFC_App_OnInjectedCurrentIsr();
#endif
  }
}
