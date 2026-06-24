#include "pfc_app.h"

#include <stdio.h>

#include "gpio.h"
#include "pfc_config.h"
#include "pfc_measure.h"
#include "pfc_pwm.h"

#define PFC_KEY_DEBOUNCE_MS    40U
#define PFC_APP_ERR_BOOST_TEST_VBUS 0x0801U
#define PFC_APP_ERR_AC_RECT_TEST_VBUS 0x0901U
#define PFC_APP_ERR_PFC_TEST_VBUS 0x0A01U

typedef enum
{
  PFC_RUN_PHASE_PRECHARGE = 0,
  PFC_RUN_PHASE_SHAPE = 1
} PFC_RunPhase_t;

static PFC_State_t pfc_state = PFC_STATE_IDLE;
static PFC_Fault_t pfc_fault = PFC_FAULT_NONE;
static GPIO_PinState key_stable_level = GPIO_PIN_SET;
static GPIO_PinState key_last_sample = GPIO_PIN_SET;
static uint32_t key_last_change_ms = 0U;
static uint32_t oled_last_refresh_ms = 0U;
static uint32_t boost_loop_last_ms = 0U;
static float boost_vref_v = PFC_BOOST_VREF_INIT_V;
static float boost_integrator = PFC_BOOST_CL_DUTY_INIT;
static float boost_duty_prev = PFC_BOOST_CL_DUTY_INIT;
static uint32_t ac_rect_loop_last_ms = 0U;
static uint32_t ac_rect_enter_ms = 0U;
static uint32_t ac_rect_vin_recover_ms = 0U;
static float ac_rect_vref_v = PFC_AC_RECT_VREF_INIT_V;
static float ac_rect_integrator = PFC_AC_RECT_DUTY_INIT;
static float ac_rect_duty_cmd = PFC_AC_RECT_DUTY_INIT;
static float ac_rect_duty_internal = 0.0f;
static float ac_rect_duty_output = 0.0f;
static uint8_t ac_rect_pwm_active = 0U;
static uint8_t ac_rect_vin_enabled = 0U;
static uint8_t il_ocp_cnt = 0U;
static uint8_t il_rev_cnt = 0U;
static uint8_t pfc_test_ilimit_cnt = 0U;
static volatile uint32_t pfc_isr_inj_div_cnt = 0U;
static volatile uint32_t pfc_isr_loop_count = 0U;
static volatile uint8_t pfc_isr_control_enabled_dbg = 0U;
static volatile uint8_t pfc_isr_ilimit_cnt = 0U;
static volatile uint8_t pfc_isr_fault_request = 0U;
static volatile uint8_t pfc_isr_headroom_ok_dbg = 0U;
static float pfc_vbus_avg = 0.0f;
static float pfc_vbus_ref = PFC_VBUS_REF_INIT_V;
static float pfc_vin_peak = 0.0f;
static float pfc_vin_shape = 0.0f;
static float pfc_iamp_cmd = 0.0f;
static float pfc_iref_a = 0.0f;
static float pfc_iloop_integrator = 0.0f;
static float pfc_vloop_integrator = 0.0f;
static float pfc_il_ctrl_avg = 0.0f;
static float pfc_il_feedback_used = 0.0f;
static float pfc_i_err_a = 0.0f;
static float pfc_duty_integrator_dbg = 0.0f;
static float pfc_duty_raw_dbg = 0.0f;
static float pfc_duty_slew_dbg = 0.0f;
static float pfc_duty_out_dbg = 0.0f;
#if (PFC_TEST_PROFILE_18VAC_SIMPLE_PFC != 0)
static float pfc_simple_duty_limit = PFC_SIMPLE_DUTY_LIMIT_INIT;
static float pfc_simple_headroom_v_dbg = 0.0f;
static float pfc_simple_duty_cap_dbg = PFC_SIMPLE_DUTY_LIMIT_INIT;
static uint8_t pfc_simple_headroom_cap_dbg = 0U;
static uint8_t pfc_simple_no_demand_dbg = 0U;
static float pfc_simple_preboost_vref = 0.0f;
static float pfc_simple_preboost_integrator = 0.0f;
static float pfc_simple_preboost_duty = 0.0f;
static uint32_t pfc_simple_preboost_last_ms = 0U;
static uint8_t pfc_simple_preboost_hold_dbg = 0U;
#endif
static uint8_t pfc_i_accel_active = 0U;
static float pfc_il_feedback_dbg = 0.0f;
static float pfc_iref_dbg = 0.0f;
static float pfc_duty_ff = 0.0f;
static float pfc_duty_pi = 0.0f;
static float pfc_duty_cmd = 0.0f;
static uint32_t pfc_current_loop_last_us = 0U;
static uint32_t pfc_current_loop_update_count = 0U;
static uint32_t pfc_vloop_last_ms = 0U;
static uint8_t pfc_pwm_active = 0U;
static uint8_t pfc_ac_i_vin_enabled = 0U;
static uint8_t pfc_ac_i_margin_blank_dbg = 0U;
static uint8_t pfc_run_margin_blank_dbg = 0U;
static uint8_t pfc_pre_zero_blank_dbg = 0U;
static uint8_t pfc_pre_soft_limit_dbg = 0U;
static uint8_t pfc_pre_duty_clamp_dbg = 0U;
static uint8_t pfc_burst_blank_dbg = 0U;
static uint8_t pfc_vloop_freeze_dbg = 0U;
static uint8_t pfc_iloop_freeze_dbg = 0U;
static PFC_RunPhase_t pfc_run_phase = PFC_RUN_PHASE_PRECHARGE;
static float pfc_precharge_duty = PFC_PFC_PRECHARGE_DUTY_INIT;
static float pfc_precharge_iref_cmd = PFC_PFC_PRECHARGE_IREF_INIT_A;
static uint8_t pfc_dwt_ready = 0U;
static float fault_vin_v = 0.0f;
static float fault_il_a = 0.0f;
static float fault_il_sync_a = 0.0f;
static float fault_vbus_v = 0.0f;
static float fault_duty = 0.0f;
static float fault_iref = 0.0f;
static PFC_State_t fault_prev_state = PFC_STATE_IDLE;
static volatile float fault_peak_isr_il_sync_a = 0.0f;
static volatile float fault_peak_isr_il_dma_a = 0.0f;
static volatile float fault_peak_isr_duty = 0.0f;
static volatile uint8_t fault_peak_isr_cnt = 0U;
static volatile float openloop_pk_is = 0.0f;
static volatile float openloop_pk_il = 0.0f;
static volatile float openloop_last_is = 0.0f;
static volatile float openloop_last_il = 0.0f;
static volatile uint32_t openloop_isr_count = 0U;
static volatile uint8_t openloop_ocp_cnt = 0U;

static void PFC_EnterFault(PFC_Fault_t fault);
static void PFC_CurrentLoop_Task(float i_ref, uint8_t freeze_integrator);
static void PFC_CurrentLoop_Core_Isr(float i_ref, float il_feedback, float duty_max);
static uint8_t PFC_HandleIsrFaultRequest(void);

static float PFC_ClampFloat(float value, float min_value, float max_value)
{
  if (max_value < min_value)
  {
    return min_value;
  }
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}

static void PFC_FormatFixed(char *buffer, size_t buffer_size, float value, uint8_t decimals)
{
  int32_t scale = 10L;
  int32_t scaled;
  const char *sign = "";

  if (decimals == 2U)
  {
    scale = 100L;
  }
  else if (decimals == 3U)
  {
    scale = 1000L;
  }

  scaled = (value >= 0.0f) ? (int32_t)((value * (float)scale) + 0.5f) : (int32_t)((value * (float)scale) - 0.5f);

  if (scaled < 0L)
  {
    sign = "-";
    scaled = -scaled;
  }

  if (decimals == 3U)
  {
    (void)snprintf(buffer,
                   buffer_size,
                   "%s%ld.%03ld",
                   sign,
                   (long)(scaled / scale),
                   (long)(scaled % scale));
  }
  else if (decimals == 2U)
  {
    (void)snprintf(buffer,
                   buffer_size,
                   "%s%ld.%02ld",
                   sign,
                   (long)(scaled / scale),
                   (long)(scaled % scale));
  }
  else
  {
    (void)snprintf(buffer,
                   buffer_size,
                   "%s%ld.%01ld",
                   sign,
                   (long)(scaled / scale),
                   (long)(scaled % scale));
  }
}

__weak void PFC_OLED_Clear(void)
{
}

__weak void PFC_OLED_WriteLine(uint8_t line, const char *text)
{
  (void)line;
  (void)text;
}

__weak void PFC_OLED_Update(void)
{
}

static const char *PFC_StateName(PFC_State_t state)
{
  switch (state)
  {
    case PFC_STATE_IDLE:
      return "IDLE";
    case PFC_STATE_ADC_TEST:
      return "ADC";
    case PFC_STATE_PWM_ASYNC_TEST:
      return "BOOST_OL";
    case PFC_STATE_PWM_SYNC_TEST:
      return "SYNC_OFF";
    case PFC_STATE_BOOST_VOLTAGE_LOOP:
      return "DC_CL";
    case PFC_STATE_AC_RECT_VOLTAGE_LOOP:
      return "AC_CL";
    case PFC_STATE_DC_CURRENT_TEST:
      return "DC_I";
    case PFC_STATE_OPENLOOP_2PCT_TEST:
      return "OPEN_2PCT";
    case PFC_STATE_AC_CURRENT_SHAPE_TEST:
      return "AC_I";
    case PFC_STATE_PFC_RUN:
      return "PFC";
    case PFC_STATE_FAULT:
      return "FAULT";
    default:
      return "UNK";
  }
}

static float PFC_BoostTargetVref(void)
{
  return PFC_ClampFloat(PFC_BOOST_VREF_TARGET_V, 0.0f, PFC_BOOST_VREF_MAX_V);
}

static void PFC_BoostLoop_Reset(void)
{
  boost_vref_v = PFC_ClampFloat(PFC_BOOST_VREF_INIT_V, 0.0f, PFC_BOOST_VREF_MAX_V);
  boost_integrator = PFC_ClampFloat(PFC_BOOST_CL_DUTY_INIT,
                                    PFC_BOOST_CL_DUTY_MIN,
                                    PFC_BOOST_CL_DUTY_MAX);
  boost_duty_prev = boost_integrator;
  boost_loop_last_ms = HAL_GetTick();
}

static void PFC_CurrentProtectionCountersReset(void)
{
  il_ocp_cnt = 0U;
  il_rev_cnt = 0U;
  /* Fast test OCP now uses the PWM-synchronized injected current path. */
  pfc_test_ilimit_cnt = 0U;
  pfc_isr_ilimit_cnt = 0U;
  openloop_ocp_cnt = 0U;
}

static float PFC_AcRectTargetVref(void)
{
  return PFC_ClampFloat(PFC_AC_RECT_VREF_TARGET_V, 0.0f, PFC_AC_RECT_VREF_MAX_V);
}

static void PFC_AcRectLoop_Reset(void)
{
  ac_rect_vref_v = PFC_ClampFloat(PFC_AC_RECT_VREF_INIT_V, 0.0f, PFC_AC_RECT_VREF_MAX_V);
  ac_rect_integrator = PFC_ClampFloat(PFC_AC_RECT_DUTY_INIT,
                                      PFC_AC_RECT_DUTY_MIN,
                                      PFC_AC_RECT_DUTY_MAX);
  ac_rect_duty_cmd = ac_rect_integrator;
  ac_rect_duty_internal = 0.0f;
  ac_rect_duty_output = 0.0f;
  ac_rect_loop_last_ms = HAL_GetTick();
  ac_rect_enter_ms = 0U;
  ac_rect_vin_recover_ms = 0U;
  ac_rect_pwm_active = 0U;
  ac_rect_vin_enabled = 0U;
}

static uint8_t PFC_IsPfcTestState(void)
{
  return ((pfc_state == PFC_STATE_DC_CURRENT_TEST) ||
          (pfc_state == PFC_STATE_AC_CURRENT_SHAPE_TEST) ||
          (pfc_state == PFC_STATE_PFC_RUN)) ? 1U : 0U;
}

#if (PFC_USE_SYNC_IL_FOR_CURRENT_LOOP != 0)
static uint8_t PFC_IsIlSyncFresh(void)
{
  if (il_sync_valid != 0U)
  {
    if ((HAL_GetTick() - il_sync_last_update_ms) <= PFC_IL_SYNC_STALE_TIMEOUT_MS)
    {
      return 1U;
    }
  }

  return 0U;
}
#endif

static void PFC_MicroTimerInit(void)
{
  if (pfc_dwt_ready != 0U)
  {
    return;
  }

#if defined(DWT) && defined(CoreDebug_DEMCR_TRCENA_Msk) && defined(DWT_CTRL_CYCCNTENA_Msk)
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U)
  {
    pfc_dwt_ready = 1U;
  }
#endif
}

static uint32_t PFC_GetMicros(void)
{
#if defined(DWT) && defined(DWT_CTRL_CYCCNTENA_Msk)
  if ((pfc_dwt_ready != 0U) && (SystemCoreClock >= 1000000U))
  {
    return DWT->CYCCNT / (SystemCoreClock / 1000000U);
  }
#endif

  return HAL_GetTick() * 1000U;
}

static void PFC_IsrControlReset(void)
{
  pfc_isr_inj_div_cnt = 0U;
  pfc_isr_loop_count = 0U;
  pfc_isr_control_enabled_dbg = 0U;
  pfc_isr_ilimit_cnt = 0U;
  pfc_isr_fault_request = (uint8_t)PFC_FAULT_NONE;
  pfc_isr_headroom_ok_dbg = 0U;
  fault_peak_isr_il_sync_a = 0.0f;
  fault_peak_isr_il_dma_a = 0.0f;
  fault_peak_isr_duty = 0.0f;
  fault_peak_isr_cnt = 0U;
}

static float PFC_OpenLoop2PctDuty(void)
{
  return PFC_ClampFloat(PFC_OPENLOOP_2PCT_DUTY,
                        0.0f,
                        PFC_OPENLOOP_2PCT_DUTY_MAX);
}

#if (PFC_OPENLOOP_2PCT_TEST_ENABLE != 0)
static void PFC_OpenLoop2PctDiagnosticsReset(void)
{
  openloop_pk_is = 0.0f;
  openloop_pk_il = 0.0f;
  openloop_last_is = 0.0f;
  openloop_last_il = 0.0f;
  openloop_isr_count = 0U;
  openloop_ocp_cnt = 0U;
}

static void PFC_OpenLoop2PctApplyDebug(float duty)
{
  pfc_duty_cmd = duty;
  pfc_duty_raw_dbg = duty;
  pfc_duty_slew_dbg = 0.0f;
  pfc_duty_out_dbg = duty;
  pfc_duty_ff = 0.0f;
  pfc_duty_pi = 0.0f;
  pfc_iref_a = 0.0f;
  pfc_iref_dbg = 0.0f;
  pfc_i_err_a = 0.0f;
  pfc_iamp_cmd = 0.0f;
}
#endif

static void PFC_PfcControl_Reset(void)
{
  PFC_MicroTimerInit();

  pfc_vbus_avg = vbus_v;
  pfc_vbus_ref = PFC_ClampFloat(pfc_vbus_avg, PFC_VBUS_REF_INIT_V, PFC_VBUS_REF_TARGET_V);
  pfc_vin_peak = vac_abs_v;
  pfc_vin_shape = 0.0f;
  pfc_iamp_cmd = PFC_IAMP_CMD_INIT_A;
  pfc_iref_a = 0.0f;
  pfc_iloop_integrator = 0.0f;
  pfc_vloop_integrator = 0.0f;
  pfc_il_ctrl_avg = il_a;
  pfc_il_feedback_used = il_a;
  pfc_i_err_a = 0.0f;
  pfc_duty_integrator_dbg = 0.0f;
  pfc_duty_raw_dbg = 0.0f;
  pfc_duty_slew_dbg = 0.0f;
  pfc_duty_out_dbg = 0.0f;
#if (PFC_TEST_PROFILE_18VAC_SIMPLE_PFC != 0)
  pfc_simple_duty_limit = PFC_SIMPLE_DUTY_LIMIT_INIT;
  pfc_simple_headroom_v_dbg = 0.0f;
  pfc_simple_duty_cap_dbg = PFC_SIMPLE_DUTY_LIMIT_INIT;
  pfc_simple_headroom_cap_dbg = 0U;
  pfc_simple_no_demand_dbg = 0U;
  pfc_simple_preboost_vref = PFC_ClampFloat(vbus_v, 0.0f, PFC_SIMPLE_PREBOOST_TARGET_V);
  pfc_simple_preboost_integrator = 0.0f;
  pfc_simple_preboost_duty = 0.0f;
  pfc_simple_preboost_last_ms = HAL_GetTick();
  pfc_simple_preboost_hold_dbg = 0U;
#endif
  pfc_i_accel_active = 0U;
  pfc_il_feedback_dbg = il_a;
  pfc_iref_dbg = 0.0f;
  pfc_duty_ff = 0.0f;
  pfc_duty_pi = 0.0f;
  pfc_duty_cmd = 0.0f;
  pfc_current_loop_last_us = PFC_GetMicros();
  pfc_current_loop_update_count = 0U;
  pfc_vloop_last_ms = HAL_GetTick();
  pfc_pwm_active = 0U;
  PFC_IsrControlReset();
  pfc_ac_i_vin_enabled = 0U;
  pfc_ac_i_margin_blank_dbg = 0U;
  pfc_run_margin_blank_dbg = 0U;
  pfc_pre_zero_blank_dbg = 0U;
  pfc_pre_soft_limit_dbg = 0U;
  pfc_pre_duty_clamp_dbg = 0U;
  pfc_burst_blank_dbg = 0U;
  pfc_vloop_freeze_dbg = 0U;
  pfc_iloop_freeze_dbg = 0U;
#if (PFC_TEST_PROFILE_18VAC_SIMPLE_PFC != 0)
#if (PFC_SIMPLE_PREBOOST_ENABLE != 0)
  pfc_run_phase = PFC_RUN_PHASE_PRECHARGE;
#else
  pfc_run_phase = PFC_RUN_PHASE_SHAPE;
#endif
#else
  pfc_run_phase = (PFC_PFC_PRECHARGE_ENABLE != 0) ? PFC_RUN_PHASE_PRECHARGE : PFC_RUN_PHASE_SHAPE;
#endif
  pfc_precharge_duty = PFC_ClampFloat(PFC_PFC_PRECHARGE_DUTY_INIT,
                                      0.0f,
                                      PFC_PFC_PRECHARGE_DUTY_MAX);
  pfc_precharge_iref_cmd = PFC_PFC_PRECHARGE_IREF_INIT_A;
}

#if (PFC_TEST_PROFILE_18VAC_SIMPLE_PFC != 0)
static float PFC_SimplePfcGetHeadroomDutyCap(void)
{
  float headroom_v = vbus_v - vac_abs_v;
  float cap = pfc_simple_duty_limit;

  pfc_simple_headroom_v_dbg = headroom_v;
  pfc_simple_headroom_cap_dbg = 0U;

#if (PFC_SIMPLE_HEADROOM_CAP_ENABLE != 0)
  if (headroom_v <= PFC_SIMPLE_HEADROOM_MIN_V)
  {
    cap = PFC_SIMPLE_HEADROOM_DUTY_MIN;
    pfc_simple_headroom_cap_dbg = 1U;
  }
  else if (headroom_v < PFC_SIMPLE_HEADROOM_FULL_V)
  {
    const float k = headroom_v / PFC_SIMPLE_HEADROOM_FULL_V;
    cap = PFC_SIMPLE_HEADROOM_DUTY_MIN +
          (k * (pfc_simple_duty_limit - PFC_SIMPLE_HEADROOM_DUTY_MIN));
    pfc_simple_headroom_cap_dbg = 1U;
  }
  else
  {
    cap = pfc_simple_duty_limit;
  }

  cap = PFC_ClampFloat(cap, PFC_SIMPLE_HEADROOM_DUTY_MIN, pfc_simple_duty_limit);
#endif

  pfc_simple_duty_cap_dbg = cap;
  return cap;
}
#endif

static float PFC_ApplyCurrentLoopDutySlew(float duty_raw)
{
  float duty_out;
  float duty_delta;
  float slew_up = PFC_I_DUTY_SLEW_UP_PER_TICK;
  float slew_down = PFC_I_DUTY_SLEW_DOWN_PER_TICK;

#if (PFC_TEST_PROFILE_18VAC_SIMPLE_PFC != 0)
  if (pfc_state == PFC_STATE_PFC_RUN)
  {
    const float duty_limit_now = PFC_SimplePfcGetHeadroomDutyCap();
    duty_out = PFC_ClampFloat(duty_raw, 0.0f, duty_limit_now);
    pfc_duty_slew_dbg = duty_out - pfc_duty_out_dbg;
    pfc_duty_out_dbg = duty_out;
    return duty_out;
  }
#endif

#if (PFC_AC_I_DUTY_SLEW_ENABLE != 0)
  if (pfc_state == PFC_STATE_AC_CURRENT_SHAPE_TEST)
  {
    slew_up = PFC_AC_I_DUTY_SLEW_UP_PER_TICK;
    slew_down = PFC_AC_I_DUTY_SLEW_DOWN_PER_TICK;
  }
#endif

#if (PFC_PFC_RUN_DUTY_SLEW_ENABLE != 0)
  if (pfc_state == PFC_STATE_PFC_RUN)
  {
    slew_up = PFC_PFC_RUN_DUTY_SLEW_UP_PER_TICK;
    slew_down = PFC_PFC_RUN_DUTY_SLEW_DOWN_PER_TICK;
  }
#endif

#if (PFC_I_DUTY_SLEW_ENABLE != 0)
  duty_delta = duty_raw - pfc_duty_out_dbg;
  if (duty_delta > slew_up)
  {
    duty_delta = slew_up;
  }
  else if (duty_delta < -slew_down)
  {
    duty_delta = -slew_down;
  }

  duty_out = pfc_duty_out_dbg + duty_delta;
  pfc_duty_slew_dbg = duty_delta;
#else
  duty_delta = duty_raw - pfc_duty_out_dbg;
  duty_out = duty_raw;
  pfc_duty_slew_dbg = duty_delta;
#endif

  if (pfc_state == PFC_STATE_AC_CURRENT_SHAPE_TEST)
  {
    duty_out = PFC_ClampFloat(duty_out, 0.0f, PFC_AC_I_DUTY_MAX);
  }
  else if (pfc_state == PFC_STATE_PFC_RUN)
  {
#if (PFC_TEST_PROFILE_18VAC_SIMPLE_PFC != 0)
    duty_out = PFC_ClampFloat(duty_out, 0.0f, PFC_PFC_RUN_DUTY_MAX);
#else
    const float duty_before_clamp = duty_out;

    if (pfc_run_phase == PFC_RUN_PHASE_PRECHARGE)
    {
      duty_out = PFC_ClampFloat(duty_out, 0.0f, PFC_PFC_PRECHARGE_DUTY_MAX);
    }
    else
    {
      duty_out = PFC_ClampFloat(duty_out, 0.0f, PFC_PFC_RUN_DUTY_MAX);
    }

    if (duty_out != duty_before_clamp)
    {
      pfc_pre_duty_clamp_dbg = 1U;
    }
#endif
  }
  else
  {
    duty_out = PFC_ClampFloat(duty_out, PFC_PFC_DUTY_MIN, PFC_PFC_DUTY_MAX);
  }
  pfc_duty_out_dbg = duty_out;
  return duty_out;
}

static uint8_t PFC_PfcControl_Start(void)
{
  PFC_PfcControl_Reset();

  /*
   * DC/AC-rectified Boost/PFC test modes use the high-side MOSFET body diode only;
   * high-side gate drive is intentionally disabled.
   */
  PFC_PWM_AllOff();
  pfc_pwm_active = 0U;
  return 1U;
}

static void PFC_EnterFault(PFC_Fault_t fault)
{
  const float peak_isr_il_sync_a = fault_peak_isr_il_sync_a;
  const float peak_isr_il_dma_a = fault_peak_isr_il_dma_a;
  const float peak_isr_duty = fault_peak_isr_duty;
  const uint8_t peak_isr_cnt = fault_peak_isr_cnt;

  fault_vin_v = vac_abs_v;
  fault_il_a = il_a;
  fault_il_sync_a = il_sync_a;
  fault_vbus_v = vbus_v;
  fault_duty = pfc_duty_cmd;
  fault_iref = pfc_iref_a;
  fault_prev_state = pfc_state;

  PFC_PWM_AllOff();
  PFC_CurrentProtectionCountersReset();
  PFC_BoostLoop_Reset();
  PFC_AcRectLoop_Reset();
  PFC_PfcControl_Reset();
  if (fault != PFC_FAULT_NONE)
  {
    fault_peak_isr_il_sync_a = peak_isr_il_sync_a;
    fault_peak_isr_il_dma_a = peak_isr_il_dma_a;
    fault_peak_isr_duty = peak_isr_duty;
    fault_peak_isr_cnt = peak_isr_cnt;
  }
  pfc_fault = fault;
  pfc_state = PFC_STATE_FAULT;
}

static uint8_t PFC_IsCurrentProtectionBlanked(void)
{
  const uint32_t now_ms = HAL_GetTick();

  if (pfc_state == PFC_STATE_AC_RECT_VOLTAGE_LOOP)
  {
    if ((now_ms - ac_rect_enter_ms) < PFC_AC_CL_STARTUP_BLANK_MS)
    {
      return 1U;
    }
    if ((now_ms - ac_rect_vin_recover_ms) < PFC_AC_CL_VIN_RECOVER_BLANK_MS)
    {
      return 1U;
    }
  }

  return 0U;
}

static uint8_t PFC_BoostVoltageLoopStart(void)
{
  const float target_v = PFC_BoostTargetVref();

  if (vbus_v > PFC_BOOST_TEST_VBUS_LIMIT_V)
  {
    PFC_PWM_SetErrorCode(PFC_APP_ERR_BOOST_TEST_VBUS);
    return 0U;
  }

  boost_vref_v = vbus_v;
  if (boost_vref_v < PFC_BOOST_VREF_INIT_V)
  {
    boost_vref_v = PFC_BOOST_VREF_INIT_V;
  }
  boost_vref_v = PFC_ClampFloat(boost_vref_v, 0.0f, target_v);
  boost_integrator = PFC_ClampFloat(PFC_BOOST_CL_DUTY_INIT,
                                    PFC_BOOST_CL_DUTY_MIN,
                                    PFC_BOOST_CL_DUTY_MAX);
  boost_duty_prev = boost_integrator;
  boost_loop_last_ms = HAL_GetTick();

  PFC_PWM_StartBoostClosedLoop();
  if (PFC_PWM_GetErrorCode() != 0U)
  {
    return 0U;
  }

  PFC_PWM_SetDuty(boost_duty_prev);
  return (PFC_PWM_GetErrorCode() == 0U) ? 1U : 0U;
}

static void PFC_BoostVoltageLoopTask(void)
{
#if (PFC_BOOST_CL_ENABLE != 0)
  float target_v;
  float error_v;
  float integrator_candidate;
  float duty_unsat;
  float duty_cmd;
  float duty_next;
  float duty_delta;
  const uint32_t now_ms = HAL_GetTick();

  if (pfc_state != PFC_STATE_BOOST_VOLTAGE_LOOP)
  {
    return;
  }

  if (vbus_v > PFC_BOOST_TEST_VBUS_LIMIT_V)
  {
    PFC_PWM_SetErrorCode(PFC_APP_ERR_BOOST_TEST_VBUS);
    PFC_EnterFault(PFC_FAULT_TEST_OVP);
    return;
  }

  if ((now_ms - boost_loop_last_ms) < PFC_BOOST_CL_PERIOD_MS)
  {
    return;
  }
  boost_loop_last_ms = now_ms;

  target_v = PFC_BoostTargetVref();
  if (boost_vref_v < target_v)
  {
    boost_vref_v += PFC_BOOST_VREF_RAMP_STEP_V;
    if (boost_vref_v > target_v)
    {
      boost_vref_v = target_v;
    }
  }
  else if (boost_vref_v > target_v)
  {
    boost_vref_v = target_v;
  }

  error_v = boost_vref_v - vbus_v;
  integrator_candidate = boost_integrator + (PFC_BOOST_VLOOP_KI * error_v);
  integrator_candidate = PFC_ClampFloat(integrator_candidate,
                                        PFC_BOOST_CL_DUTY_MIN,
                                        PFC_BOOST_CL_DUTY_MAX);

  duty_unsat = (PFC_BOOST_VLOOP_KP * error_v) + integrator_candidate;
  if (!((duty_unsat > PFC_BOOST_CL_DUTY_MAX) && (error_v > 0.0f)) &&
      !((duty_unsat < PFC_BOOST_CL_DUTY_MIN) && (error_v < 0.0f)))
  {
    boost_integrator = integrator_candidate;
  }

  duty_cmd = (PFC_BOOST_VLOOP_KP * error_v) + boost_integrator;
  duty_cmd = PFC_ClampFloat(duty_cmd, PFC_BOOST_CL_DUTY_MIN, PFC_BOOST_CL_DUTY_MAX);

  duty_delta = duty_cmd - boost_duty_prev;
  duty_delta = PFC_ClampFloat(duty_delta,
                              -PFC_BOOST_DUTY_SLEW_PER_TICK,
                              PFC_BOOST_DUTY_SLEW_PER_TICK);
  duty_next = PFC_ClampFloat(boost_duty_prev + duty_delta,
                             PFC_BOOST_CL_DUTY_MIN,
                             PFC_BOOST_CL_DUTY_MAX);

  PFC_PWM_SetDuty(duty_next);
  boost_duty_prev = duty_next;

  if (PFC_PWM_GetErrorCode() != 0U)
  {
    PFC_EnterFault(PFC_FAULT_PWM);
  }
#endif
}

static uint8_t PFC_AcRectVoltageLoopStart(void)
{
  const float target_v = PFC_AcRectTargetVref();

  if (vbus_v > PFC_AC_RECT_TEST_VBUS_LIMIT_V)
  {
    PFC_PWM_SetErrorCode(PFC_APP_ERR_AC_RECT_TEST_VBUS);
    return 0U;
  }

  ac_rect_vref_v = vbus_v;
  if (ac_rect_vref_v < PFC_AC_RECT_VREF_INIT_V)
  {
    ac_rect_vref_v = PFC_AC_RECT_VREF_INIT_V;
  }
  ac_rect_vref_v = PFC_ClampFloat(ac_rect_vref_v, 0.0f, target_v);
  ac_rect_integrator = PFC_ClampFloat(PFC_AC_RECT_DUTY_INIT,
                                      PFC_AC_RECT_DUTY_MIN,
                                      PFC_AC_RECT_DUTY_MAX);
  ac_rect_duty_cmd = ac_rect_integrator;
  ac_rect_duty_internal = 0.0f;
  ac_rect_duty_output = 0.0f;
  ac_rect_loop_last_ms = HAL_GetTick();
  ac_rect_enter_ms = ac_rect_loop_last_ms;
  ac_rect_vin_recover_ms = ac_rect_loop_last_ms;
  ac_rect_vin_enabled = (vac_abs_v > PFC_AC_RECT_VIN_ENABLE_TH_V) ? 1U : 0U;

  PFC_PWM_AllOff();
  ac_rect_pwm_active = 0U;

  return 1U;
}

static void PFC_AcRectVoltageLoopTask(void)
{
#if (PFC_AC_RECT_CL_ENABLE != 0)
  float target_v;
  float vin_rectified_v;
  float error_v;
  float integrator_candidate;
  float duty_unsat;
  float duty_delta;
  uint8_t freeze_integrator = 0U;
  const uint32_t now_ms = HAL_GetTick();

  if (pfc_state != PFC_STATE_AC_RECT_VOLTAGE_LOOP)
  {
    return;
  }

  if (vbus_v > PFC_AC_RECT_TEST_VBUS_LIMIT_V)
  {
    PFC_PWM_SetErrorCode(PFC_APP_ERR_AC_RECT_TEST_VBUS);
    PFC_EnterFault(PFC_FAULT_TEST_OVP);
    return;
  }

  vin_rectified_v = vac_abs_v;
  /*
   * VAC/VIN channel is measured before the bridge rectifier.
   * vac_inst_v is signed AC instantaneous voltage.
   * vac_abs_v is abs(vac_inst_v), used as a rectified-voltage-shaped signal
   * for AC-rectified Boost test mode. It is not RMS.
   */
  if (vin_rectified_v < PFC_AC_RECT_VIN_DISABLE_TH_V)
  {
    freeze_integrator = 1U;
    PFC_PWM_AllOff();
    ac_rect_pwm_active = 0U;
    ac_rect_duty_output = 0.0f;
    ac_rect_vin_enabled = 0U;
    return;
  }

  if (ac_rect_vin_enabled == 0U)
  {
    if (vin_rectified_v <= PFC_AC_RECT_VIN_ENABLE_TH_V)
    {
      freeze_integrator = 1U;
      PFC_PWM_AllOff();
      ac_rect_pwm_active = 0U;
      ac_rect_duty_output = 0.0f;
      return;
    }

    ac_rect_vin_enabled = 1U;
    ac_rect_vin_recover_ms = now_ms;
  }

  if ((now_ms - ac_rect_loop_last_ms) < PFC_AC_RECT_CL_PERIOD_MS)
  {
    return;
  }
  ac_rect_loop_last_ms = now_ms;

  target_v = PFC_AcRectTargetVref();
  if (ac_rect_vref_v < target_v)
  {
    ac_rect_vref_v += PFC_AC_RECT_VREF_RAMP_STEP_V;
    if (ac_rect_vref_v > target_v)
    {
      ac_rect_vref_v = target_v;
    }
  }
  else if (ac_rect_vref_v > target_v)
  {
    ac_rect_vref_v = target_v;
  }

  error_v = ac_rect_vref_v - vbus_v;
  if (freeze_integrator == 0U)
  {
    integrator_candidate = ac_rect_integrator + (PFC_AC_RECT_VLOOP_KI * error_v);
    integrator_candidate = PFC_ClampFloat(integrator_candidate,
                                          PFC_AC_RECT_DUTY_MIN,
                                          PFC_AC_RECT_DUTY_MAX);

    duty_unsat = (PFC_AC_RECT_VLOOP_KP * error_v) + integrator_candidate;
    if (!((duty_unsat > PFC_AC_RECT_DUTY_MAX) && (error_v > 0.0f)) &&
        !((duty_unsat < PFC_AC_RECT_DUTY_MIN) && (error_v < 0.0f)))
    {
      ac_rect_integrator = integrator_candidate;
    }
  }

  ac_rect_duty_cmd = (PFC_AC_RECT_VLOOP_KP * error_v) + ac_rect_integrator;
  ac_rect_duty_cmd = PFC_ClampFloat(ac_rect_duty_cmd, PFC_AC_RECT_DUTY_MIN, PFC_AC_RECT_DUTY_MAX);

  duty_delta = ac_rect_duty_cmd - ac_rect_duty_internal;
  duty_delta = PFC_ClampFloat(duty_delta,
                              -PFC_AC_RECT_DUTY_SLEW_PER_TICK,
                              PFC_AC_RECT_DUTY_SLEW_PER_TICK);
  ac_rect_duty_internal = PFC_ClampFloat(ac_rect_duty_internal + duty_delta,
                                         0.0f,
                                         PFC_AC_RECT_DUTY_MAX);

  ac_rect_duty_output = (ac_rect_vin_enabled != 0U) ? ac_rect_duty_internal : 0.0f;
  if (ac_rect_duty_output <= PFC_PFC_DUTY_MIN_ACTIVE)
  {
    PFC_PWM_AllOff();
    ac_rect_pwm_active = 0U;
  }
  else if (ac_rect_pwm_active == 0U)
  {
    PFC_PWM_StartAcRectClosedLoopAtDuty(ac_rect_duty_output);
    ac_rect_pwm_active = (PFC_PWM_GetErrorCode() == 0U) ? 1U : 0U;
  }
  else
  {
    PFC_PWM_SetDuty(ac_rect_duty_output);
  }

  if (PFC_PWM_GetErrorCode() != 0U)
  {
    PFC_EnterFault(PFC_FAULT_PWM);
  }
#endif
}

static void PFC_UpdatePfcMeasurements(void)
{
  pfc_vbus_avg += PFC_VBUS_AVG_ALPHA * (vbus_v - pfc_vbus_avg);

  /*
   * VAC/VIN channel is measured before the bridge rectifier.
   * vac_inst_v is signed AC instantaneous voltage.
   * vac_abs_v is abs(vac_inst_v), used as a rectified-voltage-shaped signal
   * for PFC current reference generation. It is not RMS.
   */
  if (vac_abs_v > pfc_vin_peak)
  {
    pfc_vin_peak = vac_abs_v;
  }
  else
  {
    pfc_vin_peak *= PFC_VIN_PEAK_DECAY;
  }

  if (pfc_vin_peak < PFC_VIN_PEAK_MIN_V)
  {
    pfc_vin_shape = 0.0f;
  }
  else
  {
    pfc_vin_shape = PFC_ClampFloat(vac_abs_v / pfc_vin_peak, 0.0f, 1.0f);
  }
}

#if (PFC_RUN_ENABLE_DEFAULT != 0)
static void PFC_UpdatePfcVoltageLoopLimited(float iamp_max, float iamp_slew, uint8_t freeze_integrator)
{
  float v_err;
  float integrator_candidate;
  float iamp_unsat;
  float iamp_candidate;
  float iamp_target;
  float iamp_delta;
  const float iamp_prev = pfc_iamp_cmd;
  const uint32_t now_ms = HAL_GetTick();

  if ((now_ms - pfc_vloop_last_ms) < PFC_VLOOP_PERIOD_MS)
  {
    return;
  }
  pfc_vloop_last_ms = now_ms;

  if (pfc_vbus_ref < PFC_VBUS_REF_TARGET_V)
  {
    pfc_vbus_ref += PFC_VBUS_REF_RAMP_STEP_V;
    if (pfc_vbus_ref > PFC_VBUS_REF_TARGET_V)
    {
      pfc_vbus_ref = PFC_VBUS_REF_TARGET_V;
    }
  }
  else if (pfc_vbus_ref > PFC_VBUS_REF_TARGET_V)
  {
    pfc_vbus_ref = PFC_VBUS_REF_TARGET_V;
  }
  pfc_vbus_ref = PFC_ClampFloat(pfc_vbus_ref, 0.0f, PFC_VBUS_REF_MAX_V);

  v_err = pfc_vbus_ref - pfc_vbus_avg;
  integrator_candidate = pfc_vloop_integrator;

  if (freeze_integrator == 0U)
  {
    integrator_candidate += PFC_VLOOP_KI * v_err;
    integrator_candidate = PFC_ClampFloat(integrator_candidate, PFC_IAMP_CMD_MIN_A, iamp_max);

    iamp_unsat = (PFC_VLOOP_KP * v_err) + integrator_candidate;
    if (!((iamp_unsat > iamp_max) && (v_err > 0.0f)) &&
        !((iamp_unsat < PFC_IAMP_CMD_MIN_A) && (v_err < 0.0f)))
    {
      pfc_vloop_integrator = integrator_candidate;
    }
  }
  else
  {
    pfc_vloop_freeze_dbg = 1U;
  }

  iamp_candidate = (PFC_VLOOP_KP * v_err) + pfc_vloop_integrator;
  iamp_target = PFC_ClampFloat(iamp_candidate, PFC_IAMP_CMD_MIN_A, iamp_max);

  if ((pfc_state == PFC_STATE_PFC_RUN) && (pfc_vbus_avg > (pfc_vbus_ref + 0.3f)))
  {
    const float iamp_release_target = PFC_ClampFloat(iamp_prev - 0.002f,
                                                     PFC_IAMP_CMD_MIN_A,
                                                     iamp_max);
    if (iamp_target > iamp_release_target)
    {
      iamp_target = iamp_release_target;
    }
  }

  iamp_delta = iamp_target - iamp_prev;
  if (iamp_slew > 0.0f)
  {
    if (iamp_delta > iamp_slew)
    {
      iamp_delta = iamp_slew;
    }
    else if (iamp_delta < -iamp_slew)
    {
      iamp_delta = -iamp_slew;
    }
  }

  pfc_iamp_cmd = PFC_ClampFloat(iamp_prev + iamp_delta, PFC_IAMP_CMD_MIN_A, iamp_max);
}

static void PFC_UpdatePfcVoltageLoop(void)
{
  PFC_UpdatePfcVoltageLoopLimited(PFC_IAMP_CMD_MAX_A, PFC_PFC_SOFTSTART_IAMP_SLEW_A, 0U);
}
#endif

static float PFC_AcIGetDerateScale(void)
{
#if (PFC_AC_I_VBUS_DERATE_ENABLE != 0)
  if (vbus_v <= PFC_AC_I_VBUS_DERATE_START_V)
  {
    return 1.0f;
  }
  if (vbus_v >= PFC_AC_I_VBUS_DERATE_END_V)
  {
    return 0.0f;
  }

  return PFC_ClampFloat((PFC_AC_I_VBUS_DERATE_END_V - vbus_v) /
                        (PFC_AC_I_VBUS_DERATE_END_V - PFC_AC_I_VBUS_DERATE_START_V),
                        0.0f,
                        1.0f);
#else
  return 1.0f;
#endif
}

static float PFC_GetCurrentLoopDutyMax(void)
{
  if (pfc_state == PFC_STATE_AC_CURRENT_SHAPE_TEST)
  {
    return PFC_AC_I_DUTY_MAX;
  }

  if (pfc_state == PFC_STATE_PFC_RUN)
  {
    if (pfc_run_phase == PFC_RUN_PHASE_PRECHARGE)
    {
      return PFC_PFC_PRECHARGE_DUTY_MAX;
    }
    return PFC_PFC_RUN_DUTY_MAX;
  }

  return PFC_PFC_DUTY_MAX;
}

static float PFC_ApplyPfcVinFeedForward(float iamp)
{
#if (PFC_PFC_VIN_RMS_FF_ENABLE != 0)
  float ff_scale;

  if (pfc_vin_peak < PFC_VIN_PEAK_MIN_V)
  {
    return iamp;
  }

  ff_scale = PFC_PFC_VIN_NOMINAL_PEAK_V / pfc_vin_peak;
  ff_scale *= ff_scale;
  ff_scale = PFC_ClampFloat(ff_scale, PFC_PFC_VIN_FF_MIN_SCALE, PFC_PFC_VIN_FF_MAX_SCALE);
  return PFC_ClampFloat(iamp * ff_scale, 0.0f, PFC_IREF_MAX_A);
#else
  return iamp;
#endif
}

#if (PFC_TEST_PROFILE_18VAC_SIMPLE_PFC != 0)
static void PFC_SimplePfcBlankOutput(void)
{
  pfc_iref_a = 0.0f;
  pfc_iref_dbg = 0.0f;
  pfc_i_err_a = 0.0f;
  pfc_duty_ff = 0.0f;
  pfc_duty_pi = 0.0f;
  pfc_duty_raw_dbg = 0.0f;
  pfc_duty_slew_dbg = 0.0f;
  pfc_duty_out_dbg = 0.0f;
  pfc_duty_cmd = 0.0f;
  pfc_iloop_freeze_dbg = 1U;

  if (pfc_pwm_active != 0U)
  {
    PFC_PWM_SetDuty(0.0f);
  }
}

static float PFC_SimplePfcCurrentFeedback(void)
{
#if (PFC_USE_SYNC_IL_FOR_CURRENT_LOOP != 0)
  if (PFC_IsIlSyncFresh() != 0U)
  {
    return il_sync_a;
  }
#endif

  return il_a;
}

static uint8_t PFC_SimplePfcSoftCurrentLimit(void)
{
#if (PFC_PFC_RUN_IL_SOFT_LIMIT_ENABLE != 0)
  const float il_feedback = PFC_SimplePfcCurrentFeedback();

  if ((pfc_state == PFC_STATE_PFC_RUN) &&
      (il_feedback > PFC_PFC_RUN_IL_SOFT_LIMIT_A))
  {
    pfc_pre_soft_limit_dbg = 1U;
    pfc_il_feedback_dbg = il_feedback;
    pfc_i_err_a = pfc_iref_a - il_feedback;

    if (pfc_duty_out_dbg > PFC_PFC_RUN_IL_SOFT_DUTY_STEP)
    {
      pfc_duty_out_dbg -= PFC_PFC_RUN_IL_SOFT_DUTY_STEP;
    }
    else
    {
      pfc_duty_out_dbg = 0.0f;
    }

    pfc_duty_cmd = 0.0f;
    pfc_duty_raw_dbg = PFC_ClampFloat(pfc_duty_raw_dbg, 0.0f, pfc_duty_out_dbg);
    pfc_duty_slew_dbg = 0.0f;
    pfc_duty_ff = 0.0f;
    pfc_duty_pi = 0.0f;
    pfc_iloop_freeze_dbg = 1U;
    pfc_iloop_integrator = PFC_ClampFloat(pfc_iloop_integrator,
                                          -0.20f,
                                          pfc_duty_out_dbg);

    if (pfc_run_phase == PFC_RUN_PHASE_PRECHARGE)
    {
      pfc_simple_preboost_integrator = 0.0f;
      pfc_simple_preboost_duty = pfc_duty_out_dbg;
    }

    if (pfc_pwm_active != 0U)
    {
      PFC_PWM_SetDuty(0.0f);
    }

    return 1U;
  }
#endif

  return 0U;
}

static void PFC_SimplePfcNoDemandBlankOutput(void)
{
  pfc_iref_a = 0.0f;
  pfc_iref_dbg = 0.0f;
  pfc_i_err_a = 0.0f;

  pfc_duty_ff = 0.0f;
  pfc_duty_pi = 0.0f;
  pfc_duty_raw_dbg = 0.0f;
  pfc_duty_slew_dbg = 0.0f;
  pfc_duty_out_dbg = 0.0f;
  pfc_duty_cmd = 0.0f;

#if (PFC_SIMPLE_NO_DEMAND_CLEAR_I != 0)
  pfc_iloop_integrator = 0.0f;
  pfc_duty_integrator_dbg = 0.0f;
#endif

  /*
   * SIMPLE_PFC no-demand blank:
   * If voltage loop asks for no input current, Dff must not drive PWM by itself.
   * Only force actual duty to zero. Do not call PFC_PWM_AllOff(), do not clear
   * pfc_pwm_active, and keep PA10 high-side disabled.
   */
  (void)PFC_SIMPLE_NO_DEMAND_KEEP_PWM_ACTIVE;
  if (pfc_pwm_active != 0U)
  {
    PFC_PWM_SetDuty(0.0f);
  }
}

static void PFC_SimplePfcEnterRunPhase(void)
{
  pfc_run_phase = PFC_RUN_PHASE_SHAPE;
  pfc_vbus_ref = PFC_ClampFloat(pfc_vbus_avg, PFC_VBUS_REF_INIT_V, PFC_VBUS_REF_TARGET_V);

  pfc_iamp_cmd = 0.0f;
  pfc_vloop_integrator = 0.0f;
  pfc_iloop_integrator = 0.0f;
  pfc_il_ctrl_avg = (il_sync_valid != 0U) ? il_sync_a : il_a;
  pfc_il_feedback_dbg = pfc_il_ctrl_avg;
  pfc_il_feedback_used = pfc_il_ctrl_avg;
  pfc_duty_ff = 0.0f;
  pfc_duty_pi = 0.0f;
  pfc_duty_raw_dbg = 0.0f;
  pfc_duty_slew_dbg = 0.0f;
  pfc_duty_out_dbg = 0.0f;
  pfc_duty_cmd = 0.0f;
  pfc_duty_integrator_dbg = 0.0f;
  pfc_simple_duty_limit = PFC_SIMPLE_DUTY_LIMIT_INIT;
  pfc_simple_headroom_cap_dbg = 0U;
  pfc_simple_no_demand_dbg = 0U;
  pfc_simple_preboost_hold_dbg = 0U;
  PFC_IsrControlReset();
  pfc_vloop_last_ms = HAL_GetTick();
  pfc_current_loop_last_us = PFC_GetMicros();

  if (pfc_pwm_active != 0U)
  {
    PFC_PWM_SetDuty(0.0f);
  }
}

/*
 * SIMPLE_PFC startup preboost:
 * Before sinusoidal current shaping, raise VBUS above the input peak with a
 * slow voltage-to-duty loop. PWM is allowed only when VA is safely below VBUS,
 * so the inductor has a real discharge path through the high-side body diode.
 */
static void PFC_SimplePreboostTask(void)
{
  float error_v;
  float integrator_candidate;
  float duty_unsat;
  float duty_cmd;
  float duty_delta;
  uint8_t allow_pwm;

  pfc_simple_no_demand_dbg = 0U;
  pfc_simple_headroom_cap_dbg = 0U;
  pfc_simple_preboost_hold_dbg = 0U;
  pfc_simple_headroom_v_dbg = vbus_v - vac_abs_v;
  pfc_simple_duty_cap_dbg = PFC_SIMPLE_PREBOOST_DUTY_MAX;
  pfc_iamp_cmd = 0.0f;
  pfc_iref_a = 0.0f;
  pfc_iref_dbg = 0.0f;
  pfc_i_err_a = 0.0f;
  pfc_il_feedback_dbg = pfc_il_ctrl_avg;
  pfc_duty_ff = 0.0f;

  if (vbus_v > PFC_PFC_TEST_VBUS_LIMIT_V)
  {
    PFC_PWM_SetErrorCode(PFC_APP_ERR_PFC_TEST_VBUS);
    PFC_EnterFault(PFC_FAULT_TEST_OVP);
    return;
  }

  if (PFC_SimplePfcSoftCurrentLimit() != 0U)
  {
    return;
  }

  /* Once VBUS has enough headroom above the measured input peak, enter PFC. */
  if ((pfc_vin_peak > PFC_VIN_PEAK_MIN_V) &&
      ((pfc_vbus_avg >= (pfc_vin_peak + PFC_SIMPLE_PREBOOST_EXIT_MARGIN_V)) ||
       (pfc_vbus_avg >= PFC_SIMPLE_PREBOOST_TARGET_V)))
  {
    PFC_SimplePfcEnterRunPhase();
    return;
  }

  if ((vac_abs_v < PFC_SIMPLE_PREBOOST_MIN_VIN_V) ||
      (pfc_vin_shape < PFC_SIMPLE_PREBOOST_SHAPE_MIN))
  {
    pfc_duty_cmd = 0.0f;
    pfc_duty_pi = 0.0f;
    pfc_duty_raw_dbg = 0.0f;
    pfc_duty_slew_dbg = 0.0f;
    pfc_duty_out_dbg = 0.0f;
    if (pfc_pwm_active != 0U)
    {
      PFC_PWM_SetDuty(0.0f);
    }
    return;
  }

  allow_pwm = (vac_abs_v < (vbus_v - PFC_SIMPLE_PREBOOST_SAFE_HEADROOM_V)) ? 1U : 0U;
  if (allow_pwm == 0U)
  {
    pfc_simple_preboost_hold_dbg = 1U;
    pfc_simple_headroom_cap_dbg = 1U;
    pfc_simple_preboost_integrator = 0.0f;
    pfc_simple_preboost_duty = 0.0f;
    pfc_duty_cmd = 0.0f;
    pfc_duty_pi = 0.0f;
    pfc_duty_raw_dbg = 0.0f;
    pfc_duty_slew_dbg = 0.0f;
    pfc_duty_out_dbg = 0.0f;
    if (pfc_pwm_active != 0U)
    {
      PFC_PWM_SetDuty(0.0f);
    }
    return;
  }

  if ((HAL_GetTick() - pfc_simple_preboost_last_ms) < PFC_SIMPLE_PREBOOST_PERIOD_MS)
  {
    return;
  }
  pfc_simple_preboost_last_ms = HAL_GetTick();

  if (pfc_simple_preboost_vref < PFC_SIMPLE_PREBOOST_TARGET_V)
  {
    pfc_simple_preboost_vref += PFC_SIMPLE_PREBOOST_VREF_RAMP_STEP_V;
    if (pfc_simple_preboost_vref > PFC_SIMPLE_PREBOOST_TARGET_V)
    {
      pfc_simple_preboost_vref = PFC_SIMPLE_PREBOOST_TARGET_V;
    }
  }
  else if (pfc_simple_preboost_vref > PFC_SIMPLE_PREBOOST_TARGET_V)
  {
    pfc_simple_preboost_vref = PFC_SIMPLE_PREBOOST_TARGET_V;
  }

  /* Reuse the OLED VR field for the active preboost reference. */
  pfc_vbus_ref = pfc_simple_preboost_vref;

  error_v = pfc_simple_preboost_vref - vbus_v;
  integrator_candidate = pfc_simple_preboost_integrator + (PFC_SIMPLE_PREBOOST_KI * error_v);
  integrator_candidate = PFC_ClampFloat(integrator_candidate,
                                        0.0f,
                                        PFC_SIMPLE_PREBOOST_DUTY_MAX);

  duty_unsat = (PFC_SIMPLE_PREBOOST_KP * error_v) + integrator_candidate;
  if (!((duty_unsat > PFC_SIMPLE_PREBOOST_DUTY_MAX) && (error_v > 0.0f)) &&
      !((duty_unsat < 0.0f) && (error_v < 0.0f)))
  {
    pfc_simple_preboost_integrator = integrator_candidate;
  }
  else
  {
    pfc_iloop_freeze_dbg = 1U;
  }

  duty_cmd = (PFC_SIMPLE_PREBOOST_KP * error_v) + pfc_simple_preboost_integrator;
  duty_cmd = PFC_ClampFloat(duty_cmd, 0.0f, PFC_SIMPLE_PREBOOST_DUTY_MAX);

  duty_delta = duty_cmd - pfc_simple_preboost_duty;
  duty_delta = PFC_ClampFloat(duty_delta,
                              -PFC_SIMPLE_PREBOOST_DUTY_SLEW,
                              PFC_SIMPLE_PREBOOST_DUTY_SLEW);
  pfc_simple_preboost_duty = PFC_ClampFloat(pfc_simple_preboost_duty + duty_delta,
                                            0.0f,
                                            PFC_SIMPLE_PREBOOST_DUTY_MAX);

  pfc_duty_pi = pfc_simple_preboost_duty;
  pfc_duty_raw_dbg = pfc_simple_preboost_duty;
  pfc_duty_slew_dbg = duty_delta;
  pfc_duty_out_dbg = pfc_simple_preboost_duty;
  pfc_duty_cmd = pfc_simple_preboost_duty;
  pfc_simple_duty_cap_dbg = PFC_SIMPLE_PREBOOST_DUTY_MAX;

  if (pfc_duty_cmd <= PFC_PFC_DUTY_MIN_ACTIVE)
  {
    if (pfc_pwm_active != 0U)
    {
      PFC_PWM_SetDuty(0.0f);
    }
    return;
  }

  if (pfc_pwm_active == 0U)
  {
    PFC_PWM_StartAcRectClosedLoopAtDuty(pfc_duty_cmd);
    pfc_pwm_active = (PFC_PWM_GetErrorCode() == 0U) ? 1U : 0U;
  }
  else
  {
    PFC_PWM_SetDuty(pfc_duty_cmd);
  }

  if (PFC_PWM_GetErrorCode() != 0U)
  {
    PFC_EnterFault(PFC_FAULT_PWM);
  }
}

#else
static void PFC_PfcRunBlankOutput(void)
{
  pfc_iref_a = 0.0f;
  pfc_iref_dbg = 0.0f;
  pfc_i_err_a = 0.0f;

  pfc_duty_cmd = 0.0f;
  pfc_duty_slew_dbg = 0.0f;
  pfc_vloop_freeze_dbg = 1U;
  pfc_iloop_freeze_dbg = 1U;

  if (pfc_duty_out_dbg > PFC_PFC_RUN_BLANK_DUTY_HOLD_MAX)
  {
    pfc_duty_out_dbg = PFC_PFC_RUN_BLANK_DUTY_HOLD_MAX;
  }

  /*
   * In PFC_RUN zero-cross blanking, cap pfc_duty_out_dbg to a small hold
   * value, but do not clear it or reset the current-loop integrator.
   * Otherwise the duty slew accumulator restarts from zero every half-cycle
   * and the average output power becomes too small.
   *
   * Only force the actual PWM compare to zero during the blanking window.
   */
  if (pfc_pwm_active != 0U)
  {
    PFC_PWM_SetDuty(0.0f);
  }
}

static void PFC_PfcPrechargeBlankOutput(void)
{
  pfc_iref_a = 0.0f;
  pfc_iref_dbg = 0.0f;
  pfc_i_err_a = 0.0f;
  pfc_duty_cmd = 0.0f;
  pfc_duty_raw_dbg = 0.0f;
  pfc_duty_slew_dbg = 0.0f;
  pfc_vloop_freeze_dbg = 1U;
  pfc_iloop_freeze_dbg = 1U;

  /*
   * PRECHARGE blank:
   * only force actual PWM compare to zero.
   * Do not clear pfc_duty_out_dbg.
   * Do not clamp pfc_duty_out_dbg to PFC_RUN blank hold.
   * Do not reset current-loop integrator.
   * Do not call PFC_PWM_AllOff().
   * Do not clear pfc_pwm_active.
   */
  if (pfc_pwm_active != 0U)
  {
    PFC_PWM_SetDuty(0.0f);
  }
}

static void PFC_PfcBurstBlankOutput(void)
{
  pfc_iref_a = 0.0f;
  pfc_iref_dbg = 0.0f;
  pfc_i_err_a = 0.0f;
  pfc_duty_cmd = 0.0f;
  pfc_duty_raw_dbg = 0.0f;
  pfc_duty_slew_dbg = 0.0f;
  pfc_duty_out_dbg = 0.0f;
  pfc_iloop_integrator = 0.0f;
  pfc_duty_integrator_dbg = 0.0f;
  pfc_iloop_freeze_dbg = 1U;

  if (pfc_pwm_active != 0U)
  {
    PFC_PWM_SetDuty(0.0f);
  }
}
#endif

#if (PFC_RUN_ENABLE_DEFAULT != 0)
#if (PFC_TEST_PROFILE_18VAC_SIMPLE_PFC == 0)
static uint8_t PFC_PfcRunSoftCurrentLimit(void)
{
#if (PFC_PFC_RUN_IL_SOFT_LIMIT_ENABLE != 0)
  if ((pfc_state == PFC_STATE_PFC_RUN) && (il_a > PFC_PFC_RUN_IL_SOFT_LIMIT_A))
  {
    pfc_pre_soft_limit_dbg = 1U;
    if (pfc_duty_out_dbg > PFC_PFC_RUN_IL_SOFT_DUTY_STEP)
    {
      pfc_duty_out_dbg -= PFC_PFC_RUN_IL_SOFT_DUTY_STEP;
    }
    else
    {
      pfc_duty_out_dbg = 0.0f;
    }

    pfc_duty_cmd = 0.0f;
    pfc_duty_slew_dbg = 0.0f;
    pfc_iloop_freeze_dbg = 1U;

    if (pfc_pwm_active != 0U)
    {
      PFC_PWM_SetDuty(0.0f);
    }

    return 1U;
  }
#endif

  return 0U;
}

static void PFC_PfcPrechargeTask(void)
{
  float iref;
  float iamp_eff;

  if (pfc_state != PFC_STATE_PFC_RUN)
  {
    return;
  }

  if (vbus_v > PFC_PFC_TEST_VBUS_LIMIT_V)
  {
    PFC_PWM_SetErrorCode(PFC_APP_ERR_PFC_TEST_VBUS);
    PFC_EnterFault(PFC_FAULT_TEST_OVP);
    return;
  }

  if ((vac_abs_v < PFC_PFC_PRECHARGE_VIN_MIN_V) ||
      (pfc_vin_shape < PFC_PFC_PRECHARGE_SHAPE_MIN))
  {
    pfc_pre_zero_blank_dbg = 1U;
    PFC_PfcPrechargeBlankOutput();
    return;
  }

  if (vbus_v < (vac_abs_v + PFC_PFC_RUN_BOOST_MARGIN_V))
  {
    pfc_run_margin_blank_dbg = 1U;
    PFC_PfcPrechargeBlankOutput();
    return;
  }

  if (il_a > PFC_PFC_PRECHARGE_IL_SOFT_LIMIT_A)
  {
    pfc_pre_soft_limit_dbg = 1U;
    if (pfc_duty_out_dbg > PFC_PFC_PRECHARGE_IL_SOFT_DUTY_STEP)
    {
      pfc_duty_out_dbg -= PFC_PFC_PRECHARGE_IL_SOFT_DUTY_STEP;
    }
    else
    {
      pfc_duty_out_dbg = 0.0f;
    }

    pfc_duty_cmd = 0.0f;
    pfc_duty_slew_dbg = 0.0f;
    pfc_iloop_freeze_dbg = 1U;
    pfc_precharge_duty = pfc_duty_out_dbg;
    if (pfc_pwm_active != 0U)
    {
      PFC_PWM_SetDuty(0.0f);
    }
    return;
  }

#if (PFC_PFC_SOFTSTART_USE_VLOOP != 0)
  PFC_UpdatePfcVoltageLoopLimited(PFC_PFC_SOFTSTART_IAMP_MAX_A,
                                  PFC_PFC_SOFTSTART_IAMP_SLEW_A,
                                  0U);
  iamp_eff = PFC_ApplyPfcVinFeedForward(pfc_iamp_cmd);
  iamp_eff = PFC_ClampFloat(iamp_eff, 0.0f, PFC_PFC_SOFTSTART_IAMP_MAX_A);
  pfc_precharge_iref_cmd = iamp_eff;

  if (iamp_eff < PFC_PFC_SOFTSTART_MIN_IAMP_A)
  {
    pfc_burst_blank_dbg = 1U;
    PFC_PfcBurstBlankOutput();
    return;
  }

  iref = iamp_eff * pfc_vin_shape;
  iref = PFC_ClampFloat(iref, 0.0f, PFC_PFC_SOFTSTART_IAMP_MAX_A);
#else
  if (pfc_precharge_iref_cmd < PFC_PFC_PRECHARGE_IREF_TARGET_A)
  {
    pfc_precharge_iref_cmd += PFC_PFC_PRECHARGE_IREF_STEP_A;
    if (pfc_precharge_iref_cmd > PFC_PFC_PRECHARGE_IREF_TARGET_A)
    {
      pfc_precharge_iref_cmd = PFC_PFC_PRECHARGE_IREF_TARGET_A;
    }
  }

  iref = pfc_precharge_iref_cmd * pfc_vin_shape;
  iref = PFC_ClampFloat(iref, 0.0f, PFC_PFC_PRECHARGE_IREF_TARGET_A);
#endif

  PFC_CurrentLoop_Task(iref, 0U);

  pfc_precharge_duty = pfc_duty_out_dbg;

  if (PFC_PWM_GetErrorCode() != 0U)
  {
    PFC_EnterFault(PFC_FAULT_PWM);
    return;
  }

  if ((pfc_vbus_avg >= (PFC_PFC_PRECHARGE_TARGET_V - PFC_PFC_PRECHARGE_EXIT_MARGIN_V)) ||
      (vbus_v >= PFC_PFC_PRECHARGE_TARGET_V))
  {
    pfc_run_phase = PFC_RUN_PHASE_SHAPE;
    pfc_vbus_ref = PFC_ClampFloat(pfc_vbus_avg, PFC_VBUS_REF_INIT_V, PFC_VBUS_REF_TARGET_V);
    pfc_iamp_cmd = 0.0f;
    pfc_iref_a = 0.0f;
    pfc_iref_dbg = 0.0f;
    pfc_i_err_a = 0.0f;
    pfc_vloop_integrator = 0.0f;
    pfc_iloop_integrator = 0.0f;
    pfc_duty_ff = 0.0f;
    pfc_duty_pi = 0.0f;
    pfc_duty_out_dbg = pfc_precharge_duty;
    pfc_duty_raw_dbg = pfc_precharge_duty;
    pfc_duty_cmd = pfc_precharge_duty;
    pfc_vloop_last_ms = HAL_GetTick();
    pfc_current_loop_last_us = PFC_GetMicros();
  }
}
#endif
#endif

#if (PFC_TEST_PROFILE_18VAC_SIMPLE_PFC != 0)
static void PFC_SimplePfcTask(void)
{
  float iamp_eff;
  float iref;

  pfc_simple_no_demand_dbg = 0U;
  pfc_simple_headroom_v_dbg = vbus_v - vac_abs_v;
  pfc_simple_duty_cap_dbg = pfc_simple_duty_limit;
  pfc_simple_headroom_cap_dbg = 0U;
  pfc_run_margin_blank_dbg = 0U;

  if (vbus_v > PFC_PFC_TEST_VBUS_LIMIT_V)
  {
    PFC_PWM_SetErrorCode(PFC_APP_ERR_PFC_TEST_VBUS);
    PFC_EnterFault(PFC_FAULT_TEST_OVP);
    return;
  }

  if ((vac_abs_v < 2.0f) || (pfc_vin_shape < 0.03f))
  {
    PFC_SimplePfcBlankOutput();
    return;
  }

  if (vbus_v < (vac_abs_v + PFC_PFC_RUN_BOOST_MARGIN_V))
  {
    pfc_run_margin_blank_dbg = 1U;
    PFC_SimplePfcBlankOutput();
    return;
  }

  PFC_UpdatePfcVoltageLoop();
  iamp_eff = PFC_ApplyPfcVinFeedForward(pfc_iamp_cmd);
  iamp_eff = PFC_ClampFloat(iamp_eff, PFC_IAMP_CMD_MIN_A, PFC_IAMP_CMD_MAX_A);

  iref = iamp_eff * pfc_vin_shape;
  iref = PFC_ClampFloat(iref, 0.0f, PFC_IREF_MAX_A);
  pfc_iref_a = iref;
  pfc_iref_dbg = iref;

  if (PFC_SimplePfcSoftCurrentLimit() != 0U)
  {
    return;
  }

  if ((pfc_iamp_cmd <= PFC_SIMPLE_MIN_IAMP_A) ||
      (pfc_iref_a <= PFC_SIMPLE_MIN_IREF_A) ||
      (pfc_vbus_avg >= (pfc_vbus_ref + PFC_SIMPLE_VBUS_HYST_V)))
  {
    pfc_simple_no_demand_dbg = 1U;
    PFC_SimplePfcNoDemandBlankOutput();
    return;
  }

#if (PFC_CURRENT_LOOP_IN_ISR_ENABLE != 0)
  (void)PFC_HandleIsrFaultRequest();
#else
  PFC_CurrentLoop_Task(iref, 0U);
#endif
}
#endif

static void PFC_CurrentLoop_Task(float i_ref, uint8_t freeze_integrator)
{
  float vin;
  float vbus;
  float il_raw_feedback;
  float i_err;
  float iloop_ki;
  float iloop_kp;
  float integrator_candidate;
  float duty_unsat;
  float duty_raw;
  float duty_max;
  float duty_aw_max;
  uint32_t now_us;
#if (PFC_I_LOOP_LARGE_ERR_ACCEL_ENABLE != 0)
  uint8_t simple_pfc_active = 0U;
#endif

  now_us = PFC_GetMicros();
  if ((now_us - pfc_current_loop_last_us) < PFC_CURRENT_LOOP_PERIOD_US)
  {
    return;
  }
  pfc_current_loop_last_us = now_us;
  pfc_current_loop_update_count++;

  pfc_iref_a = PFC_ClampFloat(i_ref, 0.0f, PFC_IREF_MAX_A);
  vin = vac_abs_v;
  vbus = vbus_v;
  duty_max = PFC_GetCurrentLoopDutyMax();
  duty_aw_max = duty_max;

  if ((vin < PFC_VIN_ZERO_TH_V) || (vbus < 1.0f))
  {
    if (pfc_state == PFC_STATE_PFC_RUN)
    {
#if (PFC_TEST_PROFILE_18VAC_SIMPLE_PFC != 0)
      PFC_SimplePfcBlankOutput();
#else
      PFC_PfcRunBlankOutput();
#endif
      return;
    }

    pfc_duty_ff = 0.0f;
    pfc_duty_pi = 0.0f;
    pfc_duty_cmd = 0.0f;
    pfc_duty_raw_dbg = 0.0f;
    pfc_duty_slew_dbg = 0.0f;
    pfc_duty_out_dbg = 0.0f;
    pfc_i_accel_active = 0U;
    PFC_PWM_AllOff();
    pfc_pwm_active = 0U;
    return;
  }

  il_raw_feedback = il_a;
#if (PFC_USE_SYNC_IL_FOR_CURRENT_LOOP != 0)
  /*
   * Keep this path for later. For the low-speed average-current debug build,
   * synchronized IL is displayed only and must not control the current loop.
   */
  if (PFC_IsIlSyncFresh() != 0U)
  {
    il_raw_feedback = il_sync_a;
  }
#endif

  pfc_il_ctrl_avg += PFC_IL_CTRL_AVG_ALPHA_EFFECTIVE * (il_raw_feedback - pfc_il_ctrl_avg);
  pfc_il_feedback_used = pfc_il_ctrl_avg;

  i_err = pfc_iref_a - pfc_il_feedback_used;
  pfc_i_err_a = i_err;
  pfc_il_feedback_dbg = pfc_il_feedback_used;
  pfc_iref_dbg = pfc_iref_a;

  if ((freeze_integrator != 0U) && (pfc_iref_a <= 0.0001f))
  {
    pfc_duty_ff = 0.0f;
    pfc_duty_pi = 0.0f;
    pfc_duty_cmd = 0.0f;
    pfc_duty_raw_dbg = 0.0f;
    pfc_duty_slew_dbg = 0.0f;
    pfc_duty_out_dbg = 0.0f;
    pfc_i_accel_active = 0U;
    PFC_PWM_AllOff();
    pfc_pwm_active = 0U;
    return;
  }

  if (pfc_state == PFC_STATE_DC_CURRENT_TEST)
  {
#if (PFC_DC_I_USE_DYNAMIC_FF != 0)
    pfc_duty_ff = 1.0f - (vin / vbus);
#else
    pfc_duty_ff = PFC_DC_I_DUTY_BIAS_EFFECTIVE;
#endif
  }
  else if (pfc_state == PFC_STATE_AC_CURRENT_SHAPE_TEST)
  {
#if (PFC_AC_I_DFF_ENABLE != 0)
    pfc_duty_ff = 1.0f - (vin / vbus);
    pfc_duty_ff = PFC_ClampFloat(pfc_duty_ff, 0.0f, PFC_AC_I_DUTY_FF_MAX);
#else
    pfc_duty_ff = 0.0f;
#endif
  }
  else
  {
#if (PFC_TEST_PROFILE_18VAC_SIMPLE_PFC != 0)
    if (pfc_state == PFC_STATE_PFC_RUN)
    {
#if (PFC_SIMPLE_DFF_ENABLE != 0)
      pfc_duty_ff = 1.0f - (vin / vbus);
      pfc_duty_ff = PFC_ClampFloat(pfc_duty_ff, 0.0f, PFC_SIMPLE_DFF_MAX);
#else
      pfc_duty_ff = 0.0f;
#endif
    }
    else
    {
      pfc_duty_ff = 1.0f - (vin / vbus);
    }
#else
    pfc_duty_ff = 1.0f - (vin / vbus);
#endif
  }
  if (pfc_state == PFC_STATE_PFC_RUN)
  {
    pfc_duty_ff = PFC_ClampFloat(pfc_duty_ff, 0.0f, PFC_PFC_RUN_DUTY_MAX);
  }
  else
  {
    pfc_duty_ff = PFC_ClampFloat(pfc_duty_ff, 0.0f, PFC_PFC_DUTY_MAX);
  }

  iloop_kp = PFC_ILOOP_KP_EFFECTIVE;
  iloop_ki = PFC_ILOOP_KI_EFFECTIVE;
#if (PFC_TEST_PROFILE_18VAC_SIMPLE_PFC != 0)
  if (pfc_state == PFC_STATE_PFC_RUN)
  {
    float simple_limit_target = PFC_SIMPLE_DUTY_LIMIT_TARGET;

#if (PFC_I_LOOP_LARGE_ERR_ACCEL_ENABLE != 0)
    simple_pfc_active = 1U;
#endif
    iloop_kp = PFC_ILOOP_KP_SIMPLE;
    iloop_ki = PFC_ILOOP_KI_SIMPLE;

    if (simple_limit_target > PFC_PFC_RUN_DUTY_MAX)
    {
      simple_limit_target = PFC_PFC_RUN_DUTY_MAX;
    }

    pfc_simple_duty_limit += PFC_SIMPLE_DUTY_LIMIT_STEP;
    pfc_simple_duty_limit = PFC_ClampFloat(pfc_simple_duty_limit,
                                           PFC_SIMPLE_DUTY_LIMIT_INIT,
                                           simple_limit_target);
    duty_aw_max = PFC_SimplePfcGetHeadroomDutyCap();
  }
#endif
  pfc_i_accel_active = 0U;
#if (PFC_I_LOOP_LARGE_ERR_ACCEL_ENABLE != 0)
  if ((simple_pfc_active == 0U) &&
      ((i_err > PFC_I_LOOP_LARGE_ERR_TH_A) || (i_err < -PFC_I_LOOP_LARGE_ERR_TH_A)))
  {
    iloop_ki *= PFC_I_LOOP_ACCEL_KI_MULT;
    pfc_i_accel_active = 1U;
  }
#endif

  integrator_candidate = pfc_iloop_integrator;
  if (freeze_integrator == 0U)
  {
    integrator_candidate += iloop_ki * i_err;
    integrator_candidate = PFC_ClampFloat(integrator_candidate, -0.20f, 0.20f);

    pfc_duty_pi = (iloop_kp * i_err) + integrator_candidate;
    duty_unsat = pfc_duty_ff + pfc_duty_pi;
    if (!((duty_unsat > duty_aw_max) && (i_err > 0.0f)) &&
        !((duty_unsat < PFC_PFC_DUTY_MIN) && (i_err < 0.0f)))
    {
      pfc_iloop_integrator = integrator_candidate;
    }
    else
    {
      pfc_iloop_freeze_dbg = 1U;
    }
  }
  else
  {
    pfc_iloop_freeze_dbg = 1U;
  }

  pfc_duty_pi = (iloop_kp * i_err) + pfc_iloop_integrator;
  pfc_duty_integrator_dbg = pfc_iloop_integrator;
  duty_raw = pfc_duty_ff + pfc_duty_pi;
  pfc_duty_raw_dbg = PFC_ClampFloat(duty_raw, PFC_PFC_DUTY_MIN, duty_max);
  if ((pfc_state == PFC_STATE_PFC_RUN) && (pfc_duty_raw_dbg != duty_raw))
  {
    pfc_pre_duty_clamp_dbg = 1U;
  }

  /* Keep PWM fully off until the integrator has built a non-zero active duty. */
  if (pfc_duty_raw_dbg <= PFC_PFC_DUTY_MIN_ACTIVE)
  {
    pfc_duty_cmd = 0.0f;
    pfc_duty_slew_dbg = 0.0f;
    pfc_duty_out_dbg = 0.0f;
    PFC_PWM_AllOff();
    pfc_pwm_active = 0U;
    return;
  }

  pfc_duty_cmd = PFC_ApplyCurrentLoopDutySlew(pfc_duty_raw_dbg);
  if (pfc_duty_cmd <= PFC_PFC_DUTY_MIN_ACTIVE)
  {
    PFC_PWM_AllOff();
    pfc_pwm_active = 0U;
    return;
  }

  if (pfc_pwm_active == 0U)
  {
    PFC_PWM_StartAcRectClosedLoopAtDuty(pfc_duty_cmd);
    pfc_pwm_active = (PFC_PWM_GetErrorCode() == 0U) ? 1U : 0U;
  }
  else
  {
    PFC_PWM_SetDuty(pfc_duty_cmd);
  }

  if (PFC_PWM_GetErrorCode() != 0U)
  {
    PFC_EnterFault(PFC_FAULT_PWM);
  }
}

static void PFC_IsrBlankDuty(void)
{
  if (pfc_state == PFC_STATE_AC_CURRENT_SHAPE_TEST)
  {
    pfc_iref_a = 0.0f;
    pfc_iref_dbg = 0.0f;
    pfc_i_err_a = 0.0f;
    pfc_duty_ff = 0.0f;
    pfc_duty_pi = 0.0f;
    pfc_duty_raw_dbg = 0.0f;
    pfc_duty_slew_dbg = 0.0f;
    pfc_duty_out_dbg = 0.0f;
    pfc_duty_cmd = 0.0f;
    pfc_iloop_integrator = 0.0f;
    pfc_duty_integrator_dbg = 0.0f;
    pfc_iloop_freeze_dbg = 1U;
    PFC_PWM_AllOff();
    pfc_pwm_active = 0U;
    return;
  }

  pfc_iref_a = 0.0f;
  pfc_iref_dbg = 0.0f;
  pfc_i_err_a = 0.0f;
  pfc_duty_ff = 0.0f;
  pfc_duty_pi = 0.0f;
  pfc_duty_raw_dbg = 0.0f;
  pfc_duty_slew_dbg = 0.0f;
  pfc_duty_out_dbg = 0.0f;
  pfc_duty_cmd = 0.0f;
  pfc_iloop_freeze_dbg = 1U;

  if (pfc_pwm_active != 0U)
  {
    PFC_PWM_SetDuty(0.0f);
  }
}

static void PFC_CurrentLoop_Core_Isr(float i_ref, float il_feedback, float duty_max)
{
  float i_err;
  float iloop_kp = PFC_ILOOP_KP_FAST;
  float iloop_ki = PFC_ILOOP_KI_FAST;
  float il_alpha = PFC_IL_CTRL_AVG_ALPHA_FAST;
  float integrator_candidate;
  float duty_unsat;
  float duty_raw;
  float duty_delta;
  float duty_limited;
  float duty_aw_max;

  duty_max = PFC_ClampFloat(duty_max, 0.0f, PFC_DUTY_OPENLOOP_MAX);

#if (PFC_TEST_PROFILE_18VAC_SIMPLE_PFC != 0)
  if (pfc_state == PFC_STATE_PFC_RUN)
  {
    float simple_limit_target = PFC_SIMPLE_DUTY_LIMIT_TARGET;

    iloop_kp = PFC_ILOOP_KP_SIMPLE;
    iloop_ki = PFC_ILOOP_KI_SIMPLE;
    il_alpha = PFC_IL_CTRL_AVG_ALPHA_SIMPLE;

    if (simple_limit_target > PFC_PFC_RUN_DUTY_MAX)
    {
      simple_limit_target = PFC_PFC_RUN_DUTY_MAX;
    }

    pfc_simple_duty_limit += PFC_SIMPLE_DUTY_LIMIT_STEP;
    pfc_simple_duty_limit = PFC_ClampFloat(pfc_simple_duty_limit,
                                           PFC_SIMPLE_DUTY_LIMIT_INIT,
                                           simple_limit_target);
    duty_max = PFC_ClampFloat(PFC_SimplePfcGetHeadroomDutyCap(), 0.0f, duty_max);
  }
#endif

  duty_aw_max = duty_max;
  pfc_i_accel_active = 0U;
  pfc_iref_a = PFC_ClampFloat(i_ref, 0.0f, PFC_IREF_MAX_A);
  pfc_il_ctrl_avg += il_alpha * (il_feedback - pfc_il_ctrl_avg);
  pfc_il_feedback_used = pfc_il_ctrl_avg;
  pfc_il_feedback_dbg = pfc_il_feedback_used;
  pfc_i_err_a = pfc_iref_a - pfc_il_feedback_dbg;
  pfc_iref_dbg = pfc_iref_a;
  i_err = pfc_i_err_a;

  if (pfc_state == PFC_STATE_AC_CURRENT_SHAPE_TEST)
  {
#if (PFC_AC_I_DFF_ENABLE != 0)
    if (vbus_v > 1.0f)
    {
      pfc_duty_ff = 1.0f - (vac_abs_v / vbus_v);
    }
    else
    {
      pfc_duty_ff = 0.0f;
    }
    pfc_duty_ff = PFC_ClampFloat(pfc_duty_ff, 0.0f, PFC_AC_I_DUTY_FF_MAX);
#else
    pfc_duty_ff = 0.0f;
#endif
  }
  else if (pfc_state == PFC_STATE_PFC_RUN)
  {
#if (PFC_TEST_PROFILE_18VAC_SIMPLE_PFC != 0)
#if (PFC_SIMPLE_DFF_ENABLE != 0)
    if (vbus_v > 1.0f)
    {
      pfc_duty_ff = 1.0f - (vac_abs_v / vbus_v);
    }
    else
    {
      pfc_duty_ff = 0.0f;
    }
    pfc_duty_ff = PFC_ClampFloat(pfc_duty_ff, 0.0f, PFC_SIMPLE_DFF_MAX);
#else
    pfc_duty_ff = 0.0f;
#endif
#else
    if (vbus_v > 1.0f)
    {
      pfc_duty_ff = 1.0f - (vac_abs_v / vbus_v);
    }
    else
    {
      pfc_duty_ff = 0.0f;
    }
    pfc_duty_ff = PFC_ClampFloat(pfc_duty_ff, 0.0f, PFC_PFC_RUN_DUTY_MAX);
#endif
  }
  else
  {
    if (vbus_v > 1.0f)
    {
      pfc_duty_ff = 1.0f - (vac_abs_v / vbus_v);
    }
    else
    {
      pfc_duty_ff = 0.0f;
    }
    pfc_duty_ff = PFC_ClampFloat(pfc_duty_ff, 0.0f, PFC_PFC_DUTY_MAX);
  }

  integrator_candidate = pfc_iloop_integrator + (iloop_ki * i_err);
  integrator_candidate = PFC_ClampFloat(integrator_candidate, -0.20f, 0.20f);

  pfc_duty_pi = (iloop_kp * i_err) + integrator_candidate;
  duty_unsat = pfc_duty_ff + pfc_duty_pi;
  if (!((duty_unsat > duty_aw_max) && (i_err > 0.0f)) &&
      !((duty_unsat < PFC_PFC_DUTY_MIN) && (i_err < 0.0f)))
  {
    pfc_iloop_integrator = integrator_candidate;
  }
  else
  {
    pfc_iloop_freeze_dbg = 1U;
  }

  pfc_duty_pi = (iloop_kp * i_err) + pfc_iloop_integrator;
  pfc_duty_integrator_dbg = pfc_iloop_integrator;
  duty_raw = pfc_duty_ff + pfc_duty_pi;
  pfc_duty_raw_dbg = PFC_ClampFloat(duty_raw, PFC_PFC_DUTY_MIN, duty_max);
  if ((pfc_state == PFC_STATE_PFC_RUN) && (pfc_duty_raw_dbg != duty_raw))
  {
    pfc_pre_duty_clamp_dbg = 1U;
  }

  duty_delta = pfc_duty_raw_dbg - pfc_duty_out_dbg;
  if (duty_delta > PFC_I_DUTY_SLEW_UP_PER_TICK)
  {
    duty_delta = PFC_I_DUTY_SLEW_UP_PER_TICK;
  }
  else if (duty_delta < -PFC_I_DUTY_SLEW_DOWN_PER_TICK)
  {
    duty_delta = -PFC_I_DUTY_SLEW_DOWN_PER_TICK;
  }

  duty_limited = PFC_ClampFloat(pfc_duty_out_dbg + duty_delta, 0.0f, duty_max);
  pfc_duty_slew_dbg = duty_limited - pfc_duty_out_dbg;
  pfc_duty_out_dbg = duty_limited;
  pfc_duty_cmd = duty_limited;

  if (pfc_duty_cmd <= PFC_PFC_DUTY_MIN_ACTIVE)
  {
    if (pfc_pwm_active != 0U)
    {
      PFC_PWM_AllOff();
      pfc_pwm_active = 0U;
    }
    pfc_current_loop_update_count++;
    pfc_isr_loop_count++;
    return;
  }

  if (pfc_state == PFC_STATE_AC_CURRENT_SHAPE_TEST)
  {
    if ((vbus_v <= (vac_abs_v + PFC_AC_I_BOOST_MARGIN_V)) ||
        (pfc_vin_shape <= PFC_AC_I_SHAPE_MIN) ||
        (pfc_iref_a <= PFC_AC_I_IREF_MIN_A))
    {
      PFC_IsrBlankDuty();
      return;
    }
  }

  if (pfc_pwm_active == 0U)
  {
    PFC_PWM_StartAcRectClosedLoopAtDuty(pfc_duty_cmd);
    pfc_pwm_active = (PFC_PWM_GetErrorCode() == 0U) ? 1U : 0U;
  }
  else
  {
    PFC_PWM_SetDuty(pfc_duty_cmd);
  }

  pfc_current_loop_update_count++;
  pfc_isr_loop_count++;
}

void PFC_App_OnInjectedCurrentIsr(void)
{
#if (PFC_CURRENT_LOOP_IN_ISR_ENABLE != 0)
  float iref;
  float duty_max;
  uint8_t is_ac_i = 0U;
  uint8_t is_pfc_shape = 0U;

  if (pfc_state == PFC_STATE_OPENLOOP_2PCT_TEST)
  {
    openloop_last_is = il_sync_a;
    openloop_last_il = il_a;
    if (il_sync_a > openloop_pk_is)
    {
      openloop_pk_is = il_sync_a;
    }
    if (il_a > openloop_pk_il)
    {
      openloop_pk_il = il_a;
    }
    openloop_isr_count++;

#if (PFC_OPENLOOP_2PCT_OCP_ENABLE != 0)
    if (il_sync_a > PFC_OPENLOOP_2PCT_OCP_A)
    {
      if (openloop_ocp_cnt < PFC_OPENLOOP_2PCT_OCP_TRIP_COUNT)
      {
        openloop_ocp_cnt++;
      }
    }
    else if (il_sync_a < PFC_OPENLOOP_2PCT_OCP_RELEASE_A)
    {
      openloop_ocp_cnt = 0U;
    }

    if (openloop_ocp_cnt >= PFC_OPENLOOP_2PCT_OCP_TRIP_COUNT)
    {
      PFC_PWM_SetDuty(0.0f);
      pfc_isr_fault_request = (uint8_t)PFC_FAULT_OCP;
    }
#endif
    return;
  }

  if ((pfc_state == PFC_STATE_PFC_RUN) && (pfc_run_phase == PFC_RUN_PHASE_PRECHARGE))
  {
    pfc_isr_control_enabled_dbg = 0U;
    pfc_isr_headroom_ok_dbg = 0U;

#if (PFC_TEST_PROFILE_18VAC_SIMPLE_PFC != 0)
    if (PFC_SimplePfcSoftCurrentLimit() != 0U)
    {
      return;
    }
#endif

    if (il_sync_a > fault_peak_isr_il_sync_a)
    {
      fault_peak_isr_il_sync_a = il_sync_a;
      fault_peak_isr_il_dma_a = il_a;
      fault_peak_isr_duty = pfc_duty_cmd;
      fault_peak_isr_cnt = pfc_isr_ilimit_cnt;
    }

    if (il_sync_a > PFC_ISR_ILIMIT_A)
    {
      if (pfc_isr_ilimit_cnt < PFC_ISR_ILIMIT_TRIP_COUNT)
      {
        pfc_isr_ilimit_cnt++;
      }
      fault_peak_isr_cnt = pfc_isr_ilimit_cnt;
    }
    else if (il_sync_a < PFC_ISR_ILIMIT_RELEASE_A)
    {
      pfc_isr_ilimit_cnt = 0U;
    }

    if (pfc_isr_ilimit_cnt >= PFC_ISR_ILIMIT_TRIP_COUNT)
    {
      PFC_IsrBlankDuty();
      pfc_isr_fault_request = (uint8_t)PFC_FAULT_OCP;
    }
    return;
  }

  pfc_isr_inj_div_cnt++;
  if (pfc_isr_inj_div_cnt < PFC_CURRENT_LOOP_ISR_DECIMATION)
  {
    return;
  }
  pfc_isr_inj_div_cnt = 0U;

  if (pfc_state == PFC_STATE_AC_CURRENT_SHAPE_TEST)
  {
    is_ac_i = 1U;
  }
  else if ((pfc_state == PFC_STATE_PFC_RUN) && (pfc_run_phase == PFC_RUN_PHASE_SHAPE))
  {
    is_pfc_shape = 1U;
  }
  else
  {
    pfc_isr_control_enabled_dbg = 0U;
    pfc_isr_ilimit_cnt = 0U;
    return;
  }

  if (pfc_pwm_active == 0U)
  {
    pfc_isr_control_enabled_dbg = 0U;
    return;
  }

  pfc_isr_control_enabled_dbg = 1U;
  pfc_isr_headroom_ok_dbg = 0U;

  if (vbus_v > ((is_ac_i != 0U) ? PFC_AC_I_VBUS_LIMIT_V : PFC_PFC_TEST_VBUS_LIMIT_V))
  {
    PFC_IsrBlankDuty();
    pfc_isr_fault_request = (uint8_t)PFC_FAULT_TEST_OVP;
    return;
  }

  if ((vac_abs_v <= PFC_AC_I_VIN_DISABLE_TH_V) ||
      (pfc_vin_shape <= PFC_AC_I_SHAPE_MIN))
  {
    PFC_IsrBlankDuty();
    pfc_isr_ilimit_cnt = 0U;
    return;
  }

#if (PFC_AC_I_BOOST_MARGIN_ENABLE != 0)
  if (vbus_v <= (vac_abs_v + PFC_AC_I_BOOST_MARGIN_V))
  {
    if (is_ac_i != 0U)
    {
      pfc_ac_i_margin_blank_dbg = 1U;
    }
    PFC_IsrBlankDuty();
    return;
  }
#endif
  pfc_isr_headroom_ok_dbg = 1U;

#if (PFC_TEST_PROFILE_18VAC_SIMPLE_PFC != 0)
  if ((is_pfc_shape != 0U) && (PFC_SimplePfcSoftCurrentLimit() != 0U))
  {
    return;
  }
#endif

  if (il_sync_a > PFC_ISR_ILIMIT_A)
  {
    if (pfc_isr_ilimit_cnt < PFC_ISR_ILIMIT_TRIP_COUNT)
    {
      pfc_isr_ilimit_cnt++;
    }
    if (il_sync_a > fault_peak_isr_il_sync_a)
    {
      fault_peak_isr_il_sync_a = il_sync_a;
      fault_peak_isr_il_dma_a = il_a;
      fault_peak_isr_duty = pfc_duty_cmd;
    }
    fault_peak_isr_cnt = pfc_isr_ilimit_cnt;
  }
  else if (il_sync_a < PFC_ISR_ILIMIT_RELEASE_A)
  {
    pfc_isr_ilimit_cnt = 0U;
  }

  if (pfc_isr_ilimit_cnt >= PFC_ISR_ILIMIT_TRIP_COUNT)
  {
    PFC_IsrBlankDuty();
    pfc_isr_fault_request = (uint8_t)PFC_FAULT_OCP;
    return;
  }

  if (is_ac_i != 0U)
  {
    const float derate = PFC_AcIGetDerateScale();
    const float iamp_target = PFC_ClampFloat(PFC_AC_I_IAMP_TARGET_A * derate,
                                             0.0f,
                                             PFC_AC_I_IAMP_MAX_A);
    float iamp_delta;

    iamp_delta = iamp_target - pfc_iamp_cmd;
    iamp_delta = PFC_ClampFloat(iamp_delta,
                                -PFC_AC_I_IAMP_SLEW_A,
                                PFC_AC_I_IAMP_SLEW_A);
    pfc_iamp_cmd = PFC_ClampFloat(pfc_iamp_cmd + iamp_delta,
                                  0.0f,
                                  PFC_AC_I_IAMP_MAX_A);
    iref = pfc_iamp_cmd * pfc_vin_shape;
    duty_max = PFC_AC_I_DUTY_MAX;
  }
  else if (is_pfc_shape != 0U)
  {
    const float iamp_eff = PFC_ClampFloat(PFC_ApplyPfcVinFeedForward(pfc_iamp_cmd),
                                          0.0f,
                                          PFC_IAMP_CMD_MAX_A);

    iref = iamp_eff * pfc_vin_shape;
#if (PFC_TEST_PROFILE_18VAC_SIMPLE_PFC != 0)
    if ((iamp_eff <= PFC_SIMPLE_MIN_IAMP_A) || (iref <= PFC_SIMPLE_MIN_IREF_A))
    {
      PFC_IsrBlankDuty();
      return;
    }
#else
    if (iamp_eff < PFC_PFC_SOFTSTART_MIN_IAMP_A)
    {
      PFC_IsrBlankDuty();
      return;
    }
#endif
    duty_max = PFC_PFC_RUN_DUTY_MAX;
  }
  else
  {
    return;
  }

  iref = PFC_ClampFloat(iref, 0.0f, PFC_IREF_MAX_A);
  if (iref <= ((is_ac_i != 0U) ? PFC_AC_I_IREF_MIN_A : 0.0001f))
  {
    PFC_IsrBlankDuty();
    return;
  }

  PFC_CurrentLoop_Core_Isr(iref, il_sync_a, duty_max);
#endif
}

static uint8_t PFC_HandleIsrFaultRequest(void)
{
  const uint8_t fault_request = pfc_isr_fault_request;

  if (fault_request == (uint8_t)PFC_FAULT_NONE)
  {
    return 0U;
  }

  pfc_isr_fault_request = (uint8_t)PFC_FAULT_NONE;

  if (fault_request == (uint8_t)PFC_FAULT_OCP)
  {
    PFC_EnterFault(PFC_FAULT_OCP);
    return 1U;
  }

  if (fault_request == (uint8_t)PFC_FAULT_TEST_OVP)
  {
    PFC_PWM_SetErrorCode(PFC_APP_ERR_PFC_TEST_VBUS);
    PFC_EnterFault(PFC_FAULT_TEST_OVP);
    return 1U;
  }

  return 0U;
}

static void PFC_OpenLoop2PctTask(void)
{
#if (PFC_OPENLOOP_2PCT_TEST_ENABLE != 0)
  const float fixed_duty = PFC_OpenLoop2PctDuty();

  if (pfc_state != PFC_STATE_OPENLOOP_2PCT_TEST)
  {
    return;
  }

  PFC_OpenLoop2PctApplyDebug(fixed_duty);

  if (vbus_v > PFC_OPENLOOP_2PCT_VBUS_LIMIT_V)
  {
    PFC_PWM_SetErrorCode(PFC_APP_ERR_PFC_TEST_VBUS);
    PFC_EnterFault(PFC_FAULT_TEST_OVP);
    return;
  }

  if (PFC_PWM_GetErrorCode() != 0U)
  {
    PFC_EnterFault(PFC_FAULT_PWM);
    return;
  }

  if (pfc_pwm_active == 0U)
  {
    PFC_PWM_StartAcRectClosedLoopAtDuty(fixed_duty);
    if (PFC_PWM_GetErrorCode() != 0U)
    {
      PFC_EnterFault(PFC_FAULT_PWM);
      return;
    }
    pfc_pwm_active = 1U;
  }
  else
  {
    PFC_PWM_SetDuty(fixed_duty);
    if (PFC_PWM_GetErrorCode() != 0U)
    {
      PFC_EnterFault(PFC_FAULT_PWM);
      return;
    }
  }
#else
  (void)PFC_OpenLoop2PctDuty;
#endif
}

static void PFC_PfcTask(void)
{
#if (PFC_PFC_ENABLE != 0)
  if (PFC_IsPfcTestState() == 0U)
  {
    return;
  }

  PFC_UpdatePfcMeasurements();

  switch (pfc_state)
  {
    case PFC_STATE_DC_CURRENT_TEST:
      pfc_iamp_cmd = 0.0f;
      PFC_CurrentLoop_Task(PFC_DC_CURRENT_TEST_REF_A, 0U);
      break;

    case PFC_STATE_AC_CURRENT_SHAPE_TEST:
      pfc_ac_i_margin_blank_dbg = 0U;

#if (PFC_CURRENT_LOOP_IN_ISR_ENABLE != 0) && (PFC_18VAC_FAST_ILOOP_TEST_ENABLE != 0)
    {
      const float start_duty = PFC_ClampFloat(PFC_AC_I_START_DUTY,
                                              0.0f,
                                              PFC_AC_I_START_DUTY_MAX);
      uint8_t current_ok = 1U;

      if (PFC_HandleIsrFaultRequest() != 0U)
      {
        break;
      }

      if (vac_abs_v < PFC_AC_I_VIN_ENABLE_TH_V)
      {
        pfc_ac_i_vin_enabled = 0U;
      }
      else
      {
        pfc_ac_i_vin_enabled = 1U;
      }

      if ((pfc_ac_i_vin_enabled == 0U) ||
          (pfc_vin_shape < PFC_AC_I_SHAPE_MIN))
      {
        PFC_IsrBlankDuty();
        break;
      }

#if (PFC_AC_I_BOOST_MARGIN_ENABLE != 0)
      if (vbus_v <= (vac_abs_v + PFC_AC_I_BOOST_MARGIN_V))
      {
        pfc_ac_i_margin_blank_dbg = 1U;
        PFC_IsrBlankDuty();
        break;
      }
#endif

      if (il_a >= PFC_ISR_ILIMIT_A)
      {
        current_ok = 0U;
      }

#if (PFC_USE_SYNC_IL_FOR_CURRENT_LOOP != 0)
      if ((PFC_IsIlSyncFresh() != 0U) && (il_sync_a >= PFC_ISR_ILIMIT_A))
      {
        current_ok = 0U;
      }
#endif

      if (current_ok == 0U)
      {
        PFC_IsrBlankDuty();
        break;
      }

      if (pfc_pwm_active == 0U)
      {
        if (start_duty <= PFC_PFC_DUTY_MIN_ACTIVE)
        {
          PFC_IsrBlankDuty();
          break;
        }

        pfc_iloop_integrator = 0.0f;
        pfc_duty_integrator_dbg = 0.0f;
        pfc_il_ctrl_avg = il_a;
#if (PFC_USE_SYNC_IL_FOR_CURRENT_LOOP != 0)
        if (PFC_IsIlSyncFresh() != 0U)
        {
          pfc_il_ctrl_avg = il_sync_a;
        }
#endif
        pfc_il_feedback_used = pfc_il_ctrl_avg;
        pfc_il_feedback_dbg = pfc_il_ctrl_avg;
        pfc_duty_raw_dbg = start_duty;
        pfc_duty_slew_dbg = 0.0f;
        pfc_duty_out_dbg = start_duty;
        pfc_duty_cmd = start_duty;
        PFC_PWM_StartAcRectClosedLoopAtDuty(start_duty);
        if (PFC_PWM_GetErrorCode() != 0U)
        {
          PFC_EnterFault(PFC_FAULT_PWM);
          break;
        }
        pfc_pwm_active = 1U;
      }
      break;
    }
#else
      if (vac_abs_v < PFC_AC_I_VIN_DISABLE_TH_V)
      {
        pfc_ac_i_vin_enabled = 0U;
      }
      else if (vac_abs_v > PFC_AC_I_VIN_ENABLE_TH_V)
      {
        pfc_ac_i_vin_enabled = 1U;
      }

      if ((pfc_ac_i_vin_enabled == 0U) || (pfc_vin_shape <= PFC_AC_I_SHAPE_MIN))
      {
        PFC_CurrentLoop_Task(0.0f, 1U);
      }
      else
      {
        const float derate = PFC_AcIGetDerateScale();
        const float iamp = PFC_ClampFloat(PFC_AC_I_IAMP_INIT_A * derate, 0.0f, PFC_AC_I_IAMP_MAX_A);
        const float iref = iamp * pfc_vin_shape;

#if (PFC_AC_I_BOOST_MARGIN_ENABLE != 0)
        if (vbus_v <= (vac_abs_v + PFC_AC_I_BOOST_MARGIN_V))
        {
          pfc_ac_i_margin_blank_dbg = 1U;
          PFC_IsrBlankDuty();
          break;
        }
#endif

        if (iref <= PFC_AC_I_IREF_MIN_A)
        {
          PFC_IsrBlankDuty();
          break;
        }

        PFC_CurrentLoop_Task(iref, 0U);
      }
      break;
#endif

    case PFC_STATE_PFC_RUN:
#if (PFC_TEST_PROFILE_18VAC_SIMPLE_PFC != 0)
      if (pfc_run_phase == PFC_RUN_PHASE_PRECHARGE)
      {
        PFC_SimplePreboostTask();
      }
      else
      {
        PFC_SimplePfcTask();
      }
#else
      pfc_run_margin_blank_dbg = 0U;
      pfc_pre_zero_blank_dbg = 0U;
      pfc_pre_soft_limit_dbg = 0U;
      pfc_pre_duty_clamp_dbg = 0U;
      pfc_burst_blank_dbg = 0U;
      pfc_vloop_freeze_dbg = 0U;
      pfc_iloop_freeze_dbg = 0U;

#if (PFC_RUN_ENABLE_DEFAULT == 0)
      pfc_iamp_cmd = 0.0f;
      pfc_iref_a = 0.0f;
      pfc_iref_dbg = 0.0f;
      pfc_i_err_a = 0.0f;
      pfc_duty_ff = 0.0f;
      pfc_duty_pi = 0.0f;
      pfc_duty_cmd = 0.0f;
      pfc_duty_raw_dbg = 0.0f;
      pfc_duty_slew_dbg = 0.0f;
      pfc_duty_out_dbg = 0.0f;
      pfc_i_accel_active = 0U;
      PFC_PWM_AllOff();
      pfc_pwm_active = 0U;
#else
      if (pfc_run_phase == PFC_RUN_PHASE_PRECHARGE)
      {
        PFC_PfcPrechargeTask();
        break;
      }

      if (vbus_v < (vac_abs_v + PFC_PFC_RUN_BOOST_MARGIN_V))
      {
        pfc_run_margin_blank_dbg = 1U;
        PFC_PfcRunBlankOutput();
        break;
      }

      PFC_UpdatePfcVoltageLoop();

      if (PFC_PfcRunSoftCurrentLimit() != 0U)
      {
        break;
      }

      if ((vac_abs_v < PFC_AC_I_VIN_DISABLE_TH_V) || (pfc_vin_shape < PFC_AC_I_SHAPE_MIN))
      {
        PFC_PfcRunBlankOutput();
      }
      else
      {
        const float iamp_eff = PFC_ApplyPfcVinFeedForward(pfc_iamp_cmd);

        if (iamp_eff < PFC_PFC_SOFTSTART_MIN_IAMP_A)
        {
          pfc_burst_blank_dbg = 1U;
          PFC_PfcBurstBlankOutput();
        }
        else
        {
          PFC_CurrentLoop_Task(iamp_eff * pfc_vin_shape, 0U);
        }
      }
#endif
#endif
      break;

    default:
      break;
  }
#endif
}

static void PFC_EnterState(PFC_State_t next_state)
{
  if (pfc_state == next_state)
  {
    return;
  }

  PFC_PWM_AllOff();

  switch (next_state)
  {
    case PFC_STATE_IDLE:
      PFC_BoostLoop_Reset();
      PFC_AcRectLoop_Reset();
      PFC_PfcControl_Reset();
      PFC_CurrentProtectionCountersReset();
      PFC_PWM_SetDuty(PFC_SAFE_DUTY);
      pfc_state = PFC_STATE_IDLE;
      break;

    case PFC_STATE_ADC_TEST:
      PFC_BoostLoop_Reset();
      PFC_AcRectLoop_Reset();
      PFC_PfcControl_Reset();
      PFC_CurrentProtectionCountersReset();
      PFC_PWM_SetDuty(PFC_SAFE_DUTY);
      pfc_state = PFC_STATE_ADC_TEST;
      break;

    case PFC_STATE_PWM_ASYNC_TEST:
      PFC_PWM_StartAsync();
      if (PFC_PWM_GetErrorCode() != 0U)
      {
        PFC_EnterFault(PFC_FAULT_PWM);
      }
      else
      {
        pfc_state = PFC_STATE_PWM_ASYNC_TEST;
      }
      break;

    case PFC_STATE_PWM_SYNC_TEST:
#if (PFC_ENABLE_SYNC_MODE != 0)
      PFC_PWM_StartSync();
      if (PFC_PWM_GetErrorCode() != 0U)
      {
        PFC_EnterFault(PFC_FAULT_PWM);
      }
      else
      {
        pfc_state = PFC_STATE_PWM_SYNC_TEST;
      }
#else
      PFC_PWM_StartSync();
      PFC_EnterFault(PFC_FAULT_PWM);
#endif
      break;

    case PFC_STATE_BOOST_VOLTAGE_LOOP:
#if (PFC_BOOST_CL_ENABLE != 0)
      if (PFC_BoostVoltageLoopStart() != 0U)
      {
        pfc_state = PFC_STATE_BOOST_VOLTAGE_LOOP;
      }
      else
      {
        if (PFC_PWM_GetErrorCode() == PFC_APP_ERR_BOOST_TEST_VBUS)
        {
          PFC_EnterFault(PFC_FAULT_TEST_OVP);
        }
        else
        {
          PFC_EnterFault(PFC_FAULT_PWM);
        }
      }
#else
      PFC_BoostLoop_Reset();
      pfc_state = PFC_STATE_IDLE;
#endif
      break;

    case PFC_STATE_AC_RECT_VOLTAGE_LOOP:
#if (PFC_AC_RECT_CL_ENABLE != 0)
      if (PFC_AcRectVoltageLoopStart() != 0U)
      {
        pfc_state = PFC_STATE_AC_RECT_VOLTAGE_LOOP;
      }
      else
      {
        if (PFC_PWM_GetErrorCode() == PFC_APP_ERR_AC_RECT_TEST_VBUS)
        {
          PFC_EnterFault(PFC_FAULT_TEST_OVP);
        }
        else
        {
          PFC_EnterFault(PFC_FAULT_PWM);
        }
      }
#else
      PFC_AcRectLoop_Reset();
      pfc_state = PFC_STATE_IDLE;
#endif
      break;

    case PFC_STATE_OPENLOOP_2PCT_TEST:
#if (PFC_OPENLOOP_2PCT_TEST_ENABLE != 0)
    {
      const float fixed_duty = PFC_OpenLoop2PctDuty();

      PFC_PWM_AllOff();
      PFC_PfcControl_Reset();
      PFC_CurrentProtectionCountersReset();
      PFC_OpenLoop2PctDiagnosticsReset();
      PFC_PWM_StartAcRectClosedLoopAtDuty(fixed_duty);
      if (PFC_PWM_GetErrorCode() != 0U)
      {
        PFC_EnterFault(PFC_FAULT_PWM);
      }
      else
      {
        pfc_pwm_active = 1U;
        pfc_state = PFC_STATE_OPENLOOP_2PCT_TEST;
        PFC_OpenLoop2PctApplyDebug(fixed_duty);
      }
    }
#else
      PFC_EnterState(PFC_STATE_AC_CURRENT_SHAPE_TEST);
#endif
      break;

    case PFC_STATE_DC_CURRENT_TEST:
    case PFC_STATE_PFC_RUN:
#if (PFC_PFC_ENABLE != 0)
      if ((next_state == PFC_STATE_PFC_RUN) && (vbus_v > PFC_PFC_TEST_VBUS_LIMIT_V))
      {
        PFC_PWM_SetErrorCode(PFC_APP_ERR_PFC_TEST_VBUS);
        PFC_EnterFault(PFC_FAULT_TEST_OVP);
        break;
      }

      if (PFC_PfcControl_Start() != 0U)
      {
        pfc_state = next_state;
      }
      else
      {
        if (PFC_PWM_GetErrorCode() == PFC_APP_ERR_PFC_TEST_VBUS)
        {
          PFC_EnterFault(PFC_FAULT_TEST_OVP);
        }
        else
        {
          PFC_EnterFault(PFC_FAULT_PWM);
        }
      }
#else
      PFC_PfcControl_Reset();
      pfc_state = PFC_STATE_IDLE;
#endif
      break;

    case PFC_STATE_AC_CURRENT_SHAPE_TEST:
#if (PFC_PFC_ENABLE != 0)
#if (PFC_CURRENT_LOOP_IN_ISR_ENABLE != 0) && (PFC_18VAC_FAST_ILOOP_TEST_ENABLE != 0)
      PFC_PfcControl_Reset();
      PFC_CurrentProtectionCountersReset();
      PFC_PWM_AllOff();
      pfc_pwm_active = 0U;
      pfc_iamp_cmd = PFC_AC_I_IAMP_INIT_A;
      pfc_iloop_integrator = 0.0f;
      pfc_vloop_integrator = 0.0f;
      pfc_duty_integrator_dbg = 0.0f;
      pfc_duty_out_dbg = 0.0f;
      pfc_duty_cmd = 0.0f;
      pfc_duty_raw_dbg = 0.0f;
      pfc_duty_slew_dbg = 0.0f;
      pfc_duty_ff = 0.0f;
      pfc_duty_pi = 0.0f;
      pfc_iref_a = 0.0f;
      pfc_iref_dbg = 0.0f;
      pfc_i_err_a = 0.0f;
      pfc_il_ctrl_avg = (il_sync_valid != 0U) ? il_sync_a : il_a;
      pfc_il_feedback_dbg = pfc_il_ctrl_avg;
      pfc_il_feedback_used = pfc_il_ctrl_avg;
      pfc_ac_i_vin_enabled = (vac_abs_v > PFC_AC_I_VIN_ENABLE_TH_V) ? 1U : 0U;
      PFC_IsrControlReset();
      pfc_state = PFC_STATE_AC_CURRENT_SHAPE_TEST;
#else
      if (PFC_PfcControl_Start() != 0U)
      {
        pfc_state = PFC_STATE_AC_CURRENT_SHAPE_TEST;
      }
      else
      {
        if (PFC_PWM_GetErrorCode() == PFC_APP_ERR_PFC_TEST_VBUS)
        {
          PFC_EnterFault(PFC_FAULT_TEST_OVP);
        }
        else
        {
          PFC_EnterFault(PFC_FAULT_PWM);
        }
      }
#endif
#else
      PFC_PfcControl_Reset();
      pfc_state = PFC_STATE_IDLE;
#endif
      break;

    case PFC_STATE_FAULT:
    default:
      PFC_EnterFault(PFC_FAULT_NONE);
      break;
  }
}

static void PFC_HandleButton(void)
{
  const uint32_t now_ms = HAL_GetTick();
  const GPIO_PinState sample = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2);

  if (sample != key_last_sample)
  {
    key_last_sample = sample;
    key_last_change_ms = now_ms;
  }

  if ((now_ms - key_last_change_ms) < PFC_KEY_DEBOUNCE_MS)
  {
    return;
  }

  if (sample == key_stable_level)
  {
    return;
  }

  key_stable_level = sample;

  if (key_stable_level != GPIO_PIN_RESET)
  {
    return;
  }

  if (pfc_state == PFC_STATE_FAULT)
  {
    pfc_fault = PFC_FAULT_NONE;
    PFC_PWM_SetErrorCode(0U);
    PFC_BoostLoop_Reset();
    PFC_AcRectLoop_Reset();
    PFC_PfcControl_Reset();
    PFC_CurrentProtectionCountersReset();
    PFC_EnterState(PFC_STATE_IDLE);
    return;
  }

  switch (pfc_state)
  {
    case PFC_STATE_IDLE:
      PFC_EnterState(PFC_STATE_ADC_TEST);
      break;
    case PFC_STATE_ADC_TEST:
#if (PFC_PFC_ENABLE != 0)
#if (PFC_OPENLOOP_2PCT_TEST_ENABLE != 0)
      PFC_EnterState(PFC_STATE_OPENLOOP_2PCT_TEST);
#else
#if (PFC_18VAC_FAST_ILOOP_TEST_ENABLE != 0)
      PFC_EnterState(PFC_STATE_AC_CURRENT_SHAPE_TEST);
#elif ((PFC_TEST_PROFILE_18VAC_SIMPLE_PFC != 0) || (PFC_TEST_PROFILE_18VAC_PFC_RUN_32V != 0))
      PFC_EnterState(PFC_STATE_PFC_RUN);
#elif (PFC_TEST_PROFILE_18VAC_AC_I != 0)
      PFC_EnterState(PFC_STATE_AC_CURRENT_SHAPE_TEST);
#else
      PFC_EnterState(PFC_STATE_DC_CURRENT_TEST);
#endif
#endif
#else
#if (PFC_AC_TEST_DIRECT_MODE != 0)
      PFC_EnterState(PFC_STATE_AC_RECT_VOLTAGE_LOOP);
#else
      PFC_EnterState(PFC_STATE_PWM_ASYNC_TEST);
#endif
#endif
      break;
    case PFC_STATE_OPENLOOP_2PCT_TEST:
      PFC_EnterState(PFC_STATE_AC_CURRENT_SHAPE_TEST);
      break;
    case PFC_STATE_DC_CURRENT_TEST:
      PFC_EnterState(PFC_STATE_AC_CURRENT_SHAPE_TEST);
      break;
    case PFC_STATE_AC_CURRENT_SHAPE_TEST:
#if (PFC_18VAC_FAST_ILOOP_TEST_ENABLE != 0)
      PFC_EnterState(PFC_STATE_PFC_RUN);
#elif ((PFC_TEST_PROFILE_18VAC_AC_I != 0) || (PFC_TEST_PROFILE_18VAC_PFC_RUN_32V != 0) || (PFC_TEST_PROFILE_18VAC_SIMPLE_PFC != 0))
      PFC_EnterState(PFC_STATE_IDLE);
#else
      PFC_EnterState(PFC_STATE_PFC_RUN);
#endif
      break;
    case PFC_STATE_PFC_RUN:
      PFC_EnterState(PFC_STATE_IDLE);
      break;
    case PFC_STATE_PWM_ASYNC_TEST:
#if (PFC_BOOST_CL_ENABLE != 0)
      PFC_EnterState(PFC_STATE_BOOST_VOLTAGE_LOOP);
#else
      PFC_EnterState(PFC_STATE_IDLE);
#endif
      break;
    case PFC_STATE_BOOST_VOLTAGE_LOOP:
#if (PFC_AC_RECT_CL_ENABLE != 0)
      PFC_EnterState(PFC_STATE_AC_RECT_VOLTAGE_LOOP);
#else
      PFC_EnterState(PFC_STATE_IDLE);
#endif
      break;
    case PFC_STATE_AC_RECT_VOLTAGE_LOOP:
      PFC_EnterState(PFC_STATE_IDLE);
      break;
    case PFC_STATE_PWM_SYNC_TEST:
      PFC_EnterState(PFC_STATE_IDLE);
      break;
    case PFC_STATE_FAULT:
    default:
      PFC_EnterState(PFC_STATE_IDLE);
      break;
  }
}

static void PFC_CheckProtection(void)
{
  if (pfc_state == PFC_STATE_FAULT)
  {
    PFC_PWM_AllOff();
    return;
  }

  if (PFC_HandleIsrFaultRequest() != 0U)
  {
    return;
  }

  if (pfc_state == PFC_STATE_OPENLOOP_2PCT_TEST)
  {
    if (PFC_PWM_GetErrorCode() != 0U)
    {
      PFC_EnterFault(PFC_FAULT_PWM);
      return;
    }

    if (vbus_v > PFC_OPENLOOP_2PCT_VBUS_LIMIT_V)
    {
      PFC_PWM_SetErrorCode(PFC_APP_ERR_PFC_TEST_VBUS);
      PFC_EnterFault(PFC_FAULT_TEST_OVP);
      return;
    }

    if (vbus_v >= PFC_VBUS_OVP_HARD_V)
    {
      PFC_EnterFault(PFC_FAULT_OVP);
      return;
    }

    if (vbus_v >= PFC_VBUS_OVP_SOFT_V)
    {
      PFC_EnterFault(PFC_FAULT_OVP);
      return;
    }

    pfc_test_ilimit_cnt = 0U;
    il_ocp_cnt = 0U;
    il_rev_cnt = 0U;
    return;
  }

  if ((pfc_state == PFC_STATE_BOOST_VOLTAGE_LOOP) && (vbus_v > PFC_BOOST_TEST_VBUS_LIMIT_V))
  {
    PFC_PWM_SetErrorCode(PFC_APP_ERR_BOOST_TEST_VBUS);
    PFC_EnterFault(PFC_FAULT_TEST_OVP);
    return;
  }

  if ((pfc_state == PFC_STATE_AC_RECT_VOLTAGE_LOOP) && (vbus_v > PFC_AC_RECT_TEST_VBUS_LIMIT_V))
  {
    PFC_PWM_SetErrorCode(PFC_APP_ERR_AC_RECT_TEST_VBUS);
    PFC_EnterFault(PFC_FAULT_TEST_OVP);
    return;
  }

  if ((pfc_state == PFC_STATE_DC_CURRENT_TEST) && (vbus_v > PFC_DC_I_VBUS_LIMIT_V))
  {
    PFC_PWM_SetErrorCode(PFC_APP_ERR_PFC_TEST_VBUS);
    PFC_EnterFault(PFC_FAULT_TEST_OVP);
    return;
  }

  if ((pfc_state == PFC_STATE_AC_CURRENT_SHAPE_TEST) && (vbus_v > PFC_AC_I_VBUS_LIMIT_V))
  {
    PFC_PWM_SetErrorCode(PFC_APP_ERR_PFC_TEST_VBUS);
    PFC_EnterFault(PFC_FAULT_TEST_OVP);
    return;
  }

  if ((pfc_state == PFC_STATE_PFC_RUN) && (vbus_v > PFC_PFC_TEST_VBUS_LIMIT_V))
  {
    PFC_PWM_SetErrorCode(PFC_APP_ERR_PFC_TEST_VBUS);
    PFC_EnterFault(PFC_FAULT_TEST_OVP);
    return;
  }

  if (PFC_PWM_GetErrorCode() != 0U)
  {
    PFC_EnterFault(PFC_FAULT_PWM);
    return;
  }

  if (vbus_v >= PFC_VBUS_OVP_HARD_V)
  {
    PFC_EnterFault(PFC_FAULT_OVP);
    return;
  }

  if (vbus_v >= PFC_VBUS_OVP_SOFT_V)
  {
    PFC_EnterFault(PFC_FAULT_OVP);
    return;
  }

  pfc_test_ilimit_cnt = 0U;

  if (PFC_IsCurrentProtectionBlanked() != 0U)
  {
    PFC_CurrentProtectionCountersReset();
    return;
  }

  if (il_a >= PFC_IL_OCP_A)
  {
    if (il_ocp_cnt < PFC_IL_OCP_TRIP_COUNT)
    {
      il_ocp_cnt++;
    }
  }
  else
  {
    il_ocp_cnt = 0U;
  }

  if (il_a <= PFC_IL_REVERSE_OCP_A)
  {
    if (il_rev_cnt < PFC_IL_REV_TRIP_COUNT)
    {
      il_rev_cnt++;
    }
  }
  else
  {
    il_rev_cnt = 0U;
  }

  if (il_ocp_cnt >= PFC_IL_OCP_TRIP_COUNT)
  {
    PFC_EnterFault(PFC_FAULT_OCP);
    return;
  }

  if (il_rev_cnt >= PFC_IL_REV_TRIP_COUNT)
  {
    PFC_EnterFault(PFC_FAULT_REV);
  }
}

static void PFC_UpdateDisplayPage1(char *line, size_t line_size, char *value, size_t value_size)
{
  (void)snprintf(line, line_size, "STATE: %s", PFC_StateName(pfc_state));
  PFC_OLED_WriteLine(0U, line);

  (void)snprintf(line, line_size, "RAW VIN  %4u", raw_vac);
  PFC_OLED_WriteLine(1U, line);

  (void)snprintf(line, line_size, "RAW IL   %4u", raw_il);
  PFC_OLED_WriteLine(2U, line);

  (void)snprintf(line, line_size, "RAW VBUS %4u", raw_vbus);
  PFC_OLED_WriteLine(3U, line);

  PFC_FormatFixed(value, value_size, vac_abs_v, 1U);
  (void)snprintf(line, line_size, "VIN: %s V", value);
  PFC_OLED_WriteLine(4U, line);

  PFC_FormatFixed(value, value_size, il_a, 2U);
  (void)snprintf(line, line_size, "IL : %s A", value);
  PFC_OLED_WriteLine(5U, line);

  PFC_FormatFixed(value, value_size, vbus_v, 1U);
  (void)snprintf(line, line_size, "VBUS: %s V", value);
  PFC_OLED_WriteLine(6U, line);

  const uint32_t duty_cmd_pct = (uint32_t)((ac_rect_duty_cmd * 100.0f) + 0.5f);
  const uint32_t duty_internal_pct = (uint32_t)((ac_rect_duty_internal * 100.0f) + 0.5f);
  const uint32_t duty_output_pct = (uint32_t)((ac_rect_duty_output * 100.0f) + 0.5f);
  (void)snprintf(line,
                 line_size,
                 "C%02luI%02luO%02luG%u",
                 (unsigned long)duty_cmd_pct,
                 (unsigned long)duty_internal_pct,
                 (unsigned long)duty_output_pct,
                 (unsigned)ac_rect_vin_enabled);
  PFC_OLED_WriteLine(7U, line);
}

static void PFC_UpdateDisplayPage2(char *line, size_t line_size, char *value, size_t value_size)
{
  (void)snprintf(line, line_size, "RAW REF  %4u", raw_vrefint);
  PFC_OLED_WriteLine(0U, line);

  PFC_FormatFixed(value, value_size, il_zero_runtime_v, 3U);
  (void)snprintf(line, line_size, "Z:%s C:%u", value, (unsigned)il_zero_calibrated);
  PFC_OLED_WriteLine(1U, line);

  PFC_FormatFixed(value, value_size, il_zero_cal_span_v, 3U);
  (void)snprintf(line, line_size, "S:%s", value);
  PFC_OLED_WriteLine(2U, line);

  PFC_FormatFixed(value, value_size, adc_vref_used_v, 3U);
  (void)snprintf(line, line_size, "VREFU %s", value);
  PFC_OLED_WriteLine(3U, line);

  PFC_FormatFixed(value, value_size, vac_adc_v, 3U);
  (void)snprintf(line, line_size, "VACADC %s", value);
  PFC_OLED_WriteLine(4U, line);

  PFC_FormatFixed(value, value_size, il_adc_v, 3U);
  (void)snprintf(line, line_size, "ILADC  %s", value);
  PFC_OLED_WriteLine(5U, line);

  PFC_FormatFixed(value, value_size, vbus_adc_v, 3U);
  (void)snprintf(line, line_size, "VBUSA  %s", value);
  PFC_OLED_WriteLine(6U, line);

  const uint32_t duty_cmd_pct = (uint32_t)((ac_rect_duty_cmd * 100.0f) + 0.5f);
  const uint32_t duty_internal_pct = (uint32_t)((ac_rect_duty_internal * 100.0f) + 0.5f);
  const uint32_t duty_output_pct = (uint32_t)((ac_rect_duty_output * 100.0f) + 0.5f);
  (void)snprintf(line,
                 line_size,
                 "C%02luI%02luO%02luG%u",
                 (unsigned long)duty_cmd_pct,
                 (unsigned long)duty_internal_pct,
                 (unsigned long)duty_output_pct,
                 (unsigned)ac_rect_vin_enabled);
  PFC_OLED_WriteLine(7U, line);
}

static void PFC_UpdateDisplayPageOpenLoop2Pct(char *line, size_t line_size, char *value, size_t value_size)
{
  char value2[12];
  const uint32_t duty_pct = (uint32_t)((PFC_OpenLoop2PctDuty() * 100.0f) + 0.5f);
  const uint32_t isr_count = openloop_isr_count & 0x0FFFUL;

  (void)snprintf(line, line_size, "OPEN D:%02lu", (unsigned long)duty_pct);
  PFC_OLED_WriteLine(0U, line);

  PFC_FormatFixed(value, value_size, il_a, 2U);
  PFC_FormatFixed(value2, sizeof(value2), il_sync_a, 2U);
  (void)snprintf(line, line_size, "IL:%s IS:%s", value, value2);
  PFC_OLED_WriteLine(1U, line);

  PFC_FormatFixed(value, value_size, openloop_pk_il, 2U);
  PFC_FormatFixed(value2, sizeof(value2), openloop_pk_is, 2U);
  (void)snprintf(line, line_size, "PKI:%s PKS:%s", value, value2);
  PFC_OLED_WriteLine(2U, line);

  PFC_FormatFixed(value, value_size, vbus_v, 1U);
  PFC_FormatFixed(value2, sizeof(value2), vac_abs_v, 1U);
  (void)snprintf(line, line_size, "VB:%s VA:%s", value, value2);
  PFC_OLED_WriteLine(3U, line);

  (void)snprintf(line,
                 line_size,
                 "RAW:%04u RS:%04u",
                 (unsigned)raw_il,
                 (unsigned)raw_il_sync);
  PFC_OLED_WriteLine(4U, line);

  PFC_FormatFixed(value, value_size, il_zero_runtime_v, 3U);
  (void)snprintf(line, line_size, "Z:%s C:%u", value, (unsigned)il_zero_calibrated);
  PFC_OLED_WriteLine(5U, line);

  PFC_FormatFixed(value, value_size, il_zero_cal_span_v, 3U);
  (void)snprintf(line, line_size, "S:%s", value);
  PFC_OLED_WriteLine(6U, line);

  (void)snprintf(line,
                 line_size,
                 "C:%04lu F:%u",
                 (unsigned long)isr_count,
                 (unsigned)pfc_fault);
  PFC_OLED_WriteLine(7U, line);
}

static void PFC_UpdateDisplayPagePfc(char *line, size_t line_size, char *value, size_t value_size)
{
  char value2[12];
  uint32_t duty_pct;
  uint32_t ff_pct;
  int32_t pi_pct;
  uint32_t raw_pct;
  uint32_t out_pct;
  uint32_t loop_count;

  (void)snprintf(line,
                 line_size,
                 "%s FST%u ACC%u",
                 PFC_StateName(pfc_state),
                 (unsigned)PFC_DC_I_FAST_RESPONSE_ENABLE,
                 (unsigned)pfc_i_accel_active);
  PFC_OLED_WriteLine(0U, line);

  PFC_FormatFixed(value, value_size, il_a, 2U);
  PFC_FormatFixed(value2, sizeof(value2), pfc_il_feedback_dbg, 2U);
  (void)snprintf(line, line_size, "IL:%s IA:%s", value, value2);
  PFC_OLED_WriteLine(1U, line);

  PFC_FormatFixed(value, value_size, pfc_iref_dbg, 2U);
  PFC_FormatFixed(value2, sizeof(value2), pfc_i_err_a, 2U);
  (void)snprintf(line, line_size, "IR:%s IE:%s", value, value2);
  PFC_OLED_WriteLine(2U, line);

  duty_pct = (uint32_t)((pfc_duty_cmd * 100.0f) + 0.5f);
  PFC_FormatFixed(value, value_size, vbus_v, 1U);
  (void)snprintf(line, line_size, "VB:%s D:%02lu", value, (unsigned long)duty_pct);
  PFC_OLED_WriteLine(3U, line);

  ff_pct = (uint32_t)((pfc_duty_ff * 100.0f) + 0.5f);
  pi_pct = (pfc_duty_pi >= 0.0f) ? (int32_t)((pfc_duty_pi * 100.0f) + 0.5f)
                                 : (int32_t)((pfc_duty_pi * 100.0f) - 0.5f);
  (void)snprintf(line,
                 line_size,
                 "FF:%02lu PI:%+03ld",
                 (unsigned long)ff_pct,
                 (long)pi_pct);
  PFC_OLED_WriteLine(4U, line);

  raw_pct = (uint32_t)((pfc_duty_raw_dbg * 100.0f) + 0.5f);
  out_pct = (uint32_t)((pfc_duty_out_dbg * 100.0f) + 0.5f);
  (void)snprintf(line,
                 line_size,
                 "RAW:%02lu OUT:%02lu",
                 (unsigned long)raw_pct,
                 (unsigned long)out_pct);
  PFC_OLED_WriteLine(5U, line);

  loop_count = pfc_current_loop_update_count & 0x0FFFUL;
  (void)snprintf(line,
                 line_size,
                 "LC:%04lu E:%04lX",
                 (unsigned long)loop_count,
                 (unsigned long)PFC_PWM_GetErrorCode());
  PFC_OLED_WriteLine(6U, line);

  PFC_FormatFixed(value, value_size, il_sync_a, 2U);
  (void)snprintf(line,
                 line_size,
                 "F:%u IS:%s",
                 (unsigned)pfc_fault,
                 value);
  PFC_OLED_WriteLine(7U, line);
}

static void PFC_UpdateDisplayPageAcI(char *line, size_t line_size, char *value, size_t value_size)
{
  char value2[12];
  uint32_t duty_pct;
  uint32_t ff_pct;
  uint32_t raw_pct;
  uint32_t out_pct;
  uint32_t loop_count;

#if (PFC_CURRENT_LOOP_IN_ISR_ENABLE != 0)
  if (pfc_state == PFC_STATE_AC_CURRENT_SHAPE_TEST)
  {
    (void)snprintf(line, line_size, "18VAC ISR10K");
    PFC_OLED_WriteLine(0U, line);

    PFC_FormatFixed(value, value_size, pfc_iref_dbg, 2U);
    PFC_FormatFixed(value2, sizeof(value2), pfc_il_feedback_dbg, 2U);
    (void)snprintf(line, line_size, "IR:%s IA:%s", value, value2);
    PFC_OLED_WriteLine(1U, line);

    PFC_FormatFixed(value, value_size, pfc_i_err_a, 2U);
    PFC_FormatFixed(value2, sizeof(value2), pfc_iamp_cmd, 2U);
    (void)snprintf(line, line_size, "IE:%s IP:%s", value, value2);
    PFC_OLED_WriteLine(2U, line);

    PFC_FormatFixed(value, value_size, il_a, 2U);
    PFC_FormatFixed(value2, sizeof(value2), il_sync_a, 2U);
    (void)snprintf(line, line_size, "IL:%s IS:%s", value, value2);
    PFC_OLED_WriteLine(3U, line);

    PFC_FormatFixed(value, value_size, vbus_v, 1U);
    PFC_FormatFixed(value2, sizeof(value2), vac_abs_v, 1U);
    (void)snprintf(line, line_size, "VB:%s VA:%s", value, value2);
    PFC_OLED_WriteLine(4U, line);

    duty_pct = (uint32_t)((pfc_duty_cmd * 100.0f) + 0.5f);
    PFC_FormatFixed(value, value_size, pfc_vin_shape, 2U);
    (void)snprintf(line, line_size, "VS:%s D:%02lu", value, (unsigned long)duty_pct);
    PFC_OLED_WriteLine(5U, line);

    raw_pct = (uint32_t)((pfc_duty_raw_dbg * 100.0f) + 0.5f);
    out_pct = (uint32_t)((pfc_duty_out_dbg * 100.0f) + 0.5f);
    (void)snprintf(line,
                   line_size,
                   "RAW:%02lu OUT:%02lu",
                   (unsigned long)raw_pct,
                   (unsigned long)out_pct);
    PFC_OLED_WriteLine(6U, line);

    loop_count = pfc_isr_loop_count & 0x0FFFUL;
    (void)snprintf(line,
                   line_size,
                   "LC:%04lu H:%u F:%u",
                   (unsigned long)loop_count,
                   (unsigned)pfc_isr_headroom_ok_dbg,
                   (unsigned)pfc_fault);
    PFC_OLED_WriteLine(7U, line);
    return;
  }
#endif

#if (PFC_TEST_PROFILE_18VAC_AC_I != 0)
  (void)snprintf(line,
                 line_size,
                 "18VAC AC_I M%u",
                 (unsigned)pfc_ac_i_margin_blank_dbg);
#else
  (void)snprintf(line,
                 line_size,
                 "AC_I FST%u ACC%u",
                 (unsigned)PFC_DC_I_FAST_RESPONSE_ENABLE,
                 (unsigned)pfc_i_accel_active);
#endif
  PFC_OLED_WriteLine(0U, line);

  PFC_FormatFixed(value, value_size, il_a, 2U);
  PFC_FormatFixed(value2, sizeof(value2), pfc_il_feedback_dbg, 2U);
  (void)snprintf(line, line_size, "IL:%s IA:%s", value, value2);
  PFC_OLED_WriteLine(1U, line);

  PFC_FormatFixed(value, value_size, pfc_iref_dbg, 2U);
  PFC_FormatFixed(value2, sizeof(value2), pfc_i_err_a, 2U);
  (void)snprintf(line, line_size, "IR:%s IE:%s", value, value2);
  PFC_OLED_WriteLine(2U, line);

  duty_pct = (uint32_t)((pfc_duty_cmd * 100.0f) + 0.5f);
  PFC_FormatFixed(value, value_size, vbus_v, 1U);
  (void)snprintf(line, line_size, "VB:%s D:%02lu", value, (unsigned long)duty_pct);
  PFC_OLED_WriteLine(3U, line);

  PFC_FormatFixed(value, value_size, vac_abs_v, 1U);
  PFC_FormatFixed(value2, sizeof(value2), pfc_vin_peak, 1U);
  (void)snprintf(line, line_size, "VA:%s VP:%s", value, value2);
  PFC_OLED_WriteLine(4U, line);

  PFC_FormatFixed(value, value_size, pfc_vin_shape, 2U);
  ff_pct = (uint32_t)((pfc_duty_ff * 100.0f) + 0.5f);
  (void)snprintf(line, line_size, "VS:%s FF:%02lu", value, (unsigned long)ff_pct);
  PFC_OLED_WriteLine(5U, line);

  raw_pct = (uint32_t)((pfc_duty_raw_dbg * 100.0f) + 0.5f);
  out_pct = (uint32_t)((pfc_duty_out_dbg * 100.0f) + 0.5f);
  (void)snprintf(line,
                 line_size,
                 "RAW:%02lu OUT:%02lu",
                 (unsigned long)raw_pct,
                 (unsigned long)out_pct);
  PFC_OLED_WriteLine(6U, line);

  loop_count = pfc_current_loop_update_count & 0x0FFFUL;
  (void)snprintf(line,
                 line_size,
                 "LC:%04lu E:%04lX F:%u",
                 (unsigned long)loop_count,
                 (unsigned long)PFC_PWM_GetErrorCode(),
                 (unsigned)pfc_fault);
  PFC_OLED_WriteLine(7U, line);
}

static void PFC_UpdateDisplayPagePfcRun(char *line, size_t line_size, char *value, size_t value_size)
{
  char value2[12];
  uint32_t duty_pct;
  uint32_t raw_pct;
  uint32_t out_pct;
  uint32_t loop_count;

#if (PFC_TEST_PROFILE_18VAC_SIMPLE_PFC != 0)
  int32_t ff_pct;
  int32_t pi_pct;
  uint32_t dl_pct;
  uint32_t cap_pct;

  (void)snprintf(line,
                 line_size,
                 "18VAC %s",
                 (pfc_run_phase == PFC_RUN_PHASE_PRECHARGE) ? "PREB" : "SIMPLE");
  PFC_OLED_WriteLine(0U, line);

  PFC_FormatFixed(value, value_size, vbus_v, 1U);
  PFC_FormatFixed(value2, sizeof(value2), pfc_vbus_avg, 1U);
  (void)snprintf(line, line_size, "VB:%s AVG:%s", value, value2);
  PFC_OLED_WriteLine(1U, line);

  PFC_FormatFixed(value, value_size, pfc_vbus_ref, 1U);
  PFC_FormatFixed(value2, sizeof(value2), pfc_iamp_cmd, 2U);
  (void)snprintf(line, line_size, "VR:%s IP:%s", value, value2);
  PFC_OLED_WriteLine(2U, line);

  PFC_FormatFixed(value, value_size, pfc_iref_dbg, 2U);
  PFC_FormatFixed(value2, sizeof(value2), pfc_i_err_a, 2U);
  (void)snprintf(line, line_size, "IR:%s IE:%s", value, value2);
  PFC_OLED_WriteLine(3U, line);

  PFC_FormatFixed(value, value_size, pfc_il_feedback_dbg, 2U);
  PFC_FormatFixed(value2, sizeof(value2), il_sync_a, 2U);
  (void)snprintf(line, line_size, "FB:%s IS:%s", value, value2);
  PFC_OLED_WriteLine(4U, line);

  ff_pct = (int32_t)((pfc_duty_ff * 100.0f) + ((pfc_duty_ff >= 0.0f) ? 0.5f : -0.5f));
  pi_pct = (int32_t)((pfc_duty_pi * 100.0f) + ((pfc_duty_pi >= 0.0f) ? 0.5f : -0.5f));
  PFC_FormatFixed(value, value_size, pfc_vin_shape, 2U);
  (void)snprintf(line,
                 line_size,
                 "VS:%s FF:%ld PI:%ld",
                 value,
                 (long)ff_pct,
                 (long)pi_pct);
  PFC_OLED_WriteLine(5U, line);

  duty_pct = (uint32_t)((pfc_duty_cmd * 100.0f) + 0.5f);
  raw_pct = (uint32_t)((pfc_duty_raw_dbg * 100.0f) + 0.5f);
  out_pct = (uint32_t)((pfc_duty_out_dbg * 100.0f) + 0.5f);
  (void)snprintf(line,
                 line_size,
                 "D:%02lu RAW:%02lu OUT:%02lu",
                 (unsigned long)duty_pct,
                 (unsigned long)raw_pct,
                 (unsigned long)out_pct);
  PFC_OLED_WriteLine(6U, line);

  dl_pct = (uint32_t)((pfc_simple_duty_limit * 100.0f) + 0.5f);
  cap_pct = (uint32_t)((pfc_simple_duty_cap_dbg * 100.0f) + 0.5f);
  loop_count = pfc_isr_loop_count & 0x0FFFUL;
  (void)loop_count;
  (void)snprintf(line,
                 line_size,
                 "DL%02lu CP%02lu B%uH%uS%uC%u",
                 (unsigned long)dl_pct,
                 (unsigned long)cap_pct,
                 (unsigned)((pfc_run_margin_blank_dbg != 0U) || (pfc_simple_no_demand_dbg != 0U)),
                 (unsigned)pfc_simple_headroom_cap_dbg,
                 (unsigned)pfc_pre_soft_limit_dbg,
                 (unsigned)pfc_pre_duty_clamp_dbg);
  PFC_OLED_WriteLine(7U, line);
  return;
#endif

#if (PFC_TEST_PROFILE_18VAC_PFC_RUN_32V != 0)
  (void)snprintf(line,
                 line_size,
                 "18VAC %s M%uZ%uS%uC%uB%u",
                 (pfc_run_phase == PFC_RUN_PHASE_PRECHARGE) ? "PRE" : "SHP",
                 (unsigned)pfc_run_margin_blank_dbg,
                 (unsigned)pfc_pre_zero_blank_dbg,
                 (unsigned)pfc_pre_soft_limit_dbg,
                 (unsigned)pfc_pre_duty_clamp_dbg,
                 (unsigned)pfc_burst_blank_dbg);
#else
  (void)snprintf(line,
                 line_size,
                 "PFC %s",
                 (pfc_run_phase == PFC_RUN_PHASE_PRECHARGE) ? "PRE" : "SHP");
#endif
  PFC_OLED_WriteLine(0U, line);

  PFC_FormatFixed(value, value_size, vbus_v, 1U);
  PFC_FormatFixed(value2, sizeof(value2), pfc_vbus_avg, 1U);
  (void)snprintf(line, line_size, "VB:%s AVG:%s", value, value2);
  PFC_OLED_WriteLine(1U, line);

  PFC_FormatFixed(value, value_size, pfc_vbus_ref, 1U);
  PFC_FormatFixed(value2, sizeof(value2), pfc_iamp_cmd, 2U);
  (void)snprintf(line, line_size, "VR:%s IP:%s", value, value2);
  PFC_OLED_WriteLine(2U, line);

  PFC_FormatFixed(value, value_size, pfc_iref_dbg, 2U);
  PFC_FormatFixed(value2, sizeof(value2), pfc_il_feedback_dbg, 2U);
  (void)snprintf(line, line_size, "IR:%s IA:%s", value, value2);
  PFC_OLED_WriteLine(3U, line);

  duty_pct = (uint32_t)((pfc_duty_cmd * 100.0f) + 0.5f);
  raw_pct = (uint32_t)((pfc_duty_raw_dbg * 100.0f) + 0.5f);
  (void)snprintf(line,
                 line_size,
                 "D:%02lu RAW:%02lu",
                 (unsigned long)duty_pct,
                 (unsigned long)raw_pct);
  PFC_OLED_WriteLine(4U, line);

  PFC_FormatFixed(value, value_size, vac_abs_v, 1U);
  out_pct = (uint32_t)((pfc_duty_out_dbg * 100.0f) + 0.5f);
  (void)snprintf(line,
                 line_size,
                 "OUT:%02lu VA:%s",
                 (unsigned long)out_pct,
                 value);
  PFC_OLED_WriteLine(5U, line);

  PFC_FormatFixed(value, value_size, pfc_vin_shape, 2U);
  PFC_FormatFixed(value2, sizeof(value2), pfc_precharge_iref_cmd, 2U);
  (void)snprintf(line,
                 line_size,
                 "VS:%s PR:%s",
                 value,
                 value2);
  PFC_OLED_WriteLine(6U, line);

  (void)snprintf(line,
                 line_size,
                 "E:%04lX F:%u",
                 (unsigned long)PFC_PWM_GetErrorCode(),
                 (unsigned)pfc_fault);
  PFC_OLED_WriteLine(7U, line);
}

static void PFC_UpdateDisplayPageFault(char *line, size_t line_size, char *value, size_t value_size)
{
  char value2[12];
  uint32_t duty_pct;

  (void)snprintf(line, line_size, "STATE: FAULT");
  PFC_OLED_WriteLine(0U, line);

  (void)snprintf(line,
                 line_size,
                 "FS:%02u F:%02u",
                 (unsigned)fault_prev_state,
                 (unsigned)pfc_fault);
  PFC_OLED_WriteLine(1U, line);

  (void)snprintf(line,
                 line_size,
                 "E:%04lX",
                 (unsigned long)PFC_PWM_GetErrorCode());
  PFC_OLED_WriteLine(2U, line);

  PFC_FormatFixed(value, value_size, fault_vin_v, 1U);
  (void)snprintf(line, line_size, "VIN:%s", value);
  PFC_OLED_WriteLine(3U, line);

  PFC_FormatFixed(value, value_size, fault_il_a, 2U);
  PFC_FormatFixed(value2, sizeof(value2), fault_il_sync_a, 2U);
  (void)snprintf(line, line_size, "IL:%s IS:%s", value, value2);
  PFC_OLED_WriteLine(4U, line);

  PFC_FormatFixed(value, value_size, fault_vbus_v, 1U);
  (void)snprintf(line, line_size, "VBUS:%s", value);
  PFC_OLED_WriteLine(5U, line);

  duty_pct = (uint32_t)((fault_duty * 100.0f) + 0.5f);
  PFC_FormatFixed(value, value_size, fault_iref, 2U);
  (void)snprintf(line,
                 line_size,
                 "D:%02lu IR:%s",
                 (unsigned long)duty_pct,
                 value);
  PFC_OLED_WriteLine(6U, line);

  PFC_FormatFixed(value, value_size, fault_peak_isr_il_sync_a, 2U);
  (void)snprintf(line,
                 line_size,
                 "PK:%s C:%u",
                 value,
                 (unsigned)fault_peak_isr_cnt);
  PFC_OLED_WriteLine(7U, line);
}

static void PFC_UpdateDisplay(void)
{
  char line[32];
  char value[12];
  const uint32_t now_ms = HAL_GetTick();
  const uint8_t page = (uint8_t)PFC_OLED_FIXED_PAGE;

  if ((now_ms - oled_last_refresh_ms) < PFC_OLED_REFRESH_MS)
  {
    return;
  }
  oled_last_refresh_ms = now_ms;

  PFC_OLED_Clear();

  if (pfc_state == PFC_STATE_FAULT)
  {
    PFC_UpdateDisplayPageFault(line, sizeof(line), value, sizeof(value));
  }
  else if (page == 0U)
  {
    if (pfc_state == PFC_STATE_OPENLOOP_2PCT_TEST)
    {
      PFC_UpdateDisplayPageOpenLoop2Pct(line, sizeof(line), value, sizeof(value));
    }
    else if (PFC_IsPfcTestState() != 0U)
    {
      if (pfc_state == PFC_STATE_PFC_RUN)
      {
        PFC_UpdateDisplayPagePfcRun(line, sizeof(line), value, sizeof(value));
      }
      else
#if (PFC_OLED_AC_I_DEBUG_PAGE_ENABLE != 0)
      if (pfc_state == PFC_STATE_AC_CURRENT_SHAPE_TEST)
      {
        PFC_UpdateDisplayPageAcI(line, sizeof(line), value, sizeof(value));
      }
      else
#endif
      {
      PFC_UpdateDisplayPagePfc(line, sizeof(line), value, sizeof(value));
      }
    }
    else
    {
      PFC_UpdateDisplayPage1(line, sizeof(line), value, sizeof(value));
    }
  }
  else
  {
    PFC_UpdateDisplayPage2(line, sizeof(line), value, sizeof(value));
  }

  PFC_OLED_Update();
}

void PFC_App_Task(void)
{
  PFC_CheckProtection();
  PFC_HandleButton();
  PFC_CheckProtection();
  PFC_BoostVoltageLoopTask();
  PFC_AcRectVoltageLoopTask();
  PFC_OpenLoop2PctTask();
  PFC_PfcTask();
  PFC_CheckProtection();
  PFC_UpdateDisplay();
}

PFC_State_t PFC_App_GetState(void)
{
  return pfc_state;
}

PFC_Fault_t PFC_App_GetFault(void)
{
  return pfc_fault;
}
