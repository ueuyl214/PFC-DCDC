#include "pfc_pwm.h"

#include "hrtim.h"

#define PFC_PWM_TIMER_INDEX             HRTIM_TIMERINDEX_TIMER_B
#define PFC_PWM_TIMER_ID                HRTIM_TIMERID_TIMER_B
#define PFC_PWM_OUTPUT_HIGH             HRTIM_OUTPUT_TB1
#define PFC_PWM_OUTPUT_LOW              HRTIM_OUTPUT_TB2
#define PFC_PWM_OUTPUT_BOTH             (PFC_PWM_OUTPUT_HIGH | PFC_PWM_OUTPUT_LOW)

#define PFC_PWM_ERR_CFG_SYNC_OUT1       0x0101U
#define PFC_PWM_ERR_CFG_SYNC_OUT2       0x0102U
#define PFC_PWM_ERR_CFG_ASYNC_OUT1      0x0201U
#define PFC_PWM_ERR_CFG_ASYNC_OUT2      0x0202U
#define PFC_PWM_ERR_UPDATE_CONFIG       0x0301U
#define PFC_PWM_ERR_UPDATE_DUTY         0x0302U
#define PFC_PWM_ERR_UPDATE_ALLOFF       0x0303U
#define PFC_PWM_ERR_UPDATE_DT_ASYNC     0x0304U
#define PFC_PWM_ERR_UPDATE_DT_SYNC      0x0305U
#define PFC_PWM_ERR_COUNT_ASYNC         0x0401U
#define PFC_PWM_ERR_OUTPUT_ASYNC        0x0402U
#define PFC_PWM_ERR_COUNT_SYNC          0x0501U
#define PFC_PWM_ERR_OUTPUT_SYNC         0x0502U
#define PFC_PWM_ERR_INIT_TIMEBASE       0x0601U
#define PFC_PWM_ERR_INIT_CONTROL        0x0602U
#define PFC_PWM_ERR_INIT_TIMER          0x0603U
#define PFC_PWM_ERR_INIT_COMPARE        0x0604U
#define PFC_PWM_ERR_INIT_DEADTIME       0x0605U
#define PFC_PWM_ERR_INIT_OUT1           0x0606U
#define PFC_PWM_ERR_INIT_OUT2           0x0607U
#define PFC_PWM_ERR_INIT_UPDATE         0x0608U
#define PFC_PWM_ERR_INIT_COMPARE2       0x0609U
#define PFC_PWM_ERR_SYNC_DISABLED       0x0701U

uint32_t pfc_pwm_error_code = 0U;
static float pfc_pwm_duty = 0.0f;
static uint8_t pfc_pwm_timer_configured = 0U;
static uint8_t pfc_pwm_timer_running = 0U;

static float PFC_PWM_LimitDuty(float duty)
{
  if (duty < 0.0f)
  {
    return 0.0f;
  }
  if (duty > PFC_DUTY_OPENLOOP_MAX)
  {
    return PFC_DUTY_OPENLOOP_MAX;
  }
  return duty;
}

static uint32_t PFC_PWM_DutyToCompare(float duty)
{
  const float limited = PFC_PWM_LimitDuty(duty);
  float compare_f = limited * (float)PFC_HRTIM_PERIOD;

  if (compare_f < 0.0f)
  {
    compare_f = 0.0f;
  }
  if (compare_f > (float)PFC_HRTIM_PERIOD)
  {
    compare_f = (float)PFC_HRTIM_PERIOD;
  }

  return (uint32_t)(compare_f + 0.5f);
}

static uint32_t PFC_PWM_DutyCompareToAdcSyncCompare(uint32_t duty_compare)
{
  uint32_t cmp2;

  if (duty_compare == 0U)
  {
    cmp2 = PFC_ADC_SYNC_EDGE_BLANK_TICKS;
  }
  else if (duty_compare < PFC_ADC_SYNC_MIN_ON_TICKS)
  {
    cmp2 = duty_compare / 2U;
    if (cmp2 == 0U)
    {
      cmp2 = 1U;
    }
  }
  else
  {
    uint32_t target = duty_compare / 2U;
    const uint32_t turnoff_limit = duty_compare - PFC_ADC_SYNC_TURNOFF_MARGIN_TICKS;

    if (target < PFC_ADC_SYNC_EDGE_BLANK_TICKS)
    {
      target = PFC_ADC_SYNC_EDGE_BLANK_TICKS;
    }
    if (target > turnoff_limit)
    {
      target = turnoff_limit;
    }

    cmp2 = target;
  }

  if (cmp2 >= PFC_HRTIM_PERIOD)
  {
    cmp2 = PFC_HRTIM_PERIOD - 1U;
  }

  return cmp2;
}

static void PFC_PWM_RecordError(uint32_t error_code)
{
  pfc_pwm_error_code = error_code;
  PFC_PWM_AllOff();
}

static uint8_t PFC_PWM_CheckHal(HAL_StatusTypeDef status, uint32_t error_code)
{
  if (status == HAL_OK)
  {
    return 1U;
  }

  PFC_PWM_RecordError(error_code);
  return 0U;
}

static uint8_t PFC_PWM_ConfigureTimer(void)
{
  HRTIM_TimeBaseCfgTypeDef time_base_cfg = {0};
  HRTIM_TimerCtlTypeDef timer_ctl = {0};
  HRTIM_TimerCfgTypeDef timer_cfg = {0};
  HRTIM_CompareCfgTypeDef compare_cfg = {0};
  HRTIM_DeadTimeCfgTypeDef deadtime_cfg = {0};
  HRTIM_OutputCfgTypeDef output_cfg = {0};

  if (pfc_pwm_timer_configured != 0U)
  {
    return 1U;
  }

  time_base_cfg.Period = PFC_HRTIM_PERIOD;
  time_base_cfg.RepetitionCounter = 0x00U;
  time_base_cfg.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL16;
  time_base_cfg.Mode = HRTIM_MODE_CONTINUOUS;
  if (PFC_PWM_CheckHal(HAL_HRTIM_TimeBaseConfig(&hhrtim1, PFC_PWM_TIMER_INDEX, &time_base_cfg),
                       PFC_PWM_ERR_INIT_TIMEBASE) == 0U)
  {
    return 0U;
  }

  timer_ctl.UpDownMode = HRTIM_TIMERUPDOWNMODE_UP;
  timer_ctl.GreaterCMP1 = HRTIM_TIMERGTCMP1_EQUAL;
  timer_ctl.DualChannelDacEnable = HRTIM_TIMER_DCDE_DISABLED;
  if (PFC_PWM_CheckHal(HAL_HRTIM_WaveformTimerControl(&hhrtim1, PFC_PWM_TIMER_INDEX, &timer_ctl),
                       PFC_PWM_ERR_INIT_CONTROL) == 0U)
  {
    return 0U;
  }

  timer_cfg.InterruptRequests = HRTIM_TIM_IT_NONE;
  timer_cfg.DMARequests = HRTIM_TIM_DMA_NONE;
  timer_cfg.DMASrcAddress = 0x0000U;
  timer_cfg.DMADstAddress = 0x0000U;
  timer_cfg.DMASize = 0x1U;
  timer_cfg.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
  timer_cfg.InterleavedMode = HRTIM_INTERLEAVED_MODE_DISABLED;
  timer_cfg.StartOnSync = HRTIM_SYNCSTART_DISABLED;
  timer_cfg.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
  timer_cfg.DACSynchro = HRTIM_DACSYNC_NONE;
  timer_cfg.PreloadEnable = HRTIM_PRELOAD_ENABLED;
  timer_cfg.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
  timer_cfg.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
  timer_cfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_ENABLED;
  timer_cfg.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
  timer_cfg.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
  timer_cfg.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  timer_cfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_DISABLED;
  timer_cfg.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
  timer_cfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE;
  timer_cfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_NONE;
  timer_cfg.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
  timer_cfg.ReSyncUpdate = HRTIM_TIMERESYNC_UPDATE_UNCONDITIONAL;
  if (PFC_PWM_CheckHal(HAL_HRTIM_WaveformTimerConfig(&hhrtim1, PFC_PWM_TIMER_INDEX, &timer_cfg),
                       PFC_PWM_ERR_INIT_TIMER) == 0U)
  {
    return 0U;
  }

  compare_cfg.CompareValue = 0U;
  if (PFC_PWM_CheckHal(HAL_HRTIM_WaveformCompareConfig(&hhrtim1,
                                                       PFC_PWM_TIMER_INDEX,
                                                       HRTIM_COMPAREUNIT_1,
                                                       &compare_cfg),
                       PFC_PWM_ERR_INIT_COMPARE) == 0U)
  {
    return 0U;
  }

  compare_cfg.CompareValue = PFC_ADC_SYNC_EDGE_BLANK_TICKS;
  if (PFC_PWM_CheckHal(HAL_HRTIM_WaveformCompareConfig(&hhrtim1,
                                                       PFC_PWM_TIMER_INDEX,
                                                       HRTIM_COMPAREUNIT_2,
                                                       &compare_cfg),
                       PFC_PWM_ERR_INIT_COMPARE2) == 0U)
  {
    return 0U;
  }

  deadtime_cfg.Prescaler = HRTIM_TIMDEADTIME_PRESCALERRATIO_MUL8;
  deadtime_cfg.RisingValue = 408U;
  deadtime_cfg.RisingSign = HRTIM_TIMDEADTIME_RISINGSIGN_POSITIVE;
  deadtime_cfg.RisingLock = HRTIM_TIMDEADTIME_RISINGLOCK_WRITE;
  deadtime_cfg.RisingSignLock = HRTIM_TIMDEADTIME_RISINGSIGNLOCK_WRITE;
  deadtime_cfg.FallingValue = 408U;
  deadtime_cfg.FallingSign = HRTIM_TIMDEADTIME_FALLINGSIGN_POSITIVE;
  deadtime_cfg.FallingLock = HRTIM_TIMDEADTIME_FALLINGLOCK_WRITE;
  deadtime_cfg.FallingSignLock = HRTIM_TIMDEADTIME_FALLINGSIGNLOCK_WRITE;
  if (PFC_PWM_CheckHal(HAL_HRTIM_DeadTimeConfig(&hhrtim1, PFC_PWM_TIMER_INDEX, &deadtime_cfg),
                       PFC_PWM_ERR_INIT_DEADTIME) == 0U)
  {
    return 0U;
  }

  output_cfg.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
  output_cfg.SetSource = HRTIM_OUTPUTSET_NONE;
  output_cfg.ResetSource = HRTIM_OUTPUTRESET_NONE;
  output_cfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
  output_cfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  output_cfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_INACTIVE;
  output_cfg.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  output_cfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
  if (PFC_PWM_CheckHal(HAL_HRTIM_WaveformOutputConfig(&hhrtim1,
                                                      PFC_PWM_TIMER_INDEX,
                                                      PFC_PWM_OUTPUT_HIGH,
                                                      &output_cfg),
                       PFC_PWM_ERR_INIT_OUT1) == 0U)
  {
    return 0U;
  }

  if (PFC_PWM_CheckHal(HAL_HRTIM_WaveformOutputConfig(&hhrtim1,
                                                      PFC_PWM_TIMER_INDEX,
                                                      PFC_PWM_OUTPUT_LOW,
                                                      &output_cfg),
                       PFC_PWM_ERR_INIT_OUT2) == 0U)
  {
    return 0U;
  }

  if (PFC_PWM_CheckHal(HAL_HRTIM_SoftwareUpdate(&hhrtim1, PFC_PWM_TIMER_ID),
                       PFC_PWM_ERR_INIT_UPDATE) == 0U)
  {
    return 0U;
  }

  pfc_pwm_timer_configured = 1U;
  return 1U;
}

static uint8_t PFC_PWM_SetDeadTimeEnabled(uint8_t enabled, uint32_t error_code)
{
  if (enabled != 0U)
  {
    SET_BIT(hhrtim1.Instance->sTimerxRegs[PFC_PWM_TIMER_INDEX].OUTxR, HRTIM_OUTR_DTEN);
  }
  else
  {
    CLEAR_BIT(hhrtim1.Instance->sTimerxRegs[PFC_PWM_TIMER_INDEX].OUTxR, HRTIM_OUTR_DTEN);
  }

  return PFC_PWM_CheckHal(HAL_HRTIM_SoftwareUpdate(&hhrtim1, PFC_PWM_TIMER_ID),
                          error_code);
}

static uint8_t PFC_PWM_ApplyOutputConfig(uint8_t sync_mode)
{
  HRTIM_OutputCfgTypeDef output_cfg = {0};

  output_cfg.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
  output_cfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
  output_cfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  output_cfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_INACTIVE;
  output_cfg.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  output_cfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;

  if (sync_mode != 0U)
  {
    /* PA10/TB1 high-side test output: off at period start, on after Compare1, off at period. */
    output_cfg.SetSource = HRTIM_OUTPUTSET_TIMCMP1;
    output_cfg.ResetSource = HRTIM_OUTPUTRESET_TIMPER;
    if (PFC_PWM_CheckHal(HAL_HRTIM_WaveformOutputConfig(&hhrtim1,
                                                        PFC_PWM_TIMER_INDEX,
                                                        PFC_PWM_OUTPUT_HIGH,
                                                        &output_cfg),
                         PFC_PWM_ERR_CFG_SYNC_OUT1) == 0U)
    {
      return 0U;
    }

    /*
     * PA11/TB2 low-side is produced by Timer B complementary/deadtime logic in
     * sync mode. Use it only for no-load waveform confirmation.
     */
    output_cfg.SetSource = HRTIM_OUTPUTSET_NONE;
    output_cfg.ResetSource = HRTIM_OUTPUTRESET_NONE;
    if (PFC_PWM_CheckHal(HAL_HRTIM_WaveformOutputConfig(&hhrtim1,
                                                        PFC_PWM_TIMER_INDEX,
                                                        PFC_PWM_OUTPUT_LOW,
                                                        &output_cfg),
                         PFC_PWM_ERR_CFG_SYNC_OUT2) == 0U)
    {
      return 0U;
    }
  }
  else
  {
    /* PA10/TB1 forced inactive by disabling all set/reset events. */
    output_cfg.SetSource = HRTIM_OUTPUTSET_NONE;
    output_cfg.ResetSource = HRTIM_OUTPUTRESET_NONE;
    if (PFC_PWM_CheckHal(HAL_HRTIM_WaveformOutputConfig(&hhrtim1,
                                                        PFC_PWM_TIMER_INDEX,
                                                        PFC_PWM_OUTPUT_HIGH,
                                                        &output_cfg),
                         PFC_PWM_ERR_CFG_ASYNC_OUT1) == 0U)
    {
      return 0U;
    }

    // DC/AC-rectified Boost/PFC test modes use the high-side MOSFET body diode only;
    // high-side gate drive is intentionally disabled.
    /*
     * PA11/TB2 outputs independent 50 kHz PWM, period start high and Compare1
     * low. PA10/TB1 stays inactive; sync complementary logic is not used.
     */
    output_cfg.SetSource = HRTIM_OUTPUTSET_TIMPER;
    output_cfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP1;
    if (PFC_PWM_CheckHal(HAL_HRTIM_WaveformOutputConfig(&hhrtim1,
                                                        PFC_PWM_TIMER_INDEX,
                                                        PFC_PWM_OUTPUT_LOW,
                                                        &output_cfg),
                         PFC_PWM_ERR_CFG_ASYNC_OUT2) == 0U)
    {
      return 0U;
    }
  }

  if (PFC_PWM_CheckHal(HAL_HRTIM_SoftwareUpdate(&hhrtim1, PFC_PWM_TIMER_ID),
                       PFC_PWM_ERR_UPDATE_CONFIG) == 0U)
  {
    return 0U;
  }

  return 1U;
}

void PFC_PWM_InitSafe(void)
{
  pfc_pwm_error_code = 0U;
  if (PFC_PWM_ConfigureTimer() == 0U)
  {
    return;
  }

  PFC_PWM_AllOff();
  if (pfc_pwm_error_code != 0U)
  {
    return;
  }
  PFC_PWM_SetDuty(PFC_SAFE_DUTY);
}

void PFC_PWM_AllOff(void)
{
  (void)HAL_HRTIM_WaveformOutputStop(&hhrtim1,
                                     HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | PFC_PWM_OUTPUT_BOTH);
  (void)HAL_HRTIM_WaveformCountStop(&hhrtim1, HRTIM_TIMERID_TIMER_A | PFC_PWM_TIMER_ID);
  pfc_pwm_timer_running = 0U;

  __HAL_HRTIM_SETCOMPARE(&hhrtim1,
                         PFC_PWM_TIMER_INDEX,
                         HRTIM_COMPAREUNIT_1,
                         0U);
  if (HAL_HRTIM_SoftwareUpdate(&hhrtim1, PFC_PWM_TIMER_ID) != HAL_OK)
  {
    if (pfc_pwm_error_code == 0U)
    {
      pfc_pwm_error_code = PFC_PWM_ERR_UPDATE_ALLOFF;
    }
  }

  pfc_pwm_duty = 0.0f;
}

void PFC_PWM_SetDuty(float duty)
{
  const float limited = PFC_PWM_LimitDuty(duty);
  const uint32_t compare = PFC_PWM_DutyToCompare(limited);
  const uint32_t adc_sync_compare = PFC_PWM_DutyCompareToAdcSyncCompare(compare);

  if (PFC_PWM_ConfigureTimer() == 0U)
  {
    return;
  }

  __HAL_HRTIM_SETCOMPARE(&hhrtim1,
                         PFC_PWM_TIMER_INDEX,
                         HRTIM_COMPAREUNIT_1,
                         compare);
  __HAL_HRTIM_SETCOMPARE(&hhrtim1,
                         PFC_PWM_TIMER_INDEX,
                         HRTIM_COMPAREUNIT_2,
                         adc_sync_compare);

  if (pfc_pwm_timer_running == 0U)
  {
    if (PFC_PWM_CheckHal(HAL_HRTIM_SoftwareUpdate(&hhrtim1, PFC_PWM_TIMER_ID),
                         PFC_PWM_ERR_UPDATE_DUTY) == 0U)
    {
      return;
    }
  }

  pfc_pwm_duty = limited;
}

static void PFC_PWM_StartLowSide(float initial_duty)
{
  pfc_pwm_error_code = 0U;
  if (PFC_PWM_ConfigureTimer() == 0U)
  {
    return;
  }

  PFC_PWM_AllOff();
  if (pfc_pwm_error_code != 0U)
  {
    return;
  }

  // DC/AC-rectified Boost/PFC test modes use the high-side MOSFET body diode only;
  // high-side gate drive is intentionally disabled.
  /*
   * Disable Timer B deadtime so TB2/PA11 can use its own set/reset events
   * instead of the complementary generator.
   */
  if (PFC_PWM_SetDeadTimeEnabled(0U, PFC_PWM_ERR_UPDATE_DT_ASYNC) == 0U)
  {
    return;
  }

  PFC_PWM_SetDuty(initial_duty);

  if (pfc_pwm_error_code != 0U)
  {
    return;
  }

  if (PFC_PWM_ApplyOutputConfig(0U) == 0U)
  {
    return;
  }

  __HAL_HRTIM_SETCOUNTER(&hhrtim1, PFC_PWM_TIMER_INDEX, 0U);

  if (PFC_PWM_CheckHal(HAL_HRTIM_WaveformCountStart(&hhrtim1, PFC_PWM_TIMER_ID),
                       PFC_PWM_ERR_COUNT_ASYNC) == 0U)
  {
    return;
  }

  if (PFC_PWM_CheckHal(HAL_HRTIM_WaveformOutputStart(&hhrtim1, PFC_PWM_OUTPUT_LOW),
                       PFC_PWM_ERR_OUTPUT_ASYNC) == 0U)
  {
    return;
  }

  pfc_pwm_timer_running = 1U;
}

void PFC_PWM_StartAsync(void)
{
  PFC_PWM_StartLowSide(PFC_BOOST_OPENLOOP_DUTY_INIT);
}

void PFC_PWM_StartBoostClosedLoop(void)
{
  PFC_PWM_StartLowSide(PFC_BOOST_CL_DUTY_INIT);
}

void PFC_PWM_StartAcRectClosedLoop(void)
{
  PFC_PWM_StartLowSide(PFC_AC_RECT_DUTY_INIT);
}

void PFC_PWM_StartAcRectClosedLoopAtDuty(float duty)
{
  PFC_PWM_StartLowSide(duty);
}

void PFC_PWM_StartSync(void)
{
#if (PFC_ENABLE_SYNC_MODE == 0)
  pfc_pwm_error_code = PFC_PWM_ERR_SYNC_DISABLED;
  PFC_PWM_AllOff();
#else
  pfc_pwm_error_code = 0U;
  if (PFC_PWM_ConfigureTimer() == 0U)
  {
    return;
  }

  PFC_PWM_AllOff();
  if (pfc_pwm_error_code != 0U)
  {
    return;
  }

  /* PWM_S uses the CubeMX complementary/deadtime pair for waveform checking. */
  if (PFC_PWM_SetDeadTimeEnabled(1U, PFC_PWM_ERR_UPDATE_DT_SYNC) == 0U)
  {
    return;
  }

  PFC_PWM_SetDuty(PFC_SAFE_DUTY);

  if (pfc_pwm_error_code != 0U)
  {
    return;
  }

  if (PFC_PWM_ApplyOutputConfig(1U) == 0U)
  {
    return;
  }

  __HAL_HRTIM_SETCOUNTER(&hhrtim1, PFC_PWM_TIMER_INDEX, 0U);

  if (PFC_PWM_CheckHal(HAL_HRTIM_WaveformCountStart(&hhrtim1, PFC_PWM_TIMER_ID),
                       PFC_PWM_ERR_COUNT_SYNC) == 0U)
  {
    return;
  }

  if (PFC_PWM_CheckHal(HAL_HRTIM_WaveformOutputStart(&hhrtim1, PFC_PWM_OUTPUT_BOTH),
                       PFC_PWM_ERR_OUTPUT_SYNC) == 0U)
  {
    return;
  }

  pfc_pwm_timer_running = 1U;
#endif
}

void PFC_PWM_Stop(void)
{
  PFC_PWM_AllOff();
}

float PFC_PWM_GetDuty(void)
{
  return pfc_pwm_duty;
}

uint32_t PFC_PWM_GetErrorCode(void)
{
  return pfc_pwm_error_code;
}

void PFC_PWM_SetErrorCode(uint32_t error_code)
{
  pfc_pwm_error_code = error_code;
}
