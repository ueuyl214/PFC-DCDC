#ifndef PFC_USER_CONFIG_H
#define PFC_USER_CONFIG_H

/* =========================================================
 * 1. ADC reference configuration
 * ========================================================= */
#define PFC_ADC_FULL_SCALE              4095.0f

/* Manual reference voltage: DMM measured VREF+ on the board. */
#define PFC_ADC_VREF_MANUAL_V           2.894f

/* Enable VREFINT-based estimate of the actual ADC reference voltage. */
#define PFC_USE_VREFINT_ESTIMATE        1

/* Fall back to the manual reference if VREFINT estimate is invalid. */
#define PFC_USE_MANUAL_VREF_FALLBACK    1

/* =========================================================
 * 2. ADC DMA buffer mapping
 * ========================================================= */
#define PFC_ADC_BUF_VAC                 0U
#define PFC_ADC_BUF_IL                  1U
#define PFC_ADC_BUF_VBUS                2U
#define PFC_ADC_BUF_VREFINT             3U

#define PFC_ADC_CH_COUNT                4U

/* =========================================================
 * 3. Measured sensor zero voltage at STM32 ADC pins
 * ========================================================= */
#define PFC_VAC_ZERO_V                  1.647f
#define PFC_IL_ZERO_V                   1.650f
#define PFC_VBUS_ZERO_V                 1.649f

/* =========================================================
 * 4. Sensor gain calibration
 * ========================================================= */
#define PFC_VAC_GAIN                    76.25f
#define PFC_VBUS_GAIN                   76.25f

#define PFC_IL_GAIN_V_PER_A             0.3280f
#define PFC_IL_SIGN                     1.0f

/* Runtime inductor-current zero calibration, RAM only. */
#define PFC_IL_ZERO_AUTO_CAL_ENABLE     1
#define PFC_IL_ZERO_CAL_SAMPLES         256U
#define PFC_IL_ZERO_CAL_MIN_V           1.45f
#define PFC_IL_ZERO_CAL_MAX_V           1.85f
#define PFC_IL_ZERO_CAL_MAX_SPAN_V      0.030f
#define PFC_IL_ZERO_CAL_SETTLE_MS       100U

/* =========================================================
 * 5. Protection thresholds
 * ========================================================= */
#define PFC_VBUS_OVP_SOFT_V             52.0f
#define PFC_VBUS_OVP_HARD_V             55.0f
#define PFC_IL_OCP_A                    2.0f
#define PFC_IL_REVERSE_OCP_A            (-0.5f)
#define PFC_IL_OCP_TRIP_COUNT           5U
#define PFC_IL_REV_TRIP_COUNT           5U

/* =========================================================
 * 6. PWM / HRTIM parameters
 * ========================================================= */
#define PFC_PWM_FREQ_HZ                 50000.0f
#define PFC_HRTIM_PERIOD                54400U
/* Conservative startup duty. Keep 0.02f before connecting the power stage. */
#define PFC_SAFE_DUTY                   0.02f
#define PFC_DUTY_OPENLOOP_MAX           0.30f

/* Low-voltage DC Boost open-loop power test duty. */
#define PFC_BOOST_OPENLOOP_DUTY_INIT    0.20f
#define PFC_BOOST_OPENLOOP_DUTY_STEP    0.01f
#define PFC_BOOST_OPENLOOP_DUTY_MAX     0.30f

/* Low-voltage DC Boost voltage-loop safety test. */
#define PFC_BOOST_CL_ENABLE             1
#define PFC_BOOST_VREF_INIT_V           12.0f
#define PFC_BOOST_VREF_TARGET_V         14.0f
#define PFC_BOOST_VREF_MAX_V            15.0f

#define PFC_BOOST_CL_DUTY_INIT          0.10f
#define PFC_BOOST_CL_DUTY_MIN           0.02f
#define PFC_BOOST_CL_DUTY_MAX           0.30f

#define PFC_BOOST_VREF_RAMP_STEP_V      0.02f
#define PFC_BOOST_DUTY_SLEW_PER_TICK    0.002f

#define PFC_BOOST_VLOOP_KP              0.02f
#define PFC_BOOST_VLOOP_KI              0.001f

#define PFC_BOOST_CL_PERIOD_MS          10U
#define PFC_BOOST_TEST_VBUS_LIMIT_V     16.0f

/* AC-rectified input Boost voltage-loop safety test. */
#define PFC_AC_RECT_CL_ENABLE              1
#define PFC_AC_TEST_DIRECT_MODE            1

#define PFC_AC_RECT_VREF_INIT_V            14.0f
#define PFC_AC_RECT_VREF_TARGET_V          18.0f
#define PFC_AC_RECT_VREF_MAX_V             22.0f

#define PFC_AC_RECT_DUTY_INIT              0.10f
#define PFC_AC_RECT_DUTY_MIN               0.02f
#define PFC_AC_RECT_DUTY_MAX               0.30f

#define PFC_AC_RECT_VREF_RAMP_STEP_V       0.02f
#define PFC_AC_RECT_DUTY_SLEW_PER_TICK     0.002f
#define PFC_AC_RECT_DUTY_RECOVER_SLEW_PER_TICK  0.02f
#define PFC_AC_RECT_CL_PERIOD_MS           1U

#define PFC_AC_RECT_VLOOP_KP               0.015f
#define PFC_AC_RECT_VLOOP_KI               0.0008f

#define PFC_AC_RECT_VIN_ENABLE_TH_V        3.0f
#define PFC_AC_RECT_VIN_DISABLE_TH_V       1.5f

#define PFC_AC_CL_STARTUP_BLANK_MS         500U
#define PFC_AC_CL_VIN_RECOVER_BLANK_MS     100U

#define PFC_AC_RECT_TEST_VBUS_LIMIT_V      22.0f

/* =========================================================
 * PFC current-loop / voltage-loop test parameters
 * ========================================================= */
#define PFC_PFC_ENABLE                    1
#define PFC_PROFILE_SOFTSTART_RUN_ENABLE  1
#define PFC_TEST_PROFILE_18VAC_SIMPLE_PFC 0
#define PFC_18VAC_FAST_ILOOP_TEST_ENABLE  0
#define PFC_DEBUG_FORCE_DC_CURRENT_LOOP   0
#define PFC_DEBUG_DC_I_USE_ISR_LOOP       1
#define PFC_DEBUG_FORCE_AC_I_TEST         0

#if (PFC_PROFILE_SOFTSTART_RUN_ENABLE != 0)
#define PFC_TEST_PROFILE_18VAC_PFC_RUN_32V 0
#elif (PFC_TEST_PROFILE_18VAC_SIMPLE_PFC != 0)
#define PFC_TEST_PROFILE_18VAC_PFC_RUN_32V 0
#else
#define PFC_TEST_PROFILE_18VAC_PFC_RUN_32V 1
#endif

#if ((PFC_PROFILE_SOFTSTART_RUN_ENABLE != 0) || \
     (PFC_TEST_PROFILE_18VAC_PFC_RUN_32V != 0) || \
     (PFC_TEST_PROFILE_18VAC_SIMPLE_PFC != 0))
#define PFC_TEST_PROFILE_18VAC_AC_I       0
#else
#define PFC_TEST_PROFILE_18VAC_AC_I       1
#endif

#if (PFC_PROFILE_SOFTSTART_RUN_ENABLE != 0)
#define PFC_RUN_ENABLE_DEFAULT            1
#elif (PFC_TEST_PROFILE_18VAC_SIMPLE_PFC != 0)
/* 18Vac -> 32V minimal double-loop PFC profile. */
#define PFC_RUN_ENABLE_DEFAULT            1
#elif (PFC_TEST_PROFILE_18VAC_PFC_RUN_32V != 0)
/* 18Vac -> 32V low-voltage PFC_RUN soft-start profile. */
#define PFC_RUN_ENABLE_DEFAULT            1
#else
#if (PFC_TEST_PROFILE_18VAC_AC_I != 0)
/* 18Vac safety profile: AC_I current-shape test only, no full PFC_RUN output. */
#define PFC_RUN_ENABLE_DEFAULT            0
#else
#define PFC_RUN_ENABLE_DEFAULT            1
#endif
#endif

/* Current-loop scheduling */
#define PFC_CURRENT_LOOP_IN_ISR_ENABLE     1
#define PFC_CURRENT_LOOP_ISR_DECIMATION    5U
#define PFC_CURRENT_LOOP_HZ                10000.0f
#define PFC_CURRENT_LOOP_PERIOD_US         100U

/* 18Vac open-loop 2% duty bring-up test. This state intentionally bypasses
 * PFC voltage/current loops and only keeps basic PWM/VBUS/sync-current guards.
 */
#define PFC_OPENLOOP_2PCT_TEST_ENABLE          0
#define PFC_OPENLOOP_2PCT_DUTY                 0.120f
#define PFC_OPENLOOP_2PCT_DUTY_MAX             0.120f
#define PFC_OPENLOOP_2PCT_VBUS_LIMIT_V         35.0f

#define PFC_OPENLOOP_2PCT_OCP_ENABLE           1
#define PFC_OPENLOOP_2PCT_OCP_A                2.50f
#define PFC_OPENLOOP_2PCT_OCP_TRIP_COUNT       5U
#define PFC_OPENLOOP_2PCT_OCP_RELEASE_A        2.00f

/* Keep the verified DC_I behavior as the default. Set this to 1 only after
 * the conservative average-current loop is stable with the current hardware.
 */
#define PFC_DC_I_FAST_RESPONSE_ENABLE      1

/* First-stage current-loop PI.
 * Estimated from L=1mH, VBUS about 48V, f_ci about 500Hz:
 * Kp ~= L * 2*pi*fci / VBUS ~= 0.065.
 * Current-loop tuning starts conservative because ADC is not yet
 * HRTIM-midpoint triggered.
 * Step 1: Kp=0.015, Ki=0.
 * Step 2: after stable current tracking, increase Ki to 0.00005 then 0.0001.
 */
#define PFC_ILOOP_KP                       0.010f
#define PFC_ILOOP_KI                       0.00015f
#define PFC_IL_CTRL_AVG_ALPHA              0.05f
#define PFC_DC_I_DUTY_BIAS                 0.07f

#define PFC_ILOOP_KP_SIMPLE                0.060f
#define PFC_ILOOP_KI_SIMPLE                0.0030f
#define PFC_IL_CTRL_AVG_ALPHA_SIMPLE       0.35f

#define PFC_ILOOP_KP_FAST                  0.060f
#define PFC_ILOOP_KI_FAST                  0.0030f
#define PFC_IL_CTRL_AVG_ALPHA_FAST         0.35f
#define PFC_DC_I_DUTY_BIAS_FAST            0.07f

#if (PFC_DC_I_FAST_RESPONSE_ENABLE != 0)
#define PFC_ILOOP_KP_EFFECTIVE             PFC_ILOOP_KP_FAST
#define PFC_ILOOP_KI_EFFECTIVE             PFC_ILOOP_KI_FAST
#define PFC_IL_CTRL_AVG_ALPHA_EFFECTIVE    PFC_IL_CTRL_AVG_ALPHA_FAST
#define PFC_DC_I_DUTY_BIAS_EFFECTIVE       PFC_DC_I_DUTY_BIAS_FAST
#else
#define PFC_ILOOP_KP_EFFECTIVE             PFC_ILOOP_KP
#define PFC_ILOOP_KI_EFFECTIVE             PFC_ILOOP_KI
#define PFC_IL_CTRL_AVG_ALPHA_EFFECTIVE    PFC_IL_CTRL_AVG_ALPHA
#define PFC_DC_I_DUTY_BIAS_EFFECTIVE       PFC_DC_I_DUTY_BIAS
#endif

#define PFC_I_LOOP_LARGE_ERR_ACCEL_ENABLE  0
#define PFC_I_LOOP_LARGE_ERR_TH_A          0.015f
#define PFC_I_LOOP_ACCEL_KI_MULT           3.0f

#define PFC_I_DUTY_SLEW_ENABLE             1
#define PFC_I_DUTY_SLEW_UP_PER_TICK        0.00025f
#define PFC_I_DUTY_SLEW_DOWN_PER_TICK      0.0100f

/* Formal average-current-mode PFC outer loop. */
#define PFC_VLOOP_PERIOD_MS                20U
#if (PFC_PROFILE_SOFTSTART_RUN_ENABLE != 0)
#define PFC_VLOOP_KP                       0.040f
#define PFC_VLOOP_KI                       0.0040f
#elif (PFC_TEST_PROFILE_18VAC_SIMPLE_PFC != 0)
#define PFC_VLOOP_KP                       0.003f
#define PFC_VLOOP_KI                       0.00001f
#elif (PFC_TEST_PROFILE_18VAC_PFC_RUN_32V != 0)
#define PFC_VLOOP_KP                       0.003f
#define PFC_VLOOP_KI                       0.00001f
#else
#define PFC_VLOOP_KP                       0.005f
#define PFC_VLOOP_KI                       0.00002f
#endif

/* PFC voltage targets, start low. */
#if (PFC_PROFILE_SOFTSTART_RUN_ENABLE != 0)
#define PFC_VBUS_REF_INIT_V                30.0f
#define PFC_VBUS_REF_TARGET_V              48.0f
#define PFC_VBUS_REF_MAX_V                 48.0f
#elif ((PFC_TEST_PROFILE_18VAC_PFC_RUN_32V != 0) || (PFC_TEST_PROFILE_18VAC_SIMPLE_PFC != 0))
#define PFC_VBUS_REF_INIT_V                24.0f
#define PFC_VBUS_REF_TARGET_V              28.0f
#define PFC_VBUS_REF_MAX_V                 30.0f
#else
#define PFC_VBUS_REF_INIT_V                18.0f
#define PFC_VBUS_REF_TARGET_V              20.0f
#define PFC_VBUS_REF_MAX_V                 22.0f
#endif
#if (PFC_PROFILE_SOFTSTART_RUN_ENABLE != 0)
#define PFC_VBUS_REF_RAMP_STEP_V           0.05f
#else
#define PFC_VBUS_REF_RAMP_STEP_V           0.005f
#endif

#define PFC_VBUS_NORMAL_MIN_V              42.0f
#define PFC_VBUS_NORMAL_MAX_V              54.0f
#define PFC_RUN_ENTER_VBUS_MIN_V           46.0f

/* Formal PFC measurement filters. */
#define PFC_VAC_FAST_ALPHA                 0.25f
#define PFC_VBUS_FAST_ALPHA                0.08f
#define PFC_VBUS_CTRL_ALPHA                0.20f
#define PFC_VBUS_LINE_AVG_PERIOD_MS        20U

/* Formal PFC soft-start and power limiting. */
#define PFC_PASSIVE_CHARGE_MS              500U
#define PFC_PASSIVE_IL_WARN_A              3.0f
#define PFC_PASSIVE_IL_FAULT_A             5.0f
#define PFC_PIN_LIMIT_INIT_W               3.0f
#define PFC_PIN_LIMIT_TARGET_W             8.0f
#define PFC_PIN_LIMIT_RAMP_W               0.05f
#define PFC_IAMP_LIMIT_ABS_MAX_A           1.20f
#define PFC_IREF_ABS_MAX_A                 3.50f

#define PFC_VIN_BLANK_V                    2.0f
#define PFC_SHAPE_MIN                      0.03f
#define PFC_HEADROOM_MARGIN_SOFTSTART_V    0.5f
#define PFC_HEADROOM_MARGIN_RUN_V          1.5f

#define PFC_ILOOP_INT_MIN_FORMAL           -0.10f
#define PFC_ILOOP_INT_MAX_SOFTSTART        0.35f
#define PFC_ILOOP_INT_MAX_RUN              0.45f

#define PFC_DFF_ENABLE                     1
#define PFC_DFF_SHAPE_ENABLE_TH            0.45f
#define PFC_DFF_BLEND_SHAPE_FULL           0.75f
#define PFC_DFF_IREF_ENABLE_TH_A           0.08f
#define PFC_DFF_MAX_SOFTSTART_SAFE         0.20f
#define PFC_DFF_MAX_RUN_SAFE               0.30f
#define PFC_DFF_MAX_SOFTSTART              0.45f
#define PFC_DFF_MAX_RUN                    0.70f
#define PFC_DUTY_MAX_SOFTSTART             0.45f
#define PFC_DUTY_MAX_RUN                   0.55f
#define PFC_PWM_ABSOLUTE_DUTY_MAX          0.80f

/* Formal PFC protection thresholds. */
#define PFC_VBUS_SOFT_LIMIT_V              52.0f
#define PFC_VBUS_PWM_OFF_V                 56.0f
#define PFC_VBUS_HARD_FAULT_V              60.0f
#define PFC_IL_SOFT_LIMIT_A                2.0f
#define PFC_IL_CTRL_FAULT_A                4.0f
#define PFC_IL_CTRL_FAULT_COUNT            8U
#define PFC_IL_ABS_HARD_FAULT_A            5.0f
#define PFC_ADC_VREF_VALID_MIN_V           2.5f
#define PFC_ADC_VREF_VALID_MAX_V           3.6f

/* Debug-stage current limits. Start very conservative. */
#define PFC_DC_CURRENT_TEST_REF_1_A        0.10f
#define PFC_DC_CURRENT_TEST_REF_2_A        0.20f
#define PFC_DC_CURRENT_TEST_REF_3_A        0.30f
#define PFC_DC_CURRENT_TEST_REF_SEL        2

#if (PFC_DC_CURRENT_TEST_REF_SEL == 1)
#define PFC_DC_CURRENT_TEST_REF_A          PFC_DC_CURRENT_TEST_REF_1_A
#elif (PFC_DC_CURRENT_TEST_REF_SEL == 2)
#define PFC_DC_CURRENT_TEST_REF_A          PFC_DC_CURRENT_TEST_REF_2_A
#else
#define PFC_DC_CURRENT_TEST_REF_A          PFC_DC_CURRENT_TEST_REF_3_A
#endif

#define PFC_DC_I_USE_DYNAMIC_FF            0
#define PFC_DC_I_VBUS_LIMIT_V              PFC_VBUS_OVP_SOFT_V
#define PFC_DC_I_DUTY_MAX                  0.60f

#define PFC_AC_I_PREBOOST_ENABLE            1
#define PFC_AC_I_PREBOOST_MARGIN_V          0.5f
#define PFC_AC_I_PREBOOST_TARGET_MARGIN_V   4.0f
#define PFC_AC_I_PREBOOST_EXIT_HOLD_MS      100U
#define PFC_AC_I_PREBOOST_VLOOP_ENABLE      1
#define PFC_AC_I_PREBOOST_VLOOP_DIV         10U
#define PFC_AC_I_PREBOOST_VREF_INIT_V       24.0f
#define PFC_AC_I_PREBOOST_VREF_TARGET_V     32.0f
#define PFC_AC_I_PREBOOST_VREF_STEP_V       0.010f
#define PFC_AC_I_PREBOOST_VLOOP_KP          0.025f
#define PFC_AC_I_PREBOOST_VLOOP_KI          0.00030f
#define PFC_AC_I_PREBOOST_IREF_MIN_A        0.00f
#define PFC_AC_I_PREBOOST_IREF_MAX_A        0.12f
#define PFC_AC_I_PREBOOST_DUTY_MAX          0.35f
#define PFC_AC_I_PREBOOST_DFF_ENABLE        1
#define PFC_AC_I_PREBOOST_DFF_MAX           0.15f
#define PFC_AC_I_PREBOOST_DUTY_SLEW_UP      0.00003f
#define PFC_AC_I_PREBOOST_DUTY_SLEW_DOWN    0.01000f
#define PFC_AC_I_PREBOOST_SOFT_ILIMIT_A     0.45f
#define PFC_AC_I_PREBOOST_EXIT_HYST_V       1.0f
#define PFC_AC_I_PASSIVE_CHARGE_ILIMIT_A    1.50f
#define PFC_AC_I_CONTROLLED_ILIMIT_A         0.60f
#define PFC_AC_I_CONTROLLED_ILIMIT_TRIP_COUNT 3U

#define PFC_AC_I_IAMP_TEST_1_A             0.04f
#define PFC_AC_I_IAMP_TEST_2_A             0.08f
#define PFC_AC_I_IAMP_TEST_3_A             0.12f
#define PFC_AC_I_IAMP_TEST_SEL             1

#if (PFC_AC_I_IAMP_TEST_SEL == 1)
#define PFC_AC_I_IAMP_TEST_A               PFC_AC_I_IAMP_TEST_1_A
#elif (PFC_AC_I_IAMP_TEST_SEL == 2)
#define PFC_AC_I_IAMP_TEST_A               PFC_AC_I_IAMP_TEST_2_A
#else
#define PFC_AC_I_IAMP_TEST_A               PFC_AC_I_IAMP_TEST_3_A
#endif

#define PFC_AC_I_IAMP_INIT_A               PFC_AC_I_IAMP_TEST_A
#define PFC_AC_I_IAMP_TARGET_A             PFC_AC_I_IAMP_TEST_A
#define PFC_AC_I_IAMP_MAX_A                PFC_AC_I_IAMP_TEST_3_A
#define PFC_AC_I_IAMP_SLEW_A               0.00005f
#define PFC_AC_I_VBUS_TARGET_V             48.0f
#define PFC_AC_I_VBUS_TAPER_START_V        49.0f
#define PFC_AC_I_VBUS_PWM_OFF_V            50.0f
#define PFC_AC_I_VBUS_FAULT_V              PFC_VBUS_OVP_SOFT_V
#define PFC_AC_I_VBUS_LIMIT_V              PFC_AC_I_VBUS_FAULT_V
#define PFC_AC_I_VIN_ENABLE_TH_V           3.0f
#define PFC_AC_I_VIN_DISABLE_TH_V          2.0f
#define PFC_AC_I_SHAPE_MIN                 0.03f
#define PFC_AC_I_IREF_MIN_A                0.0001f
#define PFC_AC_I_START_DUTY                0.006f
#define PFC_AC_I_START_DUTY_MAX            0.010f
#define PFC_AC_I_DFF_ENABLE                1
#define PFC_AC_I_DUTY_FF_MAX               0.60f
#define PFC_AC_I_DUTY_SLEW_ENABLE          1
#define PFC_AC_I_DUTY_SLEW_UP_PER_TICK     PFC_I_DUTY_SLEW_UP_PER_TICK
#define PFC_AC_I_DUTY_SLEW_DOWN_PER_TICK   PFC_I_DUTY_SLEW_DOWN_PER_TICK
#define PFC_AC_I_SHAPE_DUTY_MAX_INIT        0.20f
#define PFC_AC_I_SHAPE_DUTY_MAX_TARGET      0.60f
#define PFC_AC_I_SHAPE_DUTY_MAX_STEP        0.00002f
#define PFC_AC_I_DUTY_MAX                  0.60f
#define PFC_AC_I_SOFT_ILIMIT_A             0.45f
#define PFC_AC_I_SOFT_ILIMIT_DUTY_STEP     0.03f
#define PFC_AC_I_BOOST_MARGIN_ENABLE       1
#define PFC_AC_I_BOOST_MARGIN_V            3.0f
#define PFC_AC_I_MARGIN_BLANK_CLEAR_DUTY   1
#define PFC_AC_I_MARGIN_BLANK_FREEZE_I     1
#define PFC_PFC_RUN_DUTY_SLEW_ENABLE       1
#define PFC_PFC_RUN_DUTY_SLEW_UP_PER_TICK  0.00025f
#define PFC_PFC_RUN_DUTY_SLEW_DOWN_PER_TICK 0.020f
#if (PFC_TEST_PROFILE_18VAC_SIMPLE_PFC != 0)
#define PFC_PFC_RUN_DUTY_MAX               0.08f
#define PFC_SIMPLE_DUTY_LIMIT_INIT         0.04f
#define PFC_SIMPLE_DUTY_LIMIT_TARGET       0.08f
#define PFC_SIMPLE_DUTY_LIMIT_STEP         0.00002f
#define PFC_SIMPLE_DFF_ENABLE              0
#define PFC_SIMPLE_DFF_MAX                 0.02f
#define PFC_SIMPLE_MIN_IAMP_A              0.005f
#define PFC_SIMPLE_MIN_IREF_A              0.003f
#define PFC_SIMPLE_VBUS_HYST_V             0.30f
#define PFC_SIMPLE_NO_DEMAND_CLEAR_I       1
#define PFC_SIMPLE_NO_DEMAND_KEEP_PWM_ACTIVE 1
#define PFC_SIMPLE_HEADROOM_CAP_ENABLE     1
#define PFC_SIMPLE_HEADROOM_MIN_V          0.0f
#define PFC_SIMPLE_HEADROOM_FULL_V         3.0f
#define PFC_SIMPLE_HEADROOM_DUTY_MIN       0.00f

/* SIMPLE_PFC startup helper:
 * build VBUS above the input peak before entering sinusoidal current shaping.
 * This is not the old complex PRE/SHAPE diagnostic path; it is a minimal
 * voltage preboost stage with a strict safe-discharge headroom condition.
 */
#define PFC_SIMPLE_PREBOOST_ENABLE         1
#define PFC_SIMPLE_PREBOOST_TARGET_V       28.0f
#define PFC_SIMPLE_PREBOOST_EXIT_MARGIN_V  2.0f
#define PFC_SIMPLE_PREBOOST_SAFE_HEADROOM_V 3.0f
#define PFC_SIMPLE_PREBOOST_VREF_RAMP_STEP_V 0.005f
#define PFC_SIMPLE_PREBOOST_PERIOD_MS      1U
#define PFC_SIMPLE_PREBOOST_DUTY_MAX       0.04f
#define PFC_SIMPLE_PREBOOST_DUTY_SLEW      0.0002f
#define PFC_SIMPLE_PREBOOST_KP             0.010f
#define PFC_SIMPLE_PREBOOST_KI             0.00030f
#define PFC_SIMPLE_PREBOOST_MIN_VIN_V      2.0f
#define PFC_SIMPLE_PREBOOST_SHAPE_MIN      0.03f

#define PFC_PFC_RUN_BLANK_DUTY_HOLD_MAX    0.00f
#define PFC_PFC_RUN_BOOST_MARGIN_V         3.0f
#define PFC_PFC_RUN_IL_SOFT_LIMIT_A        0.35f
#elif (PFC_TEST_PROFILE_18VAC_PFC_RUN_32V != 0)
#define PFC_PFC_RUN_DUTY_MAX               0.12f
#define PFC_PFC_RUN_BLANK_DUTY_HOLD_MAX    0.06f
#define PFC_PFC_RUN_BOOST_MARGIN_V         3.0f
#define PFC_PFC_RUN_IL_SOFT_LIMIT_A        0.35f
#else
#define PFC_PFC_RUN_DUTY_MAX               0.18f
#define PFC_PFC_RUN_BLANK_DUTY_HOLD_MAX    0.10f
#define PFC_PFC_RUN_BOOST_MARGIN_V         3.0f
#define PFC_PFC_RUN_IL_SOFT_LIMIT_A        0.45f
#endif
#define PFC_PFC_RUN_IL_SOFT_LIMIT_ENABLE   1
#define PFC_PFC_RUN_IL_SOFT_DUTY_STEP      0.03f

#define PFC_PFC_PRECHARGE_ENABLE           1
#define PFC_PFC_PRECHARGE_USE_CURRENT_LOOP 1
#if (PFC_TEST_PROFILE_18VAC_PFC_RUN_32V != 0)
#define PFC_PFC_PRECHARGE_TARGET_V         30.0f
#define PFC_PFC_PRECHARGE_DUTY_MAX         0.20f
#else
#define PFC_PFC_PRECHARGE_TARGET_V         19.0f
#define PFC_PFC_PRECHARGE_DUTY_MAX         0.08f
#endif
#define PFC_PFC_PRECHARGE_EXIT_MARGIN_V    0.5f
#define PFC_PFC_PRECHARGE_IREF_INIT_A      0.00f
#define PFC_PFC_PRECHARGE_IREF_TARGET_A    0.08f
#define PFC_PFC_PRECHARGE_IREF_STEP_A      0.001f
#define PFC_PFC_PRECHARGE_DUTY_INIT        0.03f
#define PFC_PFC_PRECHARGE_DUTY_STEP        0.001f
#define PFC_PFC_PRECHARGE_DUTY_DOWN_STEP   0.010f
#define PFC_PFC_PRECHARGE_IL_SOFT_LIMIT_A  0.35f
#define PFC_PFC_PRECHARGE_IL_SOFT_DUTY_STEP 0.03f
#define PFC_PFC_PRECHARGE_BOOST_MARGIN_V   0.0f
#define PFC_PFC_PRECHARGE_VIN_MIN_V        2.0f
#define PFC_PFC_PRECHARGE_SHAPE_MIN        0.05f
#define PFC_PFC_PRECHARGE_DUTY_FF_ENABLE   0
#define PFC_PFC_PRECHARGE_DUTY_FF_MARGIN   0.02f
#define PFC_PFC_PRECHARGE_DUTY_FF_MIN_VIN  3.0f

#define PFC_PFC_SOFTSTART_USE_VLOOP        1
#define PFC_PFC_SOFTSTART_IAMP_MAX_A       0.10f
#define PFC_PFC_SOFTSTART_IAMP_SLEW_A      0.0001f
#define PFC_PFC_SOFTSTART_MIN_IAMP_A       0.003f

#define PFC_PFC_VIN_RMS_FF_ENABLE          0
#define PFC_PFC_VIN_NOMINAL_PEAK_V         25.5f
#define PFC_PFC_VIN_FF_MIN_SCALE           0.50f
#define PFC_PFC_VIN_FF_MAX_SCALE           2.00f

#define PFC_AC_I_VBUS_DERATE_ENABLE        1
#define PFC_AC_I_VBUS_DERATE_START_V       PFC_AC_I_VBUS_TAPER_START_V
#define PFC_AC_I_VBUS_DERATE_END_V         PFC_AC_I_VBUS_PWM_OFF_V
#define PFC_AC_CURRENT_TEST_IAMP_A         PFC_AC_I_IAMP_INIT_A
#define PFC_IAMP_CMD_INIT_A                0.00f
#define PFC_IAMP_CMD_MIN_A                 0.00f
#define PFC_IAMP_CMD_MAX_A                 0.10f
#define PFC_IAMP_MAX_A                     PFC_IAMP_CMD_MAX_A
#define PFC_IREF_MAX_A                     0.20f

/* Duty limits */
#define PFC_PFC_DUTY_MIN                   0.0f
#define PFC_PFC_DUTY_MIN_ACTIVE            0.005f
#define PFC_PFC_DUTY_MAX                   0.20f

/* PWM-synchronized current sampling */
#define PFC_USE_SYNC_IL_FOR_CURRENT_LOOP   1
#define PFC_ADC_SYNC_EDGE_BLANK_TICKS      1200U
#define PFC_ADC_SYNC_TURNOFF_MARGIN_TICKS  800U
#define PFC_ADC_SYNC_MIN_ON_TICKS          2500U
#define PFC_IL_SYNC_STALE_TIMEOUT_MS       20U

/* ADC injected timing diagnostic pulse.
 * Default fallback uses PB0 as a GPIO marker in the injected EOC callback.
 * This marks conversion completion, not the exact HRTIM CMP2 trigger instant.
 */
#define PFC_DEBUG_ADC_TIMING_PULSE_ENABLE  1
#define PFC_DEBUG_ADC_TIMING_USE_HRTIM     0
#define PFC_DEBUG_ADC_TIMING_GPIO_PORT     GPIOB
#define PFC_DEBUG_ADC_TIMING_GPIO_PIN      GPIO_PIN_0

/* Input voltage shaping / zero-cross handling */
#define PFC_VIN_ZERO_TH_V                  1.0f
#define PFC_VIN_PEAK_MIN_V                 5.0f
#define PFC_VIN_PEAK_DECAY                 0.9995f

/* Filtering */
#define PFC_VBUS_AVG_ALPHA                 0.001f

/* Debug protection */
#if (PFC_TEST_PROFILE_18VAC_SIMPLE_PFC != 0)
#define PFC_PFC_TEST_VBUS_LIMIT_V          38.0f
#elif (PFC_TEST_PROFILE_18VAC_PFC_RUN_32V != 0)
#define PFC_PFC_TEST_VBUS_LIMIT_V          36.0f
#else
#define PFC_PFC_TEST_VBUS_LIMIT_V          24.0f
#endif
#define PFC_PFC_ILIMIT_A                   0.80f
#define PFC_PFC_ILIMIT_TRIP_COUNT          5U
#define PFC_PFC_ILIMIT_RELEASE_A           0.70f
#define PFC_ISR_ILIMIT_A                   0.60f
#define PFC_ISR_ILIMIT_TRIP_COUNT          3U
#define PFC_ISR_ILIMIT_RELEASE_A           0.45f

/* Synchronous high-side drive is disabled for the first DC Boost power tests. */
#define PFC_ENABLE_SYNC_MODE             0

/* =========================================================
 * 7. Debug display
 * ========================================================= */
#define PFC_DEBUG_SHOW_ADC_VOLTAGE      1
#define PFC_DEBUG_SHOW_VREF             1
#define PFC_OLED_FIXED_PAGE             0
#define PFC_OLED_REFRESH_MS             500U
#define PFC_OLED_AC_I_DEBUG_PAGE_ENABLE 1

#endif /* PFC_USER_CONFIG_H */
