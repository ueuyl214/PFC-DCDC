#ifndef PFC_MEASURE_H
#define PFC_MEASURE_H

#include "main.h"
#include "pfc_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ADC1 DMA rank mapping, do not reorder:
 * adc_dma_buf[0] = ADC1_IN12 / PB1 = VAC input voltage sample
 * adc_dma_buf[1] = ADC1_IN2  / PA1 = IL inductor/input current sample
 * adc_dma_buf[2] = ADC1_IN9  / PC3 = VBUS DC-link voltage sample
 * adc_dma_buf[3] = ADC_CHANNEL_VREFINT = internal reference sample
 */
extern volatile uint16_t adc_dma_buf[PFC_ADC_CH_COUNT];

extern uint16_t raw_vac;
extern uint16_t raw_il;
extern uint16_t raw_vbus;
extern uint16_t raw_vrefint;

extern float vac_adc_v;
extern float il_adc_v;
extern float vbus_adc_v;
extern float adc_vref_manual_v;
extern float adc_vref_est_v;
extern float adc_vref_used_v;
extern float il_zero_runtime_v;
extern uint8_t il_zero_calibrated;
extern float il_zero_cal_span_v;

extern float vac_inst_v;
extern float vac_abs_v;
extern float vbus_v;
extern float il_a;
extern float pfc_vac_fast_v;
extern float pfc_vbus_fast_v;
extern float pfc_vbus_ctrl_v;

extern float vac_rms_v;
extern float il_rms_a;
extern float pin_w;
extern float pf_est;

extern volatile uint16_t raw_il_sync;
extern volatile float il_sync_adc_v;
extern volatile float il_sync_a;
extern volatile uint8_t il_sync_valid;
extern volatile uint32_t il_sync_update_count;
extern volatile uint32_t il_sync_last_update_ms;
extern volatile uint32_t il_sync_last_count;
extern volatile uint32_t pfc_adc_sync_pulse_cnt;
extern volatile uint32_t pfc_adc_inj_eoc_cnt;

void PFC_Measure_Start(void);
void PFC_Measure_CalibrateCurrentZero(void);
float PFC_Measure_GetCurrentZeroVoltage(void);
void PFC_Measure_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* PFC_MEASURE_H */
