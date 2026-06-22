#ifndef PFC_PWM_H
#define PFC_PWM_H

#include "main.h"
#include "pfc_config.h"

#ifdef __cplusplus
extern "C" {
#endif

void PFC_PWM_InitSafe(void);
void PFC_PWM_AllOff(void);
void PFC_PWM_SetDuty(float duty);
void PFC_PWM_StartAsync(void);
void PFC_PWM_StartBoostClosedLoop(void);
void PFC_PWM_StartAcRectClosedLoop(void);
void PFC_PWM_StartAcRectClosedLoopAtDuty(float duty);
void PFC_PWM_StartSync(void);
void PFC_PWM_Stop(void);

float PFC_PWM_GetDuty(void);
uint32_t PFC_PWM_GetErrorCode(void);
void PFC_PWM_SetErrorCode(uint32_t error_code);

#ifdef __cplusplus
}
#endif

#endif /* PFC_PWM_H */
