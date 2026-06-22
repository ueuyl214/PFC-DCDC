#ifndef PFC_APP_H
#define PFC_APP_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  PFC_STATE_IDLE = 0,
  PFC_STATE_ADC_TEST,
  PFC_STATE_PWM_ASYNC_TEST,
  PFC_STATE_PWM_SYNC_TEST,
  PFC_STATE_BOOST_VOLTAGE_LOOP,
  PFC_STATE_AC_RECT_VOLTAGE_LOOP,
  PFC_STATE_DC_CURRENT_TEST,
  PFC_STATE_OPENLOOP_2PCT_TEST,
  PFC_STATE_AC_CURRENT_SHAPE_TEST,
  PFC_STATE_PFC_RUN,
  PFC_STATE_FAULT
} PFC_State_t;

typedef enum {
  PFC_FAULT_NONE = 0,
  PFC_FAULT_OVP,
  PFC_FAULT_OCP,
  PFC_FAULT_REV,
  PFC_FAULT_TEST_OVP,
  PFC_FAULT_PWM
} PFC_Fault_t;

void PFC_App_Task(void);
void PFC_App_OnInjectedCurrentIsr(void);
PFC_State_t PFC_App_GetState(void);
PFC_Fault_t PFC_App_GetFault(void);

#ifdef __cplusplus
}
#endif

#endif /* PFC_APP_H */
