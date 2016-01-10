/**
 ******************************************************************************
 * @file    eMotionControl.h
 * @author  EMEA AMS-IPD Marketing & Application, Prague - VE
 * @version V1.0.1
 * @date    June-2012
 * @brief   Header for eMotionControl.c module
 ******************************************************************************
 * @copy
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EASYSPIN_MOTION_CONTROL_H
#define __EASYSPIN_MOTION_CONTROL_H

/* Includes ------------------------------------------------------------------*/
//#include "stm32f10x.h"
#include "easyspin.h"
#include "easyspin_target_config.h"

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef enum {
	Accelerating, Decelerating, Steady, Inactive
} eMotionState_TypeDef;

/* Exported macro ------------------------------------------------------------*/
#define Speed2Period(speed) ((uint16_t) TIM_COUNTER_CLK / speed)
#define Period2Speed(period) ((uint16_t) TIM_COUNTER_CLK / period)

/* Exported variables --------------------------------------------------------*/
extern int32_t currentPosition;

/* Exported functions --------------------------------------------------------*/
void eMotionControl_Init(void);
void eMotionControl_WaitWhileActive(void);
void eMotionControl_SpeedControlEngine(void);
eMotionState_TypeDef eMotionControl_GetState(void);

void eMotionControl_Run(easySPIN_Direction_TypeDef direction, uint16_t speed);
void eMotionControl_Move(easySPIN_Direction_TypeDef direction,
		uint32_t stepCount);
void eMotionControl_GoTo(int32_t targetPosition);
void eMotionControl_GoHome(void);
void eMotionControl_GoMark(void);
uint8_t eMotionControl_SoftStop(void);
void eMotionControl_ResetPos(void);
void eMotionControl_ResetDevice(void);

#endif /* __EASYSPIN_MOTION_CONTROL_H */

/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/

