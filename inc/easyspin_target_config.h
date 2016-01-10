/**
  ******************************************************************************
  * @file    easyspin_target_config.h 
  * @author  EMEA AMS-IPD Marketing & Application, Prague - VE
  * @version V1.0.1
  * @date    June-2012
  * @brief   Configuration header for easySPIN library
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
#ifndef __EASYSPIN_TARGET_CONFIG_H
#define __EASYSPIN_TARGET_CONFIG_H

/* Includes ------------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Specify volume of each transition (acceleration/deceleration) buffers 
   (arrays of uin16_t values) according to your application needs and amount
   of RAM available. */
#define MAX_ACC_TRANSITION_VOLUME	1000
#define MAX_DEC_TRANSITION_VOLUME	1000

/* Motor configuration configuration for easyspin_motion_control.c module.
   Specify the integer values in accordance with application needs. */
#define 	MIN_STEPS_PER_SEC		10	/* [steps/sec] */
#define 	MAX_STEPS_PER_SEC		200	/* [steps/sec] */

#define 	ACCELERATION_RATE		120 /* [steps/sec^2] */ 
#define 	DECELERATION_RATE		120	/* [steps/sec^2] */

#define 	DEFAULT_PERIOD_VALUE	(2500 - 1)

#define 	INT_CK_FREQ				24000000
#define 	TIM_COUNTER_CLK			10000  /* Counter freq = 10kHz */

#endif /* __EASYSPIN_TARGET_CONFIG_H */
