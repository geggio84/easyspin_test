/**
 ******************************************************************************
 * @file    eMotionControl.c
 * @author  EMEA AMS/IPD Marketing & Application - VE
 * @version V1.0.1
 * @date    June-2012
 * @brief   easySPIN (L6474) product related routines
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

/* Includes ------------------------------------------------------------------*/
#include "eMotionControl.h"
#include "easyspin_target_config.h"
#include "easyspin.h"
#include "gpio_lib.h"
//#include "stm32f10x.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum {
	Run_cmd, Move_cmd, GoTo_cmd, SoftStop_cmd, Ndef_cmd
} eMotionCommand_TypeDef;

/* Private define ------------------------------------------------------------*/
/* Uncomment this definition to enable speed profile analysis */
//#define SPEED_PROFILE_ANALYSIS

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint16_t PrescalerValue;
uint16_t PeriodValue;

uint16_t accelerationRate;
uint16_t decelerationRate;

volatile eMotionState_TypeDef motionState;
eMotionCommand_TypeDef commandExecuted;
uint8_t softStopRequired;

int32_t currentPosition;

/* Speed profile processing related items */
uint16_t accelerationProfile[MAX_ACC_TRANSITION_VOLUME];
uint16_t decelerationProfile[MAX_DEC_TRANSITION_VOLUME];

int16_t accelerationPhase_maxIndex;
int16_t decelerationPhase_maxIndex;
int16_t transitionIndex;

uint16_t steadyState_stepCount;
uint16_t steadyPeriod;

uint8_t accPhase_full = 0;
uint8_t decPhase_full = 0;

#ifdef SPEED_PROFILE_ANALYSIS
/* Speed profile analysis related items */
static uint16_t max_repetition_acc = 0;
static uint16_t max_repetition_dec = 0;
static uint16_t max_repetition_acc_index = 0;
static uint16_t max_repetition_dec_index = 0;

static uint16_t elem_volume_acc = 0;
static uint16_t elem_volume_dec = 0;
#endif /* SPEED_PROFILE_ANALYSIS */

/* Private function prototypes -----------------------------------------------*/
void eMotionControl_StartMovement(void);
void eMotionControl_StopMovement(void);
void eMotionControl_UpdatePeriod(uint16_t period);

#ifdef SPEED_PROFILE_ANALYSIS
void eMotionControl_LogUpdate(int16_t* transitionPhase_maxIndex,
		uint16_t transitionProfile[], uint8_t* majorPhase_full,
		uint8_t* minorPhase_full);
#endif /* SPEED_PROFILE_ANALYSIS */

void eMotionControl_UpdateSpeedProfile(int16_t* transitionPhase_maxIndex,
		uint16_t transitionProfile[], uint8_t* majorPhase_full,
		uint8_t* minorPhase_full);

uint8_t eMotionControl_TransitionCalc(uint16_t speedRate,
		uint32_t* transitionTime, int16_t* transitionPhase_maxIndex,
		uint16_t transitionProfile[], uint8_t* majorPhase_full,
		uint8_t* minorPhase_full, uint32_t stepCount);

void eMotionControl_SpeedProfileProcessing(uint32_t stepCount,
		eMotionCommand_TypeDef commandId);
int32_t eMotionControl_ConvertPosition(uint32_t abs_position_reg);

/**
 * @brief  Sets period of PWM signal on Step Clock pin
 * @param  period: signal period
 * @retval None
 */
void eMotionControl_UpdatePeriod(uint16_t period) {

	PeriodValue = period;
	//TIM_SetAutoreload(STCK_TIM_ENGAGED, PeriodValue - 1);

	/* Keep 50% duty cycle */
	//TIM_SetCompare2(STCK_TIM_ENGAGED, PeriodValue >> 1);
}

/**
 * @brief  This function clamps required speed within defined limits
 * @param  speed: input speed
 * @retval Clamped speed.
 */
uint16_t eMotionControl_SteadySpeedSetup(uint16_t speed) {
	/* Speed level setup */
	if (speed < MIN_STEPS_PER_SEC) {
		speed = MIN_STEPS_PER_SEC;
	} else if (speed > MAX_STEPS_PER_SEC) {
		speed = MAX_STEPS_PER_SEC;
	}

	steadyPeriod = Speed2Period(speed);

	return speed;
}

/**
 * @brief  Converts ABS_POSITION register value to 32b signed integer.
 * @param  abs_position_reg: register value
 * @retval converted signed integer value
 */
int32_t eMotionControl_ConvertPosition(uint32_t abs_position_reg) {
	int32_t operation_result;

	if (abs_position_reg & easySPIN_ABS_POS_SIGN_BIT_MASK) {
		/* Negative register value */
		abs_position_reg = ~abs_position_reg;
		abs_position_reg += 1;

		operation_result = (int32_t) (abs_position_reg
				& easySPIN_ABS_POS_VALUE_MASK);
		operation_result = -operation_result;
	} else {
		operation_result = (int32_t) abs_position_reg;
	}
	return operation_result;
}

/**
 * @brief  This function initializes motor motion according to current command and computed speed profile.
 * @param  None
 * @retval None
 */
void eMotionControl_StartMovement() {
	/* Configure first period */
	if (accelerationPhase_maxIndex >= 0) {
		eMotionControl_UpdatePeriod(accelerationProfile[0]);
		motionState = Accelerating;
	} else {
		eMotionControl_UpdatePeriod(steadyPeriod);
		motionState = Steady;
	}

	transitionIndex = 0;

	/* Enable interrupt generation */
	//TIM_ClearFlag(STCK_TIM_ENGAGED, TIM_FLAG_Update);
	//TIM_UpdateRequestConfig(STCK_TIM_ENGAGED, TIM_UpdateSource_Global);
	//TIM_ITConfig(STCK_TIM_ENGAGED, TIM_IT_Update, ENABLE);

	/* Enable easySPIN powerstage */
	easySPIN_Enable();

	/* Start timer */
	//TIM_Cmd(STCK_TIM_ENGAGED, ENABLE);
}

/**
 * @brief  This function stops motor motion and sets default state.
 * @param  None
 * @retval None
 */
void eMotionControl_StopMovement() {
	/* Set inactive state */
	softStopRequired = 0;
	motionState = Inactive;
	commandExecuted = Ndef_cmd;

	/* Deactivate timer */
	//TIM_Cmd(STCK_TIM_ENGAGED, DISABLE);

	/* Disable interrupt generation */
	//TIM_ITConfig(STCK_TIM_ENGAGED, TIM_IT_Update, DISABLE);
}

/**
 * @brief  Return actual state of motion control
 * @param  None
 * @retval Current state of motion control
 * 	This parameter can be one of the following values:
 *     @arg Accelerating
 *		@arg Decelerating
 *		@arg Steady
 *		@arg Inactive
 */
eMotionState_TypeDef eMotionControl_GetState() {
	return motionState;
}

/**
 * @brief  This function performs speed profile control and is meant to be called from timer Update ISR
 * @param  None
 * @retval None
 */
void eMotionControl_SpeedControlEngine() {
	switch (motionState) {
	case Accelerating: {
		/* Get upcomming sequence index */
		transitionIndex++;

		if (transitionIndex <= accelerationPhase_maxIndex) {
			/* Continue Acceleration phase */
			eMotionControl_UpdatePeriod(accelerationProfile[transitionIndex]);
		} else {
			if (steadyState_stepCount > 0 || commandExecuted == Run_cmd) {
				/* Start Steady state phase */
				transitionIndex = 0;
				motionState = Steady;
				eMotionControl_UpdatePeriod(steadyPeriod);
			} else if (decelerationPhase_maxIndex >= 0) {
				/* Start Deceleration phase */
				motionState = Decelerating;
				transitionIndex = decelerationPhase_maxIndex;
				eMotionControl_UpdatePeriod(
						decelerationProfile[transitionIndex]);
			} else {
				/* Motion process complete */
				eMotionControl_StopMovement();
			}
		}
		break;
	}
	case Steady: {
		
		if (softStopRequired) {
			
			if (decelerationPhase_maxIndex >= 0) {
				/* Start Deceleration phase */
				motionState = Decelerating;
				transitionIndex = decelerationPhase_maxIndex;
				eMotionControl_UpdatePeriod(
						decelerationProfile[transitionIndex]);
			} else {
				/* No deceleration phase defined */
				/* Motion process complete */
				eMotionControl_StopMovement();
			}
			
			break;
		}
		

		
		if (commandExecuted != Run_cmd) {
			/* Get upcomming sequence index */
			transitionIndex++;

			if (transitionIndex >= steadyState_stepCount) {
				if (decelerationPhase_maxIndex >= 0) {
					/* Start Deceleration phase */
					motionState = Decelerating;
					transitionIndex = decelerationPhase_maxIndex;
					eMotionControl_UpdatePeriod(
							decelerationProfile[transitionIndex]);
				} else {
					/* No deceleration phase defined */
					/* Motion process complete */
					eMotionControl_StopMovement();
				}
			}
		}
		break;
	}
	case Decelerating: {
		if (transitionIndex == 0) {
			/* Motion process complete */
			eMotionControl_StopMovement();
		} else {
			/* Continue Deceleration phase */
			transitionIndex--;
			eMotionControl_UpdatePeriod(decelerationProfile[transitionIndex]);
		}
		break;
	}
	default: {
		break;
	}
	}
}

#ifdef SPEED_PROFILE_ANALYSIS
/**
 * @brief  This function can be used optionally to analyse speed profile being computed.
 * 		   Execution of the code is conditioned by definition of constant SPEED_PROFILE_ANALYSIS.
 * @param  transitionPhase_maxIndex: current transition sequence index
 * @param  transitionProfile[]: pointer to transition sequence array
 * @param  majorPhase_full: major phase fulfilment switch
 * @param  minorPhase_full: minor phase fulfilment switch
 * @retval None
 */
void eMotionControl_LogUpdate(int16_t* transitionPhase_maxIndex,
		uint16_t transitionProfile[], uint8_t* majorPhase_full,
		uint8_t* minorPhase_full) {
	/* Update log */
	if (*transitionPhase_maxIndex >= 1) {
		if (transitionProfile[*transitionPhase_maxIndex]
				== transitionProfile[(*transitionPhase_maxIndex) - 1]) {

			if (majorPhase_full == &accPhase_full) {
				elem_volume_acc++;
			} else {
				elem_volume_dec++;
			}
		} else {
			if (majorPhase_full == &accPhase_full) {
				if (elem_volume_acc > max_repetition_acc) {
					max_repetition_acc_index = (*transitionPhase_maxIndex) - 1;
					max_repetition_acc = elem_volume_acc;
				}
				elem_volume_acc = 1;
			} else {
				if (elem_volume_dec > max_repetition_dec) {
					max_repetition_dec_index = (*transitionPhase_maxIndex) - 1;
					max_repetition_dec = elem_volume_dec;
				}
				elem_volume_dec = 1;
			}
		}
	} else {
		if (majorPhase_full == &accPhase_full) {
			elem_volume_acc = 1;
			max_repetition_acc = 1;
			max_repetition_acc_index = *transitionPhase_maxIndex;
		} else {
			elem_volume_dec = 1;
			max_repetition_dec = 1;
			max_repetition_dec_index = *transitionPhase_maxIndex;
		}
	}
}
#endif /* SPEED_PROFILE_ANALYSIS */

/**
 * @brief  This function computes the next period of transition sequence.
 * @param  speedRate: acceleration/deceleration rate
 * @param  transitionTime: time of transition duration
 * @param  transitionPhase_maxIndex: current transition sequence index
 * @param  transitionProfile: pointer to transition sequence array
 * @param  majorPhase_full: major phase fulfilment switch
 * @param  minorPhase_full: minor phase fulfilment switch
 * @param  stepCount: step sequence length
 *   This parameter must assigned by 0 when motion is infinite
 * @retval Processing break switch
 */
uint8_t eMotionControl_TransitionCalc(uint16_t speedRate,
		uint32_t* transitionTime, int16_t* transitionPhase_maxIndex,
		uint16_t transitionProfile[], uint8_t* majorPhase_full,
		uint8_t* minorPhase_full, uint32_t stepCount) {
	uint16_t newPeriod;
	uint8_t breakProcessing = 0;

	newPeriod = (TIM_COUNTER_CLK * TIM_COUNTER_CLK) / (MIN_STEPS_PER_SEC
			* TIM_COUNTER_CLK + speedRate * (*transitionTime));

	if (steadyPeriod < newPeriod) {
		/* Speed profile update */
		*transitionPhase_maxIndex += 1;
		transitionProfile[*transitionPhase_maxIndex] = newPeriod;
		*transitionTime += newPeriod;

#ifdef SPEED_PROFILE_ANALYSIS
		/* Update log data */
		eMotionControl_LogUpdate(transitionPhase_maxIndex, transitionProfile,
				majorPhase_full, minorPhase_full);
#endif /* SPEED_PROFILE_ANALYSIS */

		if (stepCount != 0 && (accelerationPhase_maxIndex
				+ decelerationPhase_maxIndex + 2) >= stepCount) {
			if (accPhase_full == 0 && decPhase_full == 0) {
				if (ACCELERATION_RATE > DECELERATION_RATE) {
					accPhase_full = 1;
				} else if (ACCELERATION_RATE < DECELERATION_RATE) {
					decPhase_full = 1;
				} else {
					accPhase_full = 1;
					decPhase_full = 1;
					breakProcessing = 1;
				}
			} else {
				if (transitionProfile == accelerationProfile) {
					if (transitionProfile[accelerationPhase_maxIndex]
							<= decelerationProfile[stepCount
									- accelerationPhase_maxIndex - 2]) {

						accPhase_full = 1;
						accelerationPhase_maxIndex -= 1;
						decelerationPhase_maxIndex = stepCount
								- accelerationPhase_maxIndex - 2;
						breakProcessing = 1;
					}
				} else {
					if (transitionProfile[decelerationPhase_maxIndex]
							<= accelerationProfile[stepCount
									- decelerationPhase_maxIndex - 2]) {

						decPhase_full = 1;
						decelerationPhase_maxIndex -= 1;
						accelerationPhase_maxIndex = stepCount
								- decelerationPhase_maxIndex - 2;
						breakProcessing = 1;
					}
				}
			}
		}
	} else {
		if (*minorPhase_full) {
			breakProcessing = 1;
		}
		*majorPhase_full = 1;
	}
	return breakProcessing;
}

/**
 * @brief  This function computes speed profile.
 * @param  stepCount: specifies number of steps to process
 * @param  commandId: command indentifier
 *	  This parameter can be one of the following values:
 *   	@arg Run_cmd
 *		@arg Move_cmd
 *		@arg GoTo_cmd
 *		@arg Ndef_cmd
 * @retval None
 */
void eMotionControl_SpeedProfileProcessing(uint32_t stepCount,
		eMotionCommand_TypeDef commandId) {
	uint16_t i = 0;

	uint8_t breakProcessing;

	uint32_t transitionTime_acc = 0;
	uint32_t transitionTime_dec = 0;

	accPhase_full = 0;
	decPhase_full = 0;

	accelerationPhase_maxIndex = -1;
	decelerationPhase_maxIndex = -1;

	/* Compute speed profile */
	if (commandId == Run_cmd) {
		/* Acceleration transition */
		while (i < MAX_ACC_TRANSITION_VOLUME && !accPhase_full) {
			breakProcessing = eMotionControl_TransitionCalc(ACCELERATION_RATE,
					&transitionTime_acc, &accelerationPhase_maxIndex,
					accelerationProfile, &accPhase_full, &decPhase_full,
					stepCount);

			if (breakProcessing) {
				break;
			}
			i++;
		}
	}
	else if (commandId == SoftStop_cmd) {
		/* Deceleration transition */
		while (i < MAX_DEC_TRANSITION_VOLUME && !decPhase_full) {
			breakProcessing = eMotionControl_TransitionCalc(DECELERATION_RATE,
					&transitionTime_dec, &decelerationPhase_maxIndex,
					decelerationProfile, &accPhase_full, &decPhase_full,
					stepCount);

			if (breakProcessing) {
				break;
			}
			i++;
		}
	}	else {
		for (i = 0; i < stepCount; i++) {
			/* Acceleration transition */
			if (i < MAX_ACC_TRANSITION_VOLUME && !accPhase_full) {
				breakProcessing = eMotionControl_TransitionCalc(
						ACCELERATION_RATE, &transitionTime_acc,
						&accelerationPhase_maxIndex, accelerationProfile,
						&accPhase_full, &decPhase_full, stepCount);

				if (breakProcessing) {
					break;
				}
			}

			/* Deceleration transition */
			if (i < MAX_DEC_TRANSITION_VOLUME && !decPhase_full) {
				breakProcessing = eMotionControl_TransitionCalc(
						DECELERATION_RATE, &transitionTime_dec,
						&decelerationPhase_maxIndex, decelerationProfile,
						&decPhase_full, &accPhase_full, stepCount);

				if (breakProcessing) {
					break;
				}
			}
		}
		/* Steady state duration */
		steadyState_stepCount = stepCount - ((accelerationPhase_maxIndex + 1)
				+ (decelerationPhase_maxIndex + 1));
	}

	/* Steady speed correction */
	if (accelerationPhase_maxIndex == (MAX_ACC_TRANSITION_VOLUME - 1)) {
		steadyPeriod = accelerationProfile[accelerationPhase_maxIndex];
	} else if (decelerationPhase_maxIndex == (MAX_DEC_TRANSITION_VOLUME - 1)) {
		steadyPeriod = decelerationProfile[decelerationPhase_maxIndex];
	}
}

/**
 * @brief  Initializes easySPIN target board and motion control modul
 * @param  None
 * @retval None
 */
void eMotionControl_Init() {
	easySPIN_RegsStruct_TypeDef easySPIN_RegsStruct;

	//printf("##### eMotionControl_Init BEGIN #####\n");

	/* easySPIN system init */
	easySPIN_Init();

	/* Structure initialization by default values, in order to avoid blank records */
	easySPIN_Regs_Struct_Reset(&easySPIN_RegsStruct);

	/* Program all easySPIN registers */
	easySPIN_Registers_Set(&easySPIN_RegsStruct);

	/* STM32 init */
	//eMotionControl_RCC_Configuration();
	//eMotionControl_GPIO_Configuration();
	//eMotionControl_TIM_Configuration();
	//eMotionControl_NVIC_Configuration();
	
	softStopRequired = 0;

	//printf("##### eMotionControl_Init END #####\n");
}

/**
 * @brief  This function locks until motion state gets Inactive.
 * @param  None
 * @retval None
 */
void eMotionControl_WaitWhileActive() {
	/* Wait while motor is running */
	while (eMotionControl_GetState() != Inactive)
		;
}

/**
 * @brief  Sets direction and speed of movement
 * @param  direction: Movement direction
 *   This parameter can be: DIR_Forward or DIR_Reverse.
 * @param  speed: movement speed [steps/s] - unsigned integer
 * @retval None
 */
void eMotionControl_Run(easySPIN_Direction_TypeDef direction, uint16_t speed) {

	/* Steady speed level setup */
	eMotionControl_SteadySpeedSetup(speed);

	/* Speed profile */
	eMotionControl_SpeedProfileProcessing(0, Run_cmd);

	/* Direction setup */
	easySPIN_DirectionSetup(direction);

	commandExecuted = Run_cmd;

	/* Motor activation */
	eMotionControl_StartMovement();
}

/**
 * @brief  Issues deceleration and motor stop according to defined speed profile. 
 *				 This command is only executed in Steady motion state.
 * @retval Returns zero value of command was accepted, nonzero value otherwise.
 */
uint8_t eMotionControl_SoftStop() {
	
	/* Check for steady state */
	if (motionState != Steady) {
		return 1;
	}
	
	/* Speed profile */
	eMotionControl_SpeedProfileProcessing(0, SoftStop_cmd);
	
	softStopRequired = 1;
	
	return 0;
}

/**
 * @brief  Performs steps in given count and direction in compiance with defined speed profile.
 * @param  direction: movement direction
 *   This parameter can be either DIR_Forward or DIR_Reverse
 * @param  stepCount: number of steps
 * @retval None
 */
void eMotionControl_Move(easySPIN_Direction_TypeDef direction,
		uint32_t stepCount) {
	if (stepCount != 0) {
		/* Steady speed level setup */
		eMotionControl_SteadySpeedSetup(MAX_STEPS_PER_SEC);

		commandExecuted = Move_cmd;
		/* Compute speed profile */
		eMotionControl_SpeedProfileProcessing(stepCount, commandExecuted);

		/* Direction setup */
		easySPIN_DirectionSetup(direction);

		/* Motor activation */
		eMotionControl_StartMovement();
	}
}

/**
 * @brief  This function performs movement	in order to reach given absolute position in compiance with defined speed profile.
 * @param  targetPosition: target absolute position
 * @retval None
 */
void eMotionControl_GoTo(int32_t targetPosition) {
	int32_t currentPosition;
	int32_t stepsToTake = 0;
	easySPIN_Direction_TypeDef direction;

	/* Eventually deactivate motor */
	if (motionState != Inactive) {
		eMotionControl_StopMovement();
	}

	/* Get current position */
	currentPosition = eMotionControl_ConvertPosition(easySPIN_GetParam(
			easySPIN_ABS_POS));
	stepsToTake = targetPosition - currentPosition;

	if (stepsToTake >= 0) {
		direction = DIR_Forward;
	} else {
		stepsToTake = -stepsToTake;
		direction = DIR_Reverse;
	}

	eMotionControl_Move(direction, stepsToTake);
}

/**
 * @brief  This function performs movement in order to reach zero absolute position.
 * @param  None
 * @retval None
 */
void eMotionControl_GoHome() {
	eMotionControl_GoTo(0);
}

/**
 * @brief  This function performs movement in order to reach absolute position given by easySPIN MARK register content.
 * @param  None
 * @retval None
 */
void eMotionControl_GoMark() {
	uint32_t mark;

	mark = easySPIN_GetParam(easySPIN_MARK);
	eMotionControl_GoTo(mark);
}

/**
 * @brief  Reset easySPIN ABS_POS register.
 * @param  None
 * @retval None
 */
void eMotionControl_ResetPos() {
	easySPIN_SetParam(easySPIN_ABS_POS, 0);
}

/**
 * @brief  Reset easySPIN device.
 * @param  None
 * @retval None
 */
void eMotionControl_ResetDevice() {
	/* Terminate any action */
	eMotionControl_StopMovement();

	/* Disable power stage */
	easySPIN_Disable();

	easySPIN_Reset();

	easySPIN_ReleaseReset();
}

/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/
