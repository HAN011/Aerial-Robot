/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Test */
osThreadId_t TestHandle;
const osThreadAttr_t Test_attributes = {
  .name = "Test",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Buzzer */
osThreadId_t BuzzerHandle;
const osThreadAttr_t Buzzer_attributes = {
  .name = "Buzzer",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for instask */
osThreadId_t instaskHandle;
const osThreadAttr_t instask_attributes = {
  .name = "instask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for CMD */
osThreadId_t CMDHandle;
const osThreadAttr_t CMD_attributes = {
  .name = "CMD",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for Gimbal */
osThreadId_t GimbalHandle;
const osThreadAttr_t Gimbal_attributes = {
  .name = "Gimbal",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Chassis */
osThreadId_t ChassisHandle;
const osThreadAttr_t Chassis_attributes = {
  .name = "Chassis",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Shoot */
osThreadId_t ShootHandle;
const osThreadAttr_t Shoot_attributes = {
  .name = "Shoot",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for motorControl */
osThreadId_t motorControlHandle;
const osThreadAttr_t motorControl_attributes = {
  .name = "motorControl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal7,
};
/* Definitions for VisionTask */
osThreadId_t VisionTaskHandle;
const osThreadAttr_t VisionTask_attributes = {
  .name = "VisionTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for UIDraw */
osThreadId_t UIDrawHandle;
const osThreadAttr_t UIDraw_attributes = {
  .name = "UIDraw",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void TestTask(void *argument);
void BuzzerTask(void *argument);
void StartINSTASK(void *argument);
void _RobotCMDTask(void *argument);
void _GimbalTask(void *argument);
void _ChassisTask(void *argument);
void _ShootTask(void *argument);
void motorControlTask(void *argument);
void _VisionTask(void *argument);
void _My_UIGraphRefresh(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Test */
  TestHandle = osThreadNew(TestTask, NULL, &Test_attributes);

  /* creation of Buzzer */
  BuzzerHandle = osThreadNew(BuzzerTask, NULL, &Buzzer_attributes);

  /* creation of instask */
  instaskHandle = osThreadNew(StartINSTASK, NULL, &instask_attributes);

  /* creation of CMD */
  CMDHandle = osThreadNew(_RobotCMDTask, NULL, &CMD_attributes);

  /* creation of Gimbal */
  GimbalHandle = osThreadNew(_GimbalTask, NULL, &Gimbal_attributes);

  /* creation of Chassis */
  ChassisHandle = osThreadNew(_ChassisTask, NULL, &Chassis_attributes);

  /* creation of Shoot */
  ShootHandle = osThreadNew(_ShootTask, NULL, &Shoot_attributes);

  /* creation of motorControl */
  motorControlHandle = osThreadNew(motorControlTask, NULL, &motorControl_attributes);

  /* creation of VisionTask */
  VisionTaskHandle = osThreadNew(_VisionTask, NULL, &VisionTask_attributes);

  /* creation of UIDraw */
  UIDrawHandle = osThreadNew(_My_UIGraphRefresh, NULL, &UIDraw_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
    UNUSED(argument);
    osThreadTerminate(defaultTaskHandle); // 避免空置和切换占用cpu
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_TestTask */
/**
 * @brief Function implementing the Test thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_TestTask */
__weak void TestTask(void *argument)
{
  /* USER CODE BEGIN TestTask */
  UNUSED(argument);
    /* Infinite loop */
    for (;;) {
        osDelay(1);
    }
  /* USER CODE END TestTask */
}

/* USER CODE BEGIN Header_BuzzerTask */
/**
* @brief Function implementing the Buzzer thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BuzzerTask */
__weak void BuzzerTask(void *argument)
{
  /* USER CODE BEGIN BuzzerTask */
  UNUSED(argument);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END BuzzerTask */
}

/* USER CODE BEGIN Header_StartINSTASK */
/**
* @brief Function implementing the instask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartINSTASK */
__weak void StartINSTASK(void *argument)
{
  /* USER CODE BEGIN StartINSTASK */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartINSTASK */
}

/* USER CODE BEGIN Header__RobotCMDTask */
/**
* @brief Function implementing the CMD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header__RobotCMDTask */
__weak void _RobotCMDTask(void *argument)
{
  /* USER CODE BEGIN _RobotCMDTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END _RobotCMDTask */
}

/* USER CODE BEGIN Header__GimbalTask */
/**
* @brief Function implementing the Gimbal thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header__GimbalTask */
__weak void _GimbalTask(void *argument)
{
  /* USER CODE BEGIN _GimbalTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END _GimbalTask */
}

/* USER CODE BEGIN Header__ChassisTask */
/**
* @brief Function implementing the Chassis thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header__ChassisTask */
__weak void _ChassisTask(void *argument)
{
  /* USER CODE BEGIN _ChassisTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END _ChassisTask */
}

/* USER CODE BEGIN Header__ShootTask */
/**
* @brief Function implementing the Shoot thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header__ShootTask */
__weak void _ShootTask(void *argument)
{
  /* USER CODE BEGIN _ShootTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END _ShootTask */
}

/* USER CODE BEGIN Header_motorControlTask */
/**
* @brief Function implementing the motorControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motorControlTask */
__weak void motorControlTask(void *argument)
{
  /* USER CODE BEGIN motorControlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END motorControlTask */
}

/* USER CODE BEGIN Header__VisionTask */
/**
* @brief Function implementing the VisionTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header__VisionTask */
__weak void _VisionTask(void *argument)
{
  /* USER CODE BEGIN _VisionTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END _VisionTask */
}

/* USER CODE BEGIN Header__My_UIGraphRefresh */
/**
* @brief Function implementing the UIDraw thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header__My_UIGraphRefresh */
__weak void _My_UIGraphRefresh(void *argument)
{
  /* USER CODE BEGIN _My_UIGraphRefresh */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END _My_UIGraphRefresh */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

