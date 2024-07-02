#ifndef SHOOT_H
#define SHOOT_H

#define MAX_HISTROY 10
#define Fliter_windowSize 5.0f
#define BLOCK_CURRENT 10000
#define REVERSE_CURRENT 4000

void Shoot_Fric_data_process(void);
/**
 * @brief 发射初始化,会被RobotInit()调用
 * 
 */
void ShootInit();

/**
 * @brief 发射任务
 * 
 */
void ShootTask();

#endif // SHOOT_H