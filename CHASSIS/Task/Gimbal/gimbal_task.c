#include "gimbal_task.h"
#include "chassis_task.h"
#include "sys.h"

#include "RemoteControl.h"
#include "CAN_Receive.h"
#include "pid.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "user_lib.h"

#define gimbal_free_time 50

#define GIMBAL_TASK_PRIO 25
#define GIMBAL_STK_SIZE 512
TaskHandle_t GimbalTask_Handler;

gimbal_control_state gimbal_control;
Gimbal_judge_state gumbal_judge;
extern float YAW_receive_current;

void Gimbal_task(void);
static void Gimbal_Juege_Info(void);//裁判系统数据读取
static void Gimbal_Init(gimbal_control_state *gimbal_init);//云台初始化
static void GIMBAL_Feedback_Update(gimbal_control_state *gimbal_feedback_update);//云台数据更新
int16_t get_relative_pos(int16_t raw_ecd, int16_t center_offset);//得到电机相对角度

void task_Gimbal_Create(void)
{
	xTaskCreate((TaskFunction_t)Gimbal_task,
                (const char *)"Gimbal_task",
                (uint16_t)GIMBAL_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)GIMBAL_TASK_PRIO,
                (TaskHandle_t *)&GimbalTask_Handler);
}

float Yaw_Can_Set_Current = 0;
void Gimbal_task(void)
{
	vTaskDelay(gimbal_free_time);
	Gimbal_Init(&gimbal_control);
	while(1)
	{
		if(gimbal_control.gimbal_command->commd_keyboard == 'S')
			gumbal_judge.patrol_flag = 1;
		else if(gimbal_control.gimbal_command->commd_keyboard == 'D')
			gumbal_judge.patrol_flag = 2;
		else if(gimbal_control.gimbal_command->commd_keyboard == 'F')
			gumbal_judge.patrol_flag = 3;
		Gimbal_Juege_Info();//裁判系统数据读取
		CAN_JUDGE_1();//裁判系统数据CAN发送函数
		GIMBAL_Feedback_Update(&gimbal_control);//云台数据更新
		vTaskDelay(1);  //系统延时
	}
}
/**
  * @brief 裁判系统获取
  */
static void Gimbal_Juege_Info(void)
{
	gumbal_judge.outpost_HP = Remain_outpost_HP();
	gumbal_judge.Color = is_red_or_blue();//读取红蓝
	gumbal_judge.Heat_1 = JUDGE_id1_usGetRemoteHeat17();//枪口1热量读取
	gumbal_judge.Heat_2 = JUDGE_id2_usGetRemoteHeat17();//枪口2热量读取
	gumbal_judge.ShootNum = JUDGE_ShootAllow_17mm();//剩余子弹数量
	gumbal_judge.GameSta = JUDGE_usGetState();//比赛开始标志
	gumbal_judge.robot_hurt_type = get_robot_hurt_t();//受击伤害分析
	gumbal_judge.Shoot_Speed = JUDGE_SHOOT_SPEED();
//	if(gumbal_judge.outpost_HP <= 500 && gumbal_judge.outpost_HP > 0)
//		gumbal_judge.Start_Dodge = 1;
//	else
//		gumbal_judge.Start_Dodge = 0;
}

/**
  * @brief 云台初始化
  */
static void Gimbal_Init(gimbal_control_state *gimbal_init)
{
	//电机数据指针获取
	gimbal_init->gimbal_yaw_motor.gimbal_motor_measure = get_Yaw_Gimbal_Motor_Measure_Point();
	gimbal_init->gimbal_yaw_motor.offset_ecd = Glimbal_Yaw_Offset;
	gimbal_init->gimbal_command = get_Robot_Command_t();
}


/**
  * @brief 云台数据更新
  */	
static void GIMBAL_Feedback_Update(gimbal_control_state *gimbal_feedback_update)
{
	if (gimbal_feedback_update == NULL)
	{
			return;
	}	
	static float yaw_ecd_ratio = YAW_MOTO_POSITIVE_DIR / ENCODER_ANGLE_RATIO;
  gimbal_feedback_update->gimbal_yaw_motor.relative_angle = yaw_ecd_ratio * get_relative_pos(gimbal_feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
																											gimbal_feedback_update->gimbal_yaw_motor.offset_ecd);//编码器角度
}

//得到电机相对角度
int16_t get_relative_pos(int16_t raw_ecd, int16_t center_offset)
{
    int16_t tmp = 0;

    if (center_offset >= 4096)
    {
        if (raw_ecd > center_offset - 4096)
            tmp = raw_ecd - center_offset;
        else
            tmp = raw_ecd + 8192 - center_offset;
    }
    else
    {
        if (raw_ecd > center_offset + 4096)
            tmp = raw_ecd - 8192 - center_offset;
        else
            tmp = raw_ecd - center_offset;
    }

    return tmp;
}

const Gimbal_Motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}


