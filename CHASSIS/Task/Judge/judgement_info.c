#include "judgement_info.h"
#include "protocol.h"
#include "string.h"
#include "Judge_task.h"
/*****************系统数据定义**********************/
ext_game_state_t       			GameState;					//0x0001
ext_game_result_t            		GameResult;					//0x0002
ext_game_robot_survivors_t          		GameRobotSurvivors;			//0x0003
ext_event_data_t        			EventData;					//0x0101
ext_supply_projectile_action_t		SupplyProjectileAction;		//0x0102
ext_game_robot_state_t			  	GameRobotStat;				//0x0201
ext_power_heat_data_t		  		PowerHeatData;				//0x0202
ext_game_robot_pos_t				GameRobotPos;				//0x0203
ext_buff_musk_t							BuffMusk;					//0x0204
aerial_robot_energy_t				AerialRobotEnergy;			//0x0205
ext_robot_hurt_t					RobotHurt;					//0x0206
ext_shoot_data_t					ShootData;					//0x0207

ext_projectile_allowance_t       Bullet_Remaing;             //0x0208
ext_robot_command_t              robot_command;//0x303
/**用户发送**/
robot_interactive_data_t			Communication_data;	
ext_student_interactive_header_data_t  Student_Interactive;
ext_SendClientData_t      ShowData;			//客户端信息
ext_CommunatianData_t     CommuData;		//队友通信信息
/****************************************************/
uint8_t Judge_Self_ID;//当前机器人的ID
uint16_t Judge_SelfClient_ID;//发送者机器人对应的客户端ID
uint16_t cmd_id;

void judgement_data_handler(uint8_t *p_frame)       
{
  frame_header_t *p_header = (frame_header_t*)p_frame;
  memcpy(p_header, p_frame, HEADER_LEN);

  uint16_t data_length = p_header->data_length;									//数据长度
	cmd_id      = *(uint16_t *)(p_frame + HEADER_LEN);						//命令
  uint8_t *data_addr   = p_frame + HEADER_LEN + CMD_LEN;			//数据地址
  
  switch (cmd_id)
  {
   	case ID_game_state:        			//0x0001
			memcpy(&GameState, (data_addr), data_length);
		break;
		
		case ID_game_result:          		//0x0002
			memcpy(&GameResult, (data_addr), data_length);
		break;
		
		case ID_game_robot_survivors:       //0x0003
			memcpy(&GameRobotSurvivors,(data_addr), data_length);
		break;
		
		case ID_event_data:    				//0x0101
			memcpy(&EventData, (data_addr), data_length);
		break;
		
		case ID_supply_projectile_action:   //0x0102
			memcpy(&SupplyProjectileAction, (data_addr), data_length);
		break;
		case ID_game_robot_state:      		//0x0201
			memcpy(&GameRobotStat,(data_addr), data_length);
		break;
		
		case ID_power_heat_data:      		//0x0202
			memcpy(&PowerHeatData,  (data_addr), data_length);
		break;
		
		case ID_game_robot_pos:      		//0x0203
			memcpy(&GameRobotPos,(data_addr), data_length);
		break;
		
		case ID_buff_musk:      			//0x0204
			memcpy(&BuffMusk,(data_addr), data_length);
		break;
		
		case ID_aerial_robot_energy:      	//0x0205
			memcpy(&AerialRobotEnergy,(data_addr), data_length);
		break;
		
		case ID_robot_hurt:      			//0x0206
			memcpy(&RobotHurt, (data_addr), data_length);
		break;

		case ID_shoot_data:      			//0x0207
			memcpy(&ShootData,(data_addr), data_length);
		break;
		case ID_bullet_remaing:      		//0x0208
			memcpy(&Bullet_Remaing,(data_addr), data_length);
		break;
		case ID_ROBORT_COM_DATA:
			memcpy(&Communication_data,(data_addr), data_length);
		break;
		case ID_command:
			memcpy(&robot_command,(data_addr), data_length);
		break;
//		case 
		default : break;
  }
}


/**
  * @brief  判断自己红蓝方
  * @param  void
  * @retval RED   BLUE
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
bool Color;
bool is_red_or_blue(void)
{
	Judge_Self_ID = GameRobotStat.robot_id;//读取当前机器人ID
	
	if(GameRobotStat.robot_id > 10)
	{
		return BLUE;
	}
	else 
	{
		return RED;
	}
}
/**
  * @brief  读取比赛开始标志
  * @param  void
  * @retval void
  * @attention  
  */
uint8_t JUDGE_usGetState(void)
{ 
    return GameState.game_progress;
}

/**
  * @brief  读取枪口热量
  * @param  void
  * @retval id1_17mm
  * @attention  实时热量
  */
uint16_t JUDGE_id1_usGetRemoteHeat17(void)
{
    return PowerHeatData.shooter_id1_17mm_cooling_heat;
}
uint16_t JUDGE_id2_usGetRemoteHeat17(void)
{
    return PowerHeatData.shooter_id2_17mm_cooling_heat;
}

/**
  * @brief  统计发弹量
  * @param  void
  * @retval void
  * @attention
  */
portTickType shoot_time;//发射延时测试
portTickType shoot_ping;//计算出的最终发弹延迟
uint16_t ShootNum;//统计发弹量,0x0003触发一次则认为发射了一颗
float Shoot_Speed_Now = 0;
float Shoot_Speed_Last = 0;
void JUDGE_ShootNumCount(void)
{
    Shoot_Speed_Now = ShootData.bullet_speed;
    if(Shoot_Speed_Last != Shoot_Speed_Now)//因为是float型，几乎不可能完全相等,所以速度不等时说明发射了一颗弹
    {
        ShootNum++;
        Shoot_Speed_Last = Shoot_Speed_Now;
    }
    shoot_time = xTaskGetTickCount();//获取弹丸发射时的系统时间
    //shoot_ping = shoot_time - REVOL_uiGetRevolTime();//计算延迟
}
/**
  * @brief  读取发弹量
  * @param  void
  * @retval 发弹量
  * @attention 不适用于双枪管
  */
uint16_t JUDGE_usGetShootNum(void)
{
    return ShootNum;
}

uint32_t JUDGE_SHOOT_SPEED(void)
{
	return ShootData.bullet_speed;
}
/**
  * @brief  直接从裁判系统读取剩下发弹数
  * @param  uint16_t
  * @retval projectile_allowance_17mm
  * @attention  
  */
uint16_t JUDGE_ShootAllow_17mm(void)
{
	return Bullet_Remaing.projectile_allowance_17mm;
}

/**
  * @brief  判断自身ID，选择客户端ID
  * @param  void
  * @retval RED   BLUE
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
void determine_ID(void)
{
	Color = is_red_or_blue();
	if(Color == BLUE)
	{
		Judge_SelfClient_ID = 0x0110 + (Judge_Self_ID-10);//计算客户端ID
	}
	else if(Color == RED)
	{
		Judge_SelfClient_ID = 0x0100 + Judge_Self_ID;//计算客户端ID
	}
}


uint16_t Remain_outpost_HP(void)
{
	uint16_t outpost_HP = 0;
	
	Color = is_red_or_blue();
	if(Color == BLUE)
	{
		outpost_HP	= GameRobotSurvivors.blue_outpost_HP;//蓝方前哨站血量
	}
	else if(Color ==RED )
	{
		outpost_HP	= GameRobotSurvivors.red_outpost_HP; //红方前哨战血量

	}
	return outpost_HP;
}
/**
  * @brief  上传自定义数据
  * @param  void
  * @retval void
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
#define send_max_len     200
unsigned char CliendTxBuffer[send_max_len];
void JUDGE_Show_Data(void)
{
		static u8 datalength,i;
		uint8_t judge_led = 0xff;//初始化led为全绿			//可以放在用户里面  观察本身
	
		determine_ID();//判断发送者ID和其对应的客户端ID
	
		ShowData.txFrameHeader.SOF = 0xA5;						//帧头
		ShowData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(client_custom_data_t);   //字节长度
		ShowData.txFrameHeader.Seq = 0;

		memcpy(CliendTxBuffer, &ShowData.txFrameHeader, sizeof(xFrameHeader));//写入帧头数据
		append_crc8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//写入帧头CRC8校验码

		ShowData.CmdID = 0x0301;					//官方规定
	
		ShowData.dataFrameHeader.data_cmd_id = 0xD180;//发给客户端的cmd,官方固定
		//ID已经是自动读取的了
		ShowData.dataFrameHeader.send_ID 	 = Judge_Self_ID;//发送者的ID
		ShowData.dataFrameHeader.receiver_ID = Judge_SelfClient_ID;//客户端的ID，只能为发送者机器人对应的客户端

			//可以放置大小为16数据					//可以放置实时云台角度   超级电容电量的数据信息
//		ShowData.clientData.data1 = 
//		ShowData.clientData.data2 = 
//		ShowData.clientData.data3 =	

		//放在这里四个灯
		if(1)							//位置1																				
		{
			judge_led &= 0xfe;//第1位置0,变红
		}
		else
		{
			judge_led |= 0x01;//第1位置1
		}
		
		
		if(1)//位置二灯亮红
		{
			judge_led &= 0xfd;//第2位置0,变红
		}
		else
		{
			judge_led |= 0x02;//第2位置1
		}
		
		
		if(1)//位置三灯
		{
			judge_led &= 0xfb;//第3位置0,变红
		}
		else 
		{
			judge_led |= 0x04;//第3位置1
		}
		
		
		
		if(1)//位置四
		{
			judge_led &= 0xf7;//第3位置0,变红
		}
		else 
		{
			judge_led |= 0x08;//第3位置1
		}
		
		ShowData.clientData.masks = judge_led;//0~5位  0红灯,1绿灯
		
		
				//打包写入数据段
		memcpy(	
				CliendTxBuffer + 5, 
				(uint8_t*)&ShowData.CmdID, 
				(sizeof(ShowData.CmdID)+ sizeof(ShowData.dataFrameHeader)+ sizeof(ShowData.clientData))
				);	
			
		append_crc16_check_sum(CliendTxBuffer,sizeof(ShowData));//写入数据段CRC16校验码		
		
		datalength = sizeof(ShowData); 
		
		for(i = 0;i < datalength;i++)
		{
			USART_SendData(USART1,(uint16_t)CliendTxBuffer[i]);
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
		}	 
			
			
}

/**
  * @brief  发送数据给队友
  * @param  void
  * @retval void
  * @attention  
  */
#define Teammate_max_len     200
unsigned char TeammateTxBuffer[Teammate_max_len];
bool Send_Color = 0;
uint16_t send_time = 0;
void Send_to_Teammate(void)
{
		static u8 datalength,i;
		
		Send_Color = is_red_or_blue(); 				//	7，哨兵(红)；	107，哨兵(蓝)。
	
		memset(TeammateTxBuffer,0,200);
	
		CommuData.txFrameHeader.SOF = 0xA5;
		CommuData.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(robot_interactive_data_t);
		CommuData.txFrameHeader.Seq = 0;
		memcpy(TeammateTxBuffer, &CommuData.txFrameHeader, sizeof(xFrameHeader));
		append_crc8_check_sum(TeammateTxBuffer, sizeof(xFrameHeader));	
	
		CommuData.CmdID = 0x0301;
	
	   
		CommuData.dataFrameHeader.send_ID = Judge_Self_ID;//发送者的ID
	
		//判断什么样子的情况下  进行发送	或者就是发送固有信息
		CommuData.dataFrameHeader.data_cmd_id = 0x0292;//在0x0200-0x02ff之间选择
	
							//哨兵    107蓝色
		CommuData.dataFrameHeader.receiver_ID = Send_Color;//接收者ID
		
		
		CommuData.interactData.data[0] = 0;//发送的内容 ,大小不要超过变量的变量类型   

		memcpy(TeammateTxBuffer+5,(uint8_t *)&CommuData.CmdID,(sizeof(CommuData.CmdID)+sizeof(CommuData.dataFrameHeader)+sizeof(CommuData.interactData)));		
		append_crc16_check_sum(TeammateTxBuffer,sizeof(CommuData));
	
		datalength = sizeof(CommuData); 

		for(i = 0;i < datalength;i++)
		{
			USART_SendData(USART1,(uint16_t)TeammateTxBuffer[i]);
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
		}	 	

}
	
