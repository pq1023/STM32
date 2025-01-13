#ifndef __JUDGEMENT_INFO_H__
#define __JUDGEMENT_INFO_H__
/**
  **********************************(C) NCIST ************************************
  * @file       judgement_info.c/h
  * @brief    	更改裁判系统至2023版本，增加了一些新的裁判系统变量，
	*							大体结构没有改变，大多为直接引用裁判系统数据
  * @history
  *   Date            Author          Modification
  *   2023/4/26   		PengQi            1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  **********************************(C) NCIST ************************************
  */
#include "stm32f4xx.h"            
#include "stdint.h"
#include <stdbool.h>

#define		JUDGE_23		23			    //23年裁判系统
#define		JUDGE_VERSION	JUDGE_23

#if JUDGE_VERSION==JUDGE_23

//帧头详细定义
#define    LEN_HEADER    5        //帧头长
#define    LEN_CMDID     2        //命令码长度
#define    LEN_TAIL      2	      //帧尾CRC16
#define BLUE  0
#define RED   1
/* 自定义帧头 */
typedef __packed struct
{
	uint8_t  SOF;				//起始字节，固定0XA5
	uint16_t DataLength;//帧数据长度
	uint8_t  Seq;				//包序号
	uint8_t  CRC8;			//CRC8校验值
	u16 CMD_ID;					//命令ID
} xFrameHeader;// UI



//命令码ID,用来判断接收的是什么数据
typedef enum
{ 
	ID_game_state       		      	= 0x0001,//比赛状态数据
	ID_game_result 	   				      = 0x0002,//比赛结果数据
	ID_game_robot_survivors       	= 0x0003,//比赛机器人存活数据
	ID_event_data  					        = 0x0101,//场地事件数据 
	ID_supply_projectile_action   	= 0x0102,//场地补给站动作标识数据
	ID_referee_warning              = 0x0104,//裁判警告信息
	ID_supply_dart_remaing 		     	= 0x0105,//飞镖发射口倒计时数据
	ID_game_robot_state    		    	= 0x0201,//机器人状态数据
	ID_power_heat_data    			    = 0x0202,//实时功率热量数据
	ID_game_robot_pos        		    = 0x0203,//机器人位置数据
	ID_buff_musk					          = 0x0204,//机器人增益数据
	ID_aerial_robot_energy			    = 0x0205,//空中机器人能量状态数据
	ID_robot_hurt					          = 0x0206,//伤害状态数据
	ID_shoot_data					          = 0x0207,//实时射击数据
	ID_bullet_remaing				        = 0x0208,//子弹剩余发射量数据
	ID_rfid_status                  = 0x0209,//机器人RFID状态
	ID_dart_client_cmd              = 0x020A,//飞镖机器人客户端指令书
	ID_ground_robot_position        = 0x020B,//己方机器人在以红方补给站为坐标原点建立的坐标系中的位置
	ID_radar_mark_data              = 0x020C,//对方机器人被标记的进度
	ID_ROBORT_COM_DATA				      = 0X0301,//机器人间交互数据10HZ
	ID_custom_robot_data            = 0x0302,//自定义控制器通过图传链路向对应的机器人发送的数据
	ID_command											=	0x0303,
	ID_custom_client_data           = 0x0306,//自定义控制器模拟键鼠操作
	ID_map_sentry_data              = 0x0307,//哨兵机器人可以向己方空中机器人选手端发送路径坐标数据，该路径会在其小地图上显示
} CmdID;

/* ID: 0x0001  Byte:  3    比赛状态数据 */
typedef __packed struct 
{ 
	uint8_t game_type : 4;				//比赛类型
	uint8_t game_progress : 4;		//当前比赛状态
	uint16_t stage_remain_time;		//当前阶段剩余时间、
	uint64_t SyncTimeStamp;       //机器人收到指令得时间
} ext_game_state_t; 

/* ID: 0x0002  Byte:  1    比赛结果数据 */
typedef __packed struct 
{ 
	uint8_t winner;
} ext_game_result_t; 

/* ID: 0x0003  Byte:  2    比赛机器人存活数据 */
typedef __packed struct 
{ 
 uint16_t red_1_robot_HP;			//红 1 英雄机器人血量，未上场以及罚下血量为 0
 uint16_t red_2_robot_HP; 		//红 2 工程机器人血量
 uint16_t red_3_robot_HP; //红 3 步兵机器人血量
 uint16_t red_4_robot_HP; //红 4 步兵机器人血量
 uint16_t red_5_robot_HP; //红 5 步兵机器人血量
 uint16_t red_7_robot_HP; //红 7 哨兵机器人血量
 uint16_t red_outpost_HP;//红方前哨战血量
 uint16_t red_base_HP; //红方基地血量
 uint16_t blue_1_robot_HP; //蓝 1 英雄机器人血量
 uint16_t blue_2_robot_HP; //蓝 2 工程机器人血量
 uint16_t blue_3_robot_HP; //蓝 3 步兵机器人血量
 uint16_t blue_4_robot_HP; //蓝 4 步兵机器人血量
 uint16_t blue_5_robot_HP; //蓝 5 步兵机器人血量
 uint16_t blue_7_robot_HP; //蓝 7 哨兵机器人血量
 uint16_t blue_outpost_HP;//蓝方前哨站血量
 uint16_t blue_base_HP;//蓝方基地血量
	
} ext_game_robot_survivors_t; 


/* ID: 0x0101  Byte:  4    场地事件数据 */
typedef __packed struct 
{ 
	uint32_t event_type;
} ext_event_data_t; 

/* ID: 0x0102  Byte:  3    场地补给站动作标识数据 */
typedef __packed struct 
{ 
	uint8_t supply_projectile_id;	//补给站口 ID：
	uint8_t supply_robot_id;					//补弹机器人 ID
	uint8_t supply_projectile_step;		//出弹口开闭状态
	uint8_t supply_projectile_num;   //补弹数量
	
} ext_supply_projectile_action_t; 


/* ID: 0X0104  Byte:  1     裁判警告信息，警告后发送*/
typedef __packed struct 
{
	uint8_t level;           //警告等级：1黄牌；2红牌；3判负。
	uint8_t foul_robot_id;	 //犯规机器人ID
}ext_referee_warning_t;

/* ID: 0X0105  Byte:  1    飞镖发射口倒计时 发送频率1hz */
typedef __packed struct
{
 uint8_t dart_remaining_time;	//15s 倒计时
} ext_dart_remaining_time_t;

/* ID: 0X0201  Byte: 15    机器人状态数据 发送频率10hz */
typedef __packed struct 
{ 
	uint8_t robot_id;                       //机器人ID，可用来校验发送
	uint8_t robot_level;                    //1一级，2二级，3三级
	uint16_t current_HP;                     //机器人当前血量
	uint16_t maximum_HP;                        //机器人上限血量
	
	uint16_t shooter_id1_17mm_cooling_rate; //机器人1号17mm发射机构冷却值
	uint16_t shooter_id1_17mm_cooling_limit;//机器人1号17mm发射机够热量上限
	uint16_t shooter_id1_17mm_speed_limit;  //机器人1号17mm发射机构射速上限
	uint16_t shooter_id2_17mm_cooling_rate; //机器人2号17mm发射机构冷却值
	uint16_t shooter_id2_17mm_cooling_limit;//机器人2号17mm发射机够热量上限
	uint16_t shooter_id2_17mm_speed_limit;  //机器人2号17mm发射机构射速上限

	uint16_t shooter_id1_42mm_cooling_rate; //42mm发射机构冷却值
	uint16_t shooter_id1_42mm_cooling_limit;//42mm发射机构热量上限
	uint16_t shooter_id1_42mm_speed_limit;  //42mm发射机构射速上限
	
	uint16_t chassis_power_limit;           //底盘功率上限
	uint8_t mains_power_gimbal_output : 1;  //gimbal口24V输出判定（1为有输出，0为无输出）
	uint8_t mains_power_chassis_output : 1; //chassis口24V输出判定（1为有输出，0为无输出）
	uint8_t mains_power_shooter_output : 1; //shooter口24V输出判定（1为有输出，0为无输出）
} ext_game_robot_state_t; 

/* ID: 0X0202  Byte: 14    实时功率热量数据 ，发送频率50hz*/
typedef __packed struct
{
	uint16_t chassis_volt;//底盘输出电压 单位 毫伏
	uint16_t chassis_current;//底盘输出电流 单位 毫安
	float chassis_power;//底盘输出功率 单位 W 瓦
	uint16_t chassis_power_buffer;//底盘功率缓冲 单位 J 焦耳 备注：飞坡根据规则增加至 250J
	uint16_t shooter_id1_17mm_cooling_heat;//1 号 17mm 枪口热量
	uint16_t shooter_id2_17mm_cooling_heat;//2 号 17mm 枪口热量
	uint16_t shooter_id1_42mm_cooling_heat;//42mm 枪口热量
} ext_power_heat_data_t;


/* ID: 0x0203  Byte: 16    机器人位置数据 */
typedef __packed struct 
{   
	float x;   //位置 x 坐标，单位 m
	float y;   //位置 y 坐标，单位 m
	float z;   //位置 z 坐标，单位 m
	float yaw; //位置枪口，单位度
} ext_game_robot_pos_t; 


/* ID: 0x0204  Byte:  1    机器人增益数据 */
typedef __packed struct 
{ 
	uint8_t recovery_buff;//机器人回血增益（百分比，值为 10 意为每秒回复 10%最大血量）
	uint8_t cooling_buff;//机器人枪口冷却倍率（直接值，值为 5 意味着 5 倍冷却）
	uint8_t defence_buff;//机器人防御增益（百分比，值为 50 意为 50%防御增益）
	uint16_t attack_buff;//机器人攻击增益（百分比，值为 50 意为 50%攻击增益）
} ext_buff_musk_t; 


/* ID: 0x0205  Byte:  3    空中机器人能量状态数据 */
typedef __packed struct 
{ 
	 uint8_t airforce_status;
	 uint8_t time_remain;
} aerial_robot_energy_t; 


/* ID: 0x0206  Byte:  1    伤害状态数据 */
typedef __packed struct 
{ 
//bit 0-3：当血量变化类型为装甲伤害，代表装甲 ID，其中数值为 0-4 号代表机器人的五个装甲片，
//	其他血量变化类型，该变量数值为 0。
	//装甲板0 ：显示0
	//装甲板1 ：显示1
	//装甲板2 ：显示2
	//装甲板3 ：显示3
	//装甲板4 ：显示4
	uint8_t armor_id : 4; 
	
//0x0 装甲伤害扣血；接受到的数据  0
//0x1 模块掉线扣血；接受到的数据  1
//0x2 超射速扣血；接受到的数据  2
//0x3 超枪口热量扣血；接受到的数据  3
//0x4 超底盘功率扣血；接受到的数据  4
//0x5 装甲撞击扣血	接受到的数据  5
	uint8_t hurt_type : 4; 
	
} ext_robot_hurt_t; 

/* ID: 0x0207  Byte:  6    实时射击数据 */
typedef __packed struct 
{ 
 uint8_t bullet_type;// 子弹类型: 1：17mm 弹丸 2：42mm 弹丸
 uint8_t shooter_id;//发射机构 ID：1：1号17mm 发射机构  2：2号17mm 发射机构 3：42mm发射机构
 uint8_t bullet_freq;//子弹射速 单位 Hz
 float bullet_speed;//子弹初射速 单位 m/s
} ext_shoot_data_t; 

/* ID: 0x0208  Byte:  2    允许发单量剩余数据 */
typedef __packed struct
{
  uint16_t projectile_allowance_17mm;//17mm允许发弹量
  uint16_t projectile_allowance_42mm;//42mm允许发弹量
  uint16_t remaining_gold_coin;			 //剩余金币数量	
} ext_projectile_allowance_t;

/* ID: 0x0209  Byte:  4   RFID状态读取（状态不完全代表对应的增益或处罚状态，例如地方已占领的高地增益点，不能获取对应的增益效果）*/
typedef __packed struct
{
/*********************RFID状态指示***********************/
/*
bit 位值为 1/0 的含义：是否已检测到该增益点 RFID
 bit0：己方基地增益点
 bit1：己方环形高地增益点
 bit2：对方环形高地增益点
 bit3：己方 R3/B3 梯形高地增益点
 bit4：对方 R3/B3 梯形高地增益点
 bit5：己方 R4/B4 梯形高地增益点
 bit6：对方 R4/B4 梯形高地增益点
 bit7：己方能量机关激活点
 bit8：己方飞坡增益点（靠近己方一侧飞坡前）
 bit9：己方飞坡增益点（靠近己方一侧飞坡后）
 bit10：对方飞坡增益点（靠近对方一侧飞坡前）
 bit11：对方飞坡增益点（靠近对方一侧飞坡后）
 bit12：己方前哨站增益点
 bit13：己方补血点（检测到任一均视为激活）
 bit14：己方哨兵巡逻区
 bit15：对方哨兵巡逻区
 bit16：己方大资源岛增益点
 bit17：对方大资源岛增益点
 bit18：己方控制区
 bit19：对方控制区
 bit20-31：保留
注：基地增益点，高地增益点，飞坡增益点，前哨站增益点，资源岛增益点，
补血点，控制区和哨兵巡逻区的 RFID 仅在赛内才生效，即在赛外，即使检
测到对应的 RFID 卡，对应值也将为 0。
*/
  uint32_t rfid_status;
} ext_rfid_status;


/* ID: 0x020A  Byte:  12   飞镖机器人客户端指令*/
typedef __packed struct
{
 uint8_t dart_launch_opening_status;//当前飞镖发射站的状态：1：关闭；2：正在开启或者关闭中 0：已经开启
 uint8_t dart_attack_target;//飞镖的打击目标：（默认为前哨站）0：前哨站；1：基地
 uint16_t target_change_time;//切换打击目标时的比赛剩余时间，单位：s，无未切换动作默认为 0。
 uint16_t latest_launch_cmd_time;//最后一次操作手确定发射指令时的比赛剩余时间，单位：s，初始值为 0。
} ext_dart_client_cmd_t;

/* ID: 0x020B  Byte: 40   己方机器人位置坐标 单位：m*/
typedef __packed struct
{
  float hero_x;        //己方英雄机器人X轴位置坐标
	float hero_y;        //己方英雄机器人y轴位置坐标
	float engineer_x;    //己方工程机器人X轴位置坐标
	float engineer_y;    //己方工程机器人y轴位置坐标
	float standard_3_x;  //己方3号步兵机器人X轴位置坐标
	float standard_3_y;  //己方3号步兵机器人y轴位置坐标
	float standard_4_x;  //己方4号步兵机器人X轴位置坐标
	float standard_4_y;  //己方4号步兵机器人y轴位置坐标
	float standard_5_x;  //己方5号步兵机器人X轴位置坐标
	float standard_5_y;  //己方5号步兵机器人y轴位置坐标
} ground_robot_position_t;

/* ID: 0x020C  Byte: 6   对方机器人被标记进度0~120*/
typedef __packed struct
{
	uint8_t mark_hero_progress;        //对方英雄被标记进度0-120
	uint8_t mark_engineer_progress;    //对方工程被标记进度
	uint8_t mark_standard_3_progress;  //对方三号被标记进度
	uint8_t mark_standard_4_progress;  //对方四号被标记进度
	uint8_t mark_standard_5_progress;  //对方五号被标记进度  
	uint8_t mark_sentry_progress;      //对方哨兵被标记进度
}radar_mark_data_t;


/****************************************************机器人间的交互数据********************************************************/
/*交互数据接收信息：0x0301*/
typedef __packed struct
{
 uint16_t data_cmd_id;
 uint16_t send_ID;
 uint16_t receiver_ID;
 uint8_t user_data[113];
}ext_student_interactive_header_data_t;
/* ID: 0x0302  Byte: 30   自定义控制器通过图传链路向对应的机器人发送的数据*/
typedef __packed struct
{
	uint8_t data[1];      //自定义发送数据，发送数据不超过30位
}custom_robot_data_t;

/* ID: 0x0303 协议两次发送间隔不得低于3秒，截止2023/4/2该协议官方未公布具体内容*/
typedef __packed struct
{
	float target_position_x;
	float target_position_y;
	float target_position_z;
	uint8_t commd_keyboard;
	uint16_t target_robot_ID;
} ext_robot_command_t;


//typedef _packed struct
//{
//uint16_t target_robot_id;
//float target_position_x;
//float target_position_y;
//}map_robot_data_t;
/* ID: 0x0306 自定义控制器模拟键鼠操作*/
typedef __packed struct
{
	uint16_t key_value;     //键盘键值bit0-7按键1，bit8-15按键2
	uint16_t x_position:12; //鼠标x轴像素位置
	uint16_t mouse_left:4;  //鼠标左键状态
	uint16_t y_position:12; //鼠标y轴像素位置
	uint16_t mouse_right:4; //鼠标右键状态
	uint16_t reserved;      //保留位
}custom_client_data_t; 

/* ID: 0x0307  哨兵机器人可以向己方空中机器人选手端发送路径坐标数据，该路径会在其小地图上显示*/
typedef __packed struct
{
	uint8_t intention;     					 //1到目标点攻击2到目标点攻击3移动到目标点
	uint16_t startt__position_x:12;  //路径起点x轴的坐标，单位：dm
	uint16_t startt__position_y:12;  //路径起点y轴的坐标，单位：dm
	int8_t delta_x[49];              //路径点x轴增量数组（共49个新点位），单位：dm
	int8_t delta_y[49];              //路径点y轴增量数组（共49个新点位），单位：dm
}map_sentry_data_t; 

///////////////////////////////////////////////////
typedef struct 
{
  uint8_t	Flag;				  //帧头位数
	uint16_t data_len;	  //数据长度
	uint16_t data_cnt;		//自加位置
	uint8_t	data;
}Judge_FLAG;
/* 
	
	交互数据，包括一个统一的数据段头结构，
	包含了内容 ID，发送者以及接受者的 ID 和内容数据段，
	整个交互数据的包总共长最大为 128 个字节，
	减去 frame_header,cmd_id,frame_tail 以及数据段头结构的 6 个字节，
	故而发送的内容数据段最大为 113。
	整个交互数据 0x0301 的包上行频率为 10Hz。

	机器人 ID：
	1，英雄(红)；
	2，工程(红)；
	3/4/5，步兵(红)；
	6，空中(红)；
	7，哨兵(红)；
	11，英雄(蓝)；
	12，工程(蓝)；
	13/14/15，步兵(蓝)；
	16，空中(蓝)；
	17，哨兵(蓝)。 
	客户端 ID： 
	0x0101 为英雄操作手客户端( 红) ；
	0x0102 ，工程操作手客户端 ((红 )；
	0x0103/0x0104/0x0105，步兵操作手客户端(红)；
	0x0106，空中操作手客户端((红)； 
	0x0111，英雄操作手客户端(蓝)；
	0x0112，工程操作手客户端(蓝)；
	0x0113/0x0114/0x0115，操作手客户端步兵(蓝)；
	0x0116，空中操作手客户端(蓝)。 
*/

typedef __packed struct
{
	ext_student_interactive_header_data_t interactive_data;
  float data1;
  float data2;
  float data3;
	u8    mask;
} client_show_data_t;

typedef __packed struct
{
	xFrameHeader		FrameHeader;
	CmdID			    CmdID;

  __packed union
	{ 
		ext_game_state_t 			game_information;
		ext_robot_hurt_t  			blood_changed_data;
		ext_shoot_data_t       		real_shoot_data;
		ext_power_heat_data_t   	real_powerheat_data;
		ext_event_data_t      		rfid_data;
		ext_game_result_t      		game_result_data;
		ext_buff_musk_t         			get_buff_data;
		ext_game_robot_pos_t		gameRobotPos;
		client_show_data_t  		client_show_data;
	}Data;
	uint16_t		CRC16;	//所有数据CRC校验
}Dateframe_t;//数据帧

/* 交互数据接收信息：0x0301  */

/* 
	客户端 客户端自定义数据：cmd_id:0x0301。内容 ID:0xD180
	发送频率：上限 10Hz


	1.	客户端 客户端自定义数据：cmd_id:0x0301。内容 ID:0xD180。发送频率：上限 10Hz 
	字节偏移量 	大小 	说明 				备注 
	0 			2 		数据的内容 ID 		0xD180 
	2 			2 		送者的 ID 			需要校验发送者机器人的 ID 正确性 
	4 			2 		客户端的 ID 		只能为发送者机器人对应的客户端 
	6 			4 		自定义浮点数据 1 	 
	10 			4 		自定义浮点数据 2 	 
	14 			4 		自定义浮点数据 3 	 
	18 			1 		自定义 8 位数据 4 	 

*/
typedef __packed struct 
{ 
	float data1; 
	float data2; 
	float data3; 
	uint8_t masks; 
} client_custom_data_t;


/* 
	学生机器人间通信 cmd_id 0x0301，内容 ID:0x0200~0x02FF
	交互数据 机器人间通信：0x0301。
	发送频率：上限 10Hz  

	字节偏移量 	大小 	说明 			备注 
	0 			2 		数据的内容 ID 	0x0200~0x02FF 
										可以在以上 ID 段选取，具体 ID 含义由参赛队自定义 
	
	2 			2 		发送者的 ID 	需要校验发送者的 ID 正确性， 
	
	4 			2 		接收者的 ID 	需要校验接收者的 ID 正确性，
										例如不能发送到敌对机器人的ID 
	
	6 			n 		数据段 			n 需要小于 113 

*/
typedef __packed struct 
{ 
	uint8_t data[112]; //数据段,n需要小于113
} robot_interactive_data_t;

/*-------------1. 交互数据接收信息： 0x0301。 发送频率：上限 10Hz---------
字节偏移量 大小 说明 备注
0 2 数据段的内容ID
2 2 发送者的ID   需要校验发送者的 ID 正确性，例如红 1 发送给红 5，此项需要校验红 1
4 2 接收者的ID  需要校验接收者的 ID 正确性，例如不能发送到敌对机器人的ID
6 x 内容数据段x 最大为 113
----*/

//帧头  命令码   数据段头结构  数据段   帧尾
//上传客户端
typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//帧头
	uint16_t		 						CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	client_custom_data_t  					clientData;//数据段
	uint16_t		 						FrameTail;//帧尾
}ext_SendClientData_t;
//客户端删除图形 机器人间通信：0x0301
typedef __packed struct
{
uint8_t operate_tpye; 
uint8_t layer; 
} ext_client_custom_graphic_delete_t;
//图形数据
typedef __packed struct
{ 
	uint8_t graphic_name[3]; 
	uint32_t operate_tpye:3; 
	uint32_t graphic_tpye:3; 
	uint32_t layer:4; 
	uint32_t color:4; 
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10; 
	uint32_t start_x:11; 
	uint32_t start_y:11; 
	uint32_t radius:10; 
	uint32_t end_x:11; 
	uint32_t end_y:11; 
} graphic_data_struct_t;

//客户端绘制一个图形 机器人间通信：0x0301
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct;
} ext_client_custom_graphic_single_t;
//客户端绘制两个图形 机器人间通信：0x0301
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[2];
} ext_client_custom_graphic_double_t;
//客户端绘制五个图形 机器人间通信：0x0301
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[5];
} ext_client_custom_graphic_five_t;
//客户端绘制字符 机器人间通信
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct;
	uint8_t data[30];
} ext_client_custom_character_t;
//客户端绘制七个图形 机器人间通信
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[7];
} ext_client_custom_graphic_seven_t;
//客户端下发信息

typedef __packed struct
{
	int16_t mouse_x;
	int16_t mouse_y;
	int16_t mouse_z;
	int8_t left_button_down;
	int8_t right_button_down;
	uint16_t keyboard_value;
	uint16_t reserved;
} ext_robot_commands_t;

typedef __packed struct
{
  uint8_t  data[64];
} user_to_server_t;

typedef __packed struct
{
  uint8_t  data[32];
} server_to_user_t;

/** 
  * @brief  the data structure receive from judgement
  */
/* data send (forward) */
/* data receive */


#define INFANTRY 1
#define HERO 2
#define ENGINEER 3
#define SENTINEL 4
typedef struct
{
	int16_t Voltage;
	int16_t Current;
	int16_t Supply_Num;
	int16_t ShooterHeat_17mm;
	int16_t ShooterHeat_42mm;
	int16_t Power;
	int16_t PowerBuffer;
	uint8_t level;
	uint8_t hurt_type;
}JudgementType;

typedef struct
{
	uint8_t 	graph_operate_type;
	uint8_t 	graph_type;
	uint8_t 	graph_name[5];
	uint8_t 	graph_color;
	uint8_t		graph_line_width;
	uint16_t graph_start_x;
	uint16_t graph_start_y;
	uint16_t graph_radius;
	uint16_t graph_dst_x;
	uint16_t graph_dst_y;
	uint8_t text_lengh;
	uint8_t text[30];
}Graph_Data_Type;

typedef struct
{
	int16_t 		ShootLevel;
	int16_t 		SuperCapacitorComment;
	float 					bullet_can_shoot;
	uint8_t 		State_Mask;
	uint8_t 		SuperCapacitorState;
	Graph_Data_Type Graph_Data ;
}SendToJudgementDataType;

//机器人交互信息
typedef __packed struct
{
	xFrameHeader   							txFrameHeader;//帧头
	uint16_t								CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	robot_interactive_data_t  	 			interactData;//数据段
	uint16_t		 						FrameTail;//帧尾
}ext_CommunatianData_t;


#endif
bool is_red_or_blue(void);
uint16_t Remain_outpost_HP(void);
uint16_t JUDGE_id1_usGetRemoteHeat17(void);
uint16_t JUDGE_id2_usGetRemoteHeat17(void);
uint16_t JUDGE_ShootAllow_17mm(void);
uint8_t JUDGE_usGetState(void);
uint16_t JUDGE_usGetShootNum(void);
uint32_t JUDGE_SHOOT_SPEED(void);
void  judgement_data_handler(uint8_t *p_frame);
void JUDGE_ShootNumCount(void);
void determine_ID(void);
#endif
