#ifndef ADRC_USER_H
#define ADRC_USER_H

#define OMIGA 100
//系统内参
/**************TD**********/
#define ADRC_PITCH_r 			0.2f										//快速跟踪因子
#define ADRC_PITCH_h 			0.0005f							//滤波因子，系统调用步长
#define ADRC_PITCH_h0 			0.003f							//fhan步长0.0006
/**************ESO*********/
#define ADRC_PITCH_belta01		(20*(OMIGA))					//扩张状态观测器反馈增益1
#define ADRC_PITCH_belta02		(1*(OMIGA)*(OMIGA))				//扩张状态观测器反馈增益2
#define ADRC_PITCH_belta03		0//((60)*(OMIGA)*(OMIGA))			//扩张状态观测器反馈增益3
#define Z3_PITCH_SEPERATE			2								//扰动分离
/********************************重点调整外参**********************************/
#define ADRC_PITCH_b 			14								//系统输出反馈系数
#define ADRC_PITCH_delta 		1.5								//线性区宽度
#define ADRC_PITCH_alpha1		0.30			
#define ADRC_PITCH_alpha2		1.5				
#define ADRC_PITCH_belta1		0//10000							//跟踪输入信号增益
#define ADRC_PITCH_belta2		0//0.5								//跟踪微分信号增益
/**********NLSEF-PID***********/
#define ADRC_KP								33000
#define ADRC_KI								3
#define ADRC_KD								1.4
/**************TD**********/
#define ADRC_YAW_r 				0.2								//快速跟踪因子
#define ADRC_YAW_h 				0.005f							//滤波因子，系统调用步长
#define ADRC_YAW_h0 			0.05f							//fhan步长
/**************ESO*********/
#define ADRC_YAW_belta01		(10*(OMIGA))					//扩张状态观测器反馈增益1
#define ADRC_YAW_belta02		(1*(OMIGA)*(OMIGA))				//扩张状态观测器反馈增益2
#define ADRC_YAW_belta03		((10)*(OMIGA)*(OMIGA))			//扩张状态观测器反馈增益3
#define Z3_YAW_SEPERATE				3								//扰动分离
/********************************重点调整外参**********************************/
#define ADRC_YAW_b 				50								//系统系数
#define ADRC_YAW_delta 			0.6								//线性区宽度
#define ADRC_YAW_alpha1			0.40			
#define ADRC_YAW_alpha2			1.2	
#define ADRC_YAW_belta1			1000
#define ADRC_YAW_belta2			3

#define Lenth 3000 // number of samples
#define A 20000.0 // amplitude of the signal
#define F 1.0 // frequency of the signal
#define P 0.0 // phase of the signal
#define FS 200.0 // sampling frequency


/**************TD**********/
#define ADRC_LEFT_r 			10								//快速跟踪因子
#define ADRC_LEFT_h 			0.001f							//滤波因子，系统调用步长
#define ADRC_LEFT_h0 			0.005f							//fhan步长
/**************ESO*********/
#define ADRC_LEFT_belta01		(10*(OMIGA))					//扩张状态观测器反馈增益1
#define ADRC_LEFT_belta02		(1*(OMIGA)*(OMIGA))				//扩张状态观测器反馈增益2
#define ADRC_LEFT_belta03		((10)*(OMIGA)*(OMIGA))			//扩张状态观测器反馈增益3
#define Z3_LEFT_SEPERATE		7								//扰动分离
/********************************重点调整外参**********************************/
#define ADRC_LEFT_b 			60								//系统系数
#define ADRC_LEFT_delta 		0.6								//线性区宽度
#define ADRC_LEFT_alpha1		0.40			
#define ADRC_LEFT_alpha2		1.2	
#define ADRC_LEFT_belta1		1000
#define ADRC_LEFT_belta2		3

/**************TD**********/
#define ADRC_RIGHT_r 			10								//快速跟踪因子
#define ADRC_RIGHT_h 			0.001f							//滤波因子，系统调用步长
#define ADRC_RIGHT_h0 			0.005f							//fhan步长
/**************ESO*********/
#define ADRC_RIGHT_belta01		(10*(OMIGA))					//扩张状态观测器反馈增益1
#define ADRC_RIGHT_belta02		(1*(OMIGA)*(OMIGA))				//扩张状态观测器反馈增益2
#define ADRC_RIGHT_belta03		((10)*(OMIGA)*(OMIGA))			//扩张状态观测器反馈增益3
#define Z3_RIGHT_SEPERATE		7								//扰动分离
/********************************重点调整外参**********************************/
#define ADRC_RIGHT_b 			60								//系统系数
#define ADRC_RIGHT_delta 		0.6								//线性区宽度
#define ADRC_RIGHT_alpha1		0.40			
#define ADRC_RIGHT_alpha2		1.2	
#define ADRC_RIGHT_belta1		1000
#define ADRC_RIGHT_belta2		3

typedef struct
{
    float set;
    float rel;
    float out;

    //参数区，这11个就是需要调整的参数
    /****************TD**********/
    float r;        	//快速跟踪因子
    float h;        	//滤波因子,系统步长
	float h0;        	//滤波因子,fhan步长
    /**************ESO**********/
    float b;        	//系统系数
    float delta;    	//delta为fal（e，alpha，delta）函数的线性区间宽度
    float belta01;  	//扩张状态观测器反馈增益1
    float belta02;  	//扩张状态观测器反馈增益2
    float belta03;  	//扩张状态观测器反馈增益3
	float z3_seperate;	//扰动补偿分离
    /**************NLSEF*******/
    float alpha1;
    float alpha2;
    float belta1;		//跟踪输入信号增益
    float belta2;		//跟踪微分信号增益
		float KP;
		float KI;
		float KD;
}ADRC_t;

void ADRC_init(ADRC_t *ADRC_type , float *a);

double sine_wave(void);


#endif

