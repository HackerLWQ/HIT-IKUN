/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "user_ili9341_lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*generic define*/
#define u32 uint32_t
#define u16 uint16_t
#define u8 uint8_t


/*a star part*/

#define treasure 1
#define none_treasure 2
#define north 1<<0
#define south 1<<1
#define west 1<<2
#define east 1<<3




// exti part


#define form_openmv_exti_irqn EXTI9_5_IRQn


#define button_port GPIOB
#define button_pin GPIO_PIN_6

/* control law part start******************************************************/
#define N 9
/*pid law part*/
#define set_center 35

#define normal_speed 
//kp=5,kd=30,forward_spee=1500;
#define fast_speed 
//kp=10,kd=50,forward_spee=1800;
/* motor part define*/
#define RmotorB   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
#define RmotorF  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);

#define LmotorB   HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_RESET);
#define LmotorF   HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_SET);HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);


#define _USE_LCD_DISPLAY
#define _DEBUG





#define speed_base 44

#define to_openmv_busy_pin  GPIO_PIN_7
#define to_openmv_busy_port GPIOC
#define to_openmv_busy 1
#define to_openmv_not_busy 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */

/* control law part variety*/



uint32_t n_Times;//进入中断的次数，请勿清零，将在每次转弯清零

uint32_t fast_time;//快速期中断计数，功能已修改
uint32_t fast_distt;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FSMC_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
int t_turning(int dir);
void SHOW_SENSOR(void);
void GET_SENSOR(void);
void USAR_UART_IDLECallback(UART_HandleTypeDef *huart);
void rest_action(void);
int astar(void);
int usr_run_astar(int startx,int starty,int dstx,int dsty,uint8_t tre_ture);
void update_pos(int way);
void update_speed(int left,int right);
void usr_init_dijkstra();
void get_path(void);
void TSP();
void move_dist(int16_t left_m,int16_t right_m,uint16_t ld,uint16_t rd);
int is_duichen();
void usr_rerun_dijkstra(uint8_t xnow,uint8_t ynow);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
delay example

		for(int i=0;i<500;i++){
	__HAL_TIM_SET_COUNTER(&htim6,0);
	HAL_TIM_Base_Start(&htim6);
	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
	HAL_TIM_Base_Stop(&htim6);
	}



*/
//dijstra

int break_temp=0;
uint8_t dynamic_treasure_count=3;
int pioneer=0;//出发点，注意tempj里这个量不能为1
uint8_t building_the_map_point_neib=0;
uint8_t red_one_blue_two=0;
uint8_t building_the_map=1;
uint8_t building_the_map_point=0;
uint8_t solving_the_map =0;
uint8_t solving_the_map_point =0;
uint8_t solving_the_map_point_neib=0;
uint8_t is_duichenma=0;
short dij_cost[10][10]={0};
int target_j = 0;
uint8_t dij_result[8] = { 0 };
uint8_t dij_result_pointer;

/*   pid part    */
int kp,kd;
int leftm,rightm;
int forward_speed;//功能已移除，本身用于速度控制
int all_wheel_speed=0;
int  forward_spee=1500;//功能已移除，本身用于速度控制
int last_bias;//用途，计算kd
int center,counte;//未知
char white_num;//计算白色个数
int out;//未知
uint16_t error_log[1000];//日志，用于记录数据
int error_log_point=0;//日志指针
uint8_t error_log_lock=0;
//uint8_t u2_buff[600]={01,01, 01, 01, 01, 01, 01, 01, 01, 01, 01, 01, 01, 01, 01, 01, 01, 01, 01, 01, 01,
//	01, 06, 06, 06, 06, 06, 06, 06, 06, 06, 01, 06, 06, 06, 06, 06, 01, 06, 06, 06, 01,
//	01, 06, 01, 06, 01,01, 01, 06, 01, 06, 01, 06, 01, 01, 01, 06, 01, 06, 01, 01, 01, 
//	01, 06, 01, 06, 01, 06, 06, 06, 01, 06, 06, 06, 06, 06, 01, 06, 01, 06, 06, 06, 01, 
//	01, 06, 01, 01, 01, 06, 01, 01, 01, 06, 01, 01, 01, 06, 01, 06, 01, 01, 01, 06, 01, 
//	01, 06, 01, 06, 06, 06, 06, 06, 01, 06, 06, 06, 06, 06, 01, 06, 01, 06, 06, 06, 01, 
//	01, 01, 01, 06, 01, 06, 01, 06, 01, 01, 01, 01, 01, 06, 01, 01, 01, 06, 01, 01, 01, 
//	01, 06, 06, 06, 01, 06, 01, 06, 06, 06, 06, 06, 01, 06, 06, 06, 06, 06, 01, 06, 01, 
//	01, 06, 01, 01, 01, 06, 01, 06, 01, 01, 01, 01, 01, 01, 01, 06, 01, 01, 01, 06, 01, 
//	01, 06, 01, 06, 06, 06, 01, 06, 06, 06, 06, 06, 06, 06, 06, 06, 06, 06, 06, 06, 01, 
//	01, 06, 01, 01, 01, 01, 01, 06, 01, 01, 01, 01, 01, 06, 01, 01, 01, 01, 01, 06, 01, 
//	01, 06, 06, 06, 06, 06, 06, 06, 06, 06, 06, 06, 06, 06, 01, 06, 06, 06, 01, 06, 01, 
//	01, 06, 01, 01, 01, 06, 01, 01, 01, 01, 01, 01, 01, 06, 01, 06, 01, 01, 01, 06, 01, 
//	01, 06, 01, 06, 06, 06, 06, 06, 01, 06, 06, 06, 06, 06, 01, 06, 01, 06, 06, 06, 01, 
//	01, 01, 01, 06, 01, 01, 01, 06, 01, 01, 01, 01, 01, 06, 01, 06, 01, 06, 01, 01, 01, 
//	01, 06, 06, 06, 01, 06, 01, 06, 06, 06, 06, 06, 01, 06, 06, 06, 06, 06, 01, 06, 01, 
//	01, 06, 01, 01, 01, 06, 01, 06, 01, 01, 01, 06, 01, 01, 01, 06, 01, 01, 01 ,06 ,01 ,
//	01 ,06 ,06 ,06 ,01 ,06 ,01 ,06 ,06 ,06 ,06 ,06 ,01 ,06 ,06 ,06 ,01 ,06 ,01 ,06 ,01 ,
//	01 ,01 ,01 ,06 ,01 ,06 ,01 ,01 ,01 ,06 ,01 ,06 ,01 ,06 ,01 ,01 ,01 ,06 ,01 ,06 ,01 ,
//	01 ,06 ,06 ,06 ,01 ,06 ,06 ,06 ,06 ,06 ,01 ,06 ,06 ,06 ,06 ,06 ,06 ,06 ,06 ,06 ,01 ,
//	01 ,01 ,01 ,01 ,01 ,01 ,01 ,01 ,01 ,01 ,01 ,01 ,01 ,01 ,01 ,01 ,01 ,01 ,01 ,01 ,01 ,
//1,3,8,3,5,3,10,4,10,8,3,8,1,7,6,8,1,48};//串口数据暂存包
uint8_t u2_buff[600]={0};
char _tel_ok=1;//串口数据解析完成的标志位
int unlock_dist=0;
int turning_period;
int turning_status;//用于控制转向的开关
	int speed_control_time=0;//当前距离区间经过的中断数初始化曲线表时需要归零
uint16_t speed_and_time[150][4]={0};//存储当前的速度-距离曲线表
int speed_and_time_pointer;//表格用的指针，每次重新存放数据需要归零
short llast_speed=0;//存放上一个循环时的值，有可能需要归零，请注意这里！！！！！！！！！！！！！！！！！！！！！
short rlast_speed=0;
short lspeed_diff=0;//暂存差值
short rspeed_diff=0;
short speed_k=30;
short target_diff=speed_base;//目标走过的路程，p控制
short now_diff;//存放本次走过的路程
//short rtarget_diff;
short ldecoder_value;//单次读取的编码器绝对值
short rdecoder_value;
int finded_treasure=0;//已找到的宝藏数
//char road_book[256]={4,4,4,4,4,4};                                                                                                                                                                                                                                                                       4,12,4,8,14,0,4,8,4,4,12,4,4,14,0,8,8,4,4,14,0,8,4,8,4,8,4,8,8,4,4};
uint8_t road_book[128]={0};//路书
uint8_t road_pointer;//路书指针
uint8_t temp_ever_pushed_treasure=0;
int treasure_pointer=1;//寻找到第几个宝藏了
uint8_t treasure_point[10][3]={0};//宝藏点数组，xy值

uint8_t treasure_lock=0;//宝藏锁，在中断中反复调用
	int treasure_slow_lock=1;

int direction[2][2]={0};//当前位置
/* astar part*/
uint16_t score_left1=0,score_left2=0,score_left3=0;
uint16_t score_right1=0,score_right2=0,score_right3=0;
const int dir[4][2] =     
{
    {0, -1},   	// North 
    {0, 1},  	// South		
    {-1, 0},   	// West
    {1, 0},  	// East

};//四个方向对于的坐标值


typedef struct		
{
    uint8_t x, y;              		
    uint8_t reachable;				
    uint8_t sur;					
}MapNode;

typedef struct Close	
{
    MapNode *cur;              
    char vis;                 	
    struct Close *from;            	
    uint16_t F, G;
    uint16_t H;
}Close;

typedef struct 		
{
    int length;        				
    Close* Array[100];    	
}Open;
uint8_t mem_sur[8];
int road_point[128][2]={0};//路径解析用代码，标记经过的所有点
int road_point_pointer=0;
MapNode  graph[10][10]={0};       
Close  close[10][10] = {0}; 
Open q;                
Close *p;

int srcX = 0;    //??X??
int  srcY = 0;    //??Y??
int    dstX = 0;    //??X??
int    dstY = 0;    //??Y??

int start_x=0;
int start_y=9;

int destina_x=9;
int destina_y=0;

int change_temp;//用于交换方向的临时变量

int soft_exti_count=0;
uint8_t redefine_path_once=1;
int count=0;

int x=0,y=0;//咱也不知道这是干啥的

/* exti part*/
int temp_x_cur=0,temp_y_cur=0;
int fast_speed_tag=0;
uint8_t map_mem=0;
uint8_t map_curr=0;
Close *map_mem_p;
int wait_mask=0;
int exti_count=0;//重新规划路径次数
uint8_t soft_exti=0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	update_speed(900,900);

	if(GPIO_Pin == GPIO_PIN_6){
		update_speed(1050,1050);
		while(1);
		HAL_NVIC_DisableIRQ(form_openmv_exti_irqn);
		HAL_GPIO_WritePin(to_openmv_busy_port,to_openmv_busy_pin,to_openmv_busy);
		
		HAL_TIM_Base_Stop(&htim6);
		update_speed(1000,1000);
		exti_count++;
		map_mem_p=&close[direction[1][0]][direction[1][1]];
		temp_x_cur=direction[1][0]-direction[0][0];
		temp_y_cur=direction[1][1]-direction[0][1];
		if(temp_x_cur==0){
			if(temp_y_cur==1){
				//down is blocked,
				map_mem=14;	
			}else{
				//up is blocked
				map_mem=13;		
			}
		}else{
			if(temp_x_cur==1){
				//right is blocked
				map_mem=11;
			}else{
				//left is blocked
				map_mem=7;
			}
		}
	//	map_mem_p->cur->sur=(map_mem_p->cur->sur)^map_mem;
		map_curr=map_mem_p->cur->sur;
		map_mem_p->cur->sur=(map_mem_p->cur->sur)|map_mem;
		count=0;
 LCD_Clear(0, 0, 240, 320, BACKGROUND);
for(y=0;y<21;y++)
{			
for(x=0;x<21;x++)
{			
		if(u2_buff[count]==1){
		LCD_DispChar(9*x,9*y,'#',BLUE);
	}else{
		LCD_DispChar(9*x,9*y,'*',WHITE);	
	}
		count++;
}
}
		if(treasure_pointer<=dij_result_pointer){
			
			wait_mask= usr_run_astar(direction[1][0],direction[1][1],treasure_point[dij_result[treasure_pointer-1]-1][0],treasure_point[dij_result[treasure_pointer-1]-1][1],1);
		}else{
			wait_mask= usr_run_astar(direction[1][0],direction[1][1],9,0,0);
			
		}
		map_mem_p->cur->sur=map_curr;
		if(wait_mask)
		{
		change_temp=direction[0][0];
	direction[0][0]=direction[1][0];
	direction[1][0]=change_temp;
		change_temp=direction[0][1];
	direction[0][1]=direction[1][1];
	direction[1][1]=change_temp;
//		normal_speed
		fast_time=0;
		treasure_lock=0;
	treasure_slow_lock=0;
		n_Times=0;
		t_turning(1);
		}else{
			while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_6));
			normal_speed
		fast_time=0;
		treasure_lock=0;
	treasure_slow_lock=0;
		n_Times=0;
			
		}
	__HAL_TIM_SET_COUNTER(&htim6,0);
	HAL_TIM_Base_Start_IT(&htim6);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
		HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	}else if(soft_exti){
		exti_count++;
		
				HAL_NVIC_DisableIRQ(form_openmv_exti_irqn);
		HAL_GPIO_WritePin(to_openmv_busy_port,to_openmv_busy_pin,to_openmv_busy);
		
		HAL_TIM_Base_Stop(&htim6);
		update_speed(1000,1000);
		
		 LCD_Clear(0, 0, 240, 320, BACKGROUND);
for(y=0;y<21;y++)
{			
for(x=0;x<21;x++)
{			
		if(u2_buff[count]==1){
		LCD_DispChar(9*x,9*y,'#',BLUE);
	}else{
		LCD_DispChar(9*x,9*y,'*',WHITE);	
	}
		count++;
}
}
soft_exti_count++;
if(exti_count<5){
if(building_the_map)
{
	close[treasure_point[building_the_map_point][0]][treasure_point[building_the_map_point][1]].cur->sur=mem_sur[building_the_map_point];
	usr_run_astar(start_x,start_y,treasure_point[building_the_map_point][0],treasure_point[building_the_map_point][1],1);
	close[treasure_point[building_the_map_point][0]][treasure_point[building_the_map_point][1]].cur->sur=15;
	
	
	
}else if(solving_the_map){
	close[treasure_point[solving_the_map_point][0]][treasure_point[solving_the_map_point][1]].cur->sur=mem_sur[solving_the_map_point];
	usr_run_astar(start_x,start_y,treasure_point[solving_the_map_point][0],treasure_point[solving_the_map_point][1],1);
	close[treasure_point[solving_the_map_point][0]][treasure_point[solving_the_map_point][1]].cur->sur=15;
	
}else if((treasure_pointer<=dij_result_pointer)&&(finded_treasure<dynamic_treasure_count)){
	
//	usr_run_astar(start_x,start_y,treasure_point[dij_result[treasure_pointer-1]-1][0],treasure_point[dij_result[treasure_pointer-1]-1][1],1);
		target_j=0;
for(int i=0;i<8;i++)
{
	if(treasure_point[i][2]&(1<<6))
	{
		target_j+=(1<<(i+1));
	}
	
	
}
pioneer=0;
	
treasure_pointer=1;

get_path();
close[treasure_point[dij_result[0]-1][0]][treasure_point[dij_result[0]-1][1]].cur->sur=mem_sur[dij_result[0]-1];
usr_run_astar(start_x,start_y,treasure_point[dij_result[0]-1][0],treasure_point[dij_result[0]-1][1],1);
close[treasure_point[dij_result[0]-1][0]][treasure_point[dij_result[0]-1][1]].cur->sur=15;
//			wait_mask= usr_run_astar(start_x,start_y,treasure_point[dij_result[treasure_pointer-1]-1][0],treasure_point[dij_result[treasure_pointer-1]-1][1],1);
		}else{
			wait_mask= usr_run_astar(start_x,start_y,9,0,0);
			
		}
		

}else if(exti_count<10){
	
	building_the_map=0;
	solving_the_map=0;

	if(redefine_path_once)
	{
		dynamic_treasure_count=20;
		
		target_j=510;
for(int i=0;i<8;i++)
{
	treasure_point[i][2]=treasure_point[i][2]|(1<<6);

	
	
}
pioneer=0;
	
treasure_pointer=1;

get_path();
close[treasure_point[dij_result[0]-1][0]][treasure_point[dij_result[0]-1][1]].cur->sur=mem_sur[dij_result[0]-1];
usr_run_astar(start_x,start_y,treasure_point[dij_result[0]-1][0],treasure_point[dij_result[0]-1][1],1);
	close[treasure_point[dij_result[0]-1][0]][treasure_point[dij_result[0]-1][1]].cur->sur=15;	
		
		redefine_path_once=0;
	}else if((treasure_pointer<=dij_result_pointer)){
		
		target_j=0;
for(int i=0;i<8;i++)
{
	if(treasure_point[i][2]&(1<<6))
	{
		target_j+=(1<<(i+1));
	}
	
	
}
pioneer=0;
	
treasure_pointer=1;

get_path();
close[treasure_point[dij_result[0]-1][0]][treasure_point[dij_result[0]-1][1]].cur->sur=mem_sur[dij_result[0]-1];
usr_run_astar(start_x,start_y,treasure_point[dij_result[0]-1][0],treasure_point[dij_result[0]-1][1],1);
	close[treasure_point[dij_result[0]-1][0]][treasure_point[dij_result[0]-1][1]].cur->sur=15;
		
	}else{
		
		usr_run_astar(start_x,start_y,9,0,0);
		
		
	}
	
	
	
	
}else{
	
	usr_run_astar(start_x,start_y,9,0,0);
	
	
}


		
			direction[0][0]=start_x;
	direction[0][1]=start_y;
	for(int i=0;i<4;i++)
	{
	if((graph[start_x][start_y].sur)&(1<<i))
	{}else{
		direction[1][0]=start_x+dir[i][0];
		direction[1][1]=start_y+dir[i][1];
		break;
	}
	}
		
		
		
//			speed_and_time[0][0]=speed_base;
//speed_and_time[0][1]=20000;
//speed_and_time[0][2]=15;
//speed_and_time[0][3]=40;				
//	speed_and_time[1][0]=speed_base+10;
//speed_and_time[1][1]=200;	
//speed_and_time[1][2]=25;
//speed_and_time[1][3]=50;				
//	speed_and_time[2][0]=speed_base+20;
//speed_and_time[2][1]=300;
//speed_and_time[2][2]=35;
//speed_and_time[2][3]=65;
//				
//speed_and_time[3][0]=speed_base+30;
//speed_and_time[3][1]=400;
//speed_and_time[3][2]=45;
//speed_and_time[3][3]=80;
//				
//speed_and_time[4][0]=speed_base+40;
//speed_and_time[4][1]=1500;
//speed_and_time[4][2]=55;
//speed_and_time[4][3]=95;

//speed_and_time[5][0]=speed_base+25;
//speed_and_time[5][1]=1600;
//speed_and_time[5][2]=35;
//speed_and_time[5][3]=70;
//				
//				
//				
//speed_and_time[6][0]=speed_base+10;
//speed_and_time[6][1]=1700;
//speed_and_time[6][2]=20;
//speed_and_time[6][3]=50;

//speed_and_time[7][0]=speed_base-5;
//speed_and_time[7][1]=1800;
//speed_and_time[7][2]=10;
//speed_and_time[7][3]=40;

//speed_and_time[8][0]=speed_base;
//speed_and_time[8][1]=30000;
//speed_and_time[8][2]=7;
//speed_and_time[8][3]=30;		
////	LCD_DispStr(40, 100, (uint8_t *)"xxx", WHITE);
unlock_dist=1300;
	speed_and_time[0][0]=speed_base;
speed_and_time[0][1]=100;
speed_and_time[0][2]=15;
speed_and_time[0][3]=40;				
	speed_and_time[1][0]=speed_base+10;
speed_and_time[1][1]=200;	
speed_and_time[1][2]=25;
speed_and_time[1][3]=50;				
	speed_and_time[2][0]=speed_base+20;
speed_and_time[2][1]=300;
speed_and_time[2][2]=35;
speed_and_time[2][3]=65;
				
speed_and_time[3][0]=speed_base+30;
speed_and_time[3][1]=400;
speed_and_time[3][2]=45;
speed_and_time[3][3]=80;
				
speed_and_time[4][0]=speed_base+40;
speed_and_time[4][1]=1500;
speed_and_time[4][2]=55;
speed_and_time[4][3]=95;

speed_and_time[5][0]=speed_base+25;
speed_and_time[5][1]=1600;
speed_and_time[5][2]=35;
speed_and_time[5][3]=70;
				
				
				
speed_and_time[6][0]=speed_base+10;
speed_and_time[6][1]=1700;
speed_and_time[6][2]=20;
speed_and_time[6][3]=50;

speed_and_time[7][0]=speed_base-5;
speed_and_time[7][1]=1800;
speed_and_time[7][2]=10;
speed_and_time[7][3]=40;

speed_and_time[8][0]=speed_base;
speed_and_time[8][1]=30000;
speed_and_time[8][2]=7;
speed_and_time[8][3]=30;		
//	LCD_DispStr(40, 100, (uint8_t *)"xxx", WHITE);
unlock_dist=1300;
turning_status=0;
			__HAL_TIM_SET_COUNTER(&htim6,0);
			__HAL_TIM_SET_COUNTER(&htim3,0);
			__HAL_TIM_SET_COUNTER(&htim4,0);
				turning_status=0;
			n_Times=0;
	speed_control_time=0;
target_diff=speed_and_time[0][0];
kp=speed_and_time[0][2];
kd=speed_and_time[0][3];
score_left1=0,score_left2=0,score_left3=0;
score_right1=0,score_right2=0,score_right3=0;
speed_and_time_pointer=0;
		while(HAL_GPIO_ReadPin(button_port,button_pin))
{
	SHOW_SENSOR();
}
	__HAL_TIM_SET_COUNTER(&htim6,0);
__HAL_TIM_SET_COUNTER(&htim3,0);
__HAL_TIM_SET_COUNTER(&htim4,0);
	HAL_TIM_Base_Start_IT(&htim6);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
		HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
		
		soft_exti=0;






	}
}




/*sensor part*/
int cv_count;
int total_count;
uint8_t cv1,cv2;

uint8_t next_fast_tag=0;
uint8_t treasure_result_temp;
void turning_action(void)
{
	
	HAL_TIM_Base_Stop(&htim6);
	uint16_t temp_min=10000;
	uint8_t temp_min_point;
temp_ever_pushed_treasure=0;
treasure_lock=1;
	treasure_slow_lock=1;
	cv_count=0;
	total_count=0;
	cv1=0;cv2=0;

	error_log[error_log_point]=__HAL_TIM_GET_COUNTER(&htim3)+__HAL_TIM_GET_COUNTER(&htim4);
	error_log_point++;
			error_log[error_log_point]=10000;
	error_log_point++;
	error_log[error_log_point]=(road_book[road_pointer]>>3);
	error_log_point++;
HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
	__HAL_TIM_SET_COUNTER(&htim3,0);
	__HAL_TIM_SET_COUNTER(&htim4,0);
//	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
//	if((road_book[road_pointer]&)==7)
//	{
//		forward_spee=1100;
//		kp=5;
//	  kd=20;
//		fast_time=(((road_book[road_pointer]&240)>>4));
//		LCD_DisNum(200, 270, fast_time, YELLOW);
//		for(int j=0;j<450;j++)
//		{	
//for(int i=0;i<2;i++){
//	__HAL_TIM_SET_COUNTER(&htim6,0);
//	HAL_TIM_Base_Start(&htim6);
//	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
//	HAL_TIM_Base_Stop(&htim6);
//		}
//		GET_SENSOR();
//	}
//		forward_spee=1000;
//	for(int j=0;;j++)
//		{	
//for(int i=0;i<2;i++){
//	__HAL_TIM_SET_COUNTER(&htim6,0);
//	HAL_TIM_Base_Start(&htim6);
//	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
//	HAL_TIM_Base_Stop(&htim6);
//		}
//		GET_SENSOR();
//	}
//		//take a photo
//	while(1);
		
		
//		forward_spee=1100;

//		do{
//			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9))cv_count++;
//			total_count++;
//		for(int i=0;i<2;i++){
//	__HAL_TIM_SET_COUNTER(&htim6,0);
//	HAL_TIM_Base_Start(&htim6);
//	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
//	HAL_TIM_Base_Stop(&htim6);
//		}
//		GET_SENSOR();
//	if(total_count>=10)
//	{
//  LCD_DisNum(200, 270, cv_count, YELLOW); 
//	LCD_DisNum(220, 270, total_count, YELLOW);
//		if(cv_count>6){
//			cv1=1;
//		}else{
//			cv_count=0;
//	total_count=0;
//		}
//	}
//	}while(cv1==0);
//	cv_count=0;
//	total_count=0;
//	
//do{
//			if(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9))cv_count++;
//			total_count++;
//		for(int i=0;i<2;i++){
//	__HAL_TIM_SET_COUNTER(&htim6,0);
//	HAL_TIM_Base_Start(&htim6);
//	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
//	HAL_TIM_Base_Stop(&htim6);
//		}
//		GET_SENSOR();
//	if(total_count>=10)
//	{
//		  LCD_DisNum(200, 270, cv_count, YELLOW); 
//	LCD_DisNum(220, 270, total_count, YELLOW);
//		if(cv_count>6){
//			cv2=1;
//		}else{
//			cv_count=0;
//	total_count=0;
//		}
//	}
//	}while(cv2==0);

//	forward_spee=1100;
//		for(int j=0;j<200;j++)
//		{	
//for(int i=0;i<2;i++){
//	__HAL_TIM_SET_COUNTER(&htim6,0);
//	HAL_TIM_Base_Start(&htim6);
//	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
//	HAL_TIM_Base_Stop(&htim6);
//		}
//		GET_SENSOR();
//	}
//			t_turning(1);

//		forward_spee=1500;


	if(fast_speed_tag)
	{
		next_fast_tag=1;
		update_speed(1600,1600);
					for(int i=0;i<20;i++){
	__HAL_TIM_SET_COUNTER(&htim6,0);
	HAL_TIM_Base_Start(&htim6);
	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
	HAL_TIM_Base_Stop(&htim6);
	}
					
	
	
	
//update_speed(1000,1000);
//target_diff=speed_base;
//	speed_and_time[0][1]=5000;
//speed_and_time_pointer=0;
//	speed_control_time=0;
//					for(int j=0;j<20;j++)
//		{
//		for(int i=0;i<2;i++){
//	__HAL_TIM_SET_COUNTER(&htim6,0);
//	HAL_TIM_Base_Start(&htim6);
//	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
//	HAL_TIM_Base_Stop(&htim6);
//		}
//		GET_SENSOR();
//		
//	}
		
		update_pos(0);
//	fast_time=(((road_book[road_pointer]&248)/8))*18-20;//7/14 -15 to -20
		
		
		
	}else{




	switch(road_book[road_pointer]&6){
		case 2:
			if(t_turning(1))return;
			
		update_pos(1);
		fast_time=(((road_book[road_pointer]&248)/8))*18-5;
		break;
		case 4:
			if(t_turning(-1))return;
		update_pos(-1);
		fast_time=(((road_book[road_pointer]&248)/8))*18-5;
		break;
		case 6:
move_dist(500,500,100,100);
			update_speed(1000,1000);
//			for(int i=0;i<30;i++){
//	__HAL_TIM_SET_COUNTER(&htim6,0);
//	HAL_TIM_Base_Start(&htim6);
//	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
//	HAL_TIM_Base_Stop(&htim6);
//	}
					
//target_diff=speed_base;
//	speed_and_time[0][1]=5000;
//speed_and_time_pointer=0;
//	speed_control_time=0;
//					for(int j=0;j<10;j++)
//		{
//		for(int i=0;i<2;i++){
//	__HAL_TIM_SET_COUNTER(&htim6,0);
//	HAL_TIM_Base_Start(&htim6);
//	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
//	HAL_TIM_Base_Stop(&htim6);
//		}
//		GET_SENSOR();
//		
//	}
		update_pos(0);
	fast_time=(((road_book[road_pointer]&248)/8))*18-20;//7/14 -15 to -20
			break;
	}
}
	fast_speed_tag=0;

	switch(road_book[road_pointer]>>4){
		
		case 1:
			unlock_dist=400;
			if((road_book[road_pointer]&6)==6)
			{
unlock_dist=400;
	//			update_speed(1000,1000);
//				fast_speed_tag=1;
speed_and_time[0][0]=speed_base;
speed_and_time[0][1]=100;
speed_and_time[0][2]=7;
speed_and_time[0][3]=40;				
	speed_and_time[1][0]=speed_base;
speed_and_time[1][1]=200;	
speed_and_time[1][2]=7;
speed_and_time[1][3]=40;				
	speed_and_time[2][0]=speed_base-5;
speed_and_time[2][1]=30000;
speed_and_time[2][2]=7;
speed_and_time[2][3]=40;		
			}else{
				
speed_and_time[0][0]=speed_base;
speed_and_time[0][1]=100;
speed_and_time[0][2]=7;
speed_and_time[0][3]=30;				
	speed_and_time[1][0]=speed_base;
speed_and_time[1][1]=400;	
speed_and_time[1][2]=7;
speed_and_time[1][3]=30;				
	speed_and_time[2][0]=speed_base;
speed_and_time[2][1]=30000;
speed_and_time[2][2]=7;
speed_and_time[2][3]=30;				
				
			}	
			
		break;
		case 2:
			unlock_dist=1500;
						if((road_book[road_pointer]&6)==6)
			{
//				fast_speed_tag=1;
//speed_and_time[0][0]=speed_base;
//speed_and_time[0][1]=50;
//speed_and_time[1][0]=speed_base+10;
//speed_and_time[1][1]=200;			
//speed_and_time[2][0]=speed_base+20;
//speed_and_time[2][1]=400;	
//speed_and_time[3][0]=speed_base+30;
//speed_and_time[3][1]=600;
//speed_and_time[4][0]=speed_base+40;
//speed_and_time[4][1]=1400;			
//speed_and_time[5][0]=speed_base+30;
//speed_and_time[5][1]=1500;	
//speed_and_time[6][0]=speed_base+20;
//speed_and_time[6][1]=1600;
//speed_and_time[7][0]=speed_base+10;
//speed_and_time[7][1]=1700;			
//speed_and_time[8][0]=speed_base;
//speed_and_time[8][1]=30000;	
				
speed_and_time[0][0]=speed_base;
speed_and_time[0][1]=100;
speed_and_time[0][2]=15;
speed_and_time[0][3]=40;				
	speed_and_time[1][0]=speed_base+10;
speed_and_time[1][1]=200;	
speed_and_time[1][2]=25;
speed_and_time[1][3]=50;				
	speed_and_time[2][0]=speed_base+20;
speed_and_time[2][1]=300;
speed_and_time[2][2]=35;
speed_and_time[2][3]=65;
				
speed_and_time[3][0]=speed_base+30;
speed_and_time[3][1]=400;
speed_and_time[3][2]=45;
speed_and_time[3][3]=80;
				
speed_and_time[4][0]=speed_base+40;
speed_and_time[4][1]=1400;
speed_and_time[4][2]=55;
speed_and_time[4][3]=95;

speed_and_time[5][0]=speed_base+25;
speed_and_time[5][1]=1500;
speed_and_time[5][2]=35;
speed_and_time[5][3]=70;
				
				
				
speed_and_time[6][0]=speed_base+10;
speed_and_time[6][1]=1600;
speed_and_time[6][2]=20;
speed_and_time[6][3]=50;

speed_and_time[7][0]=speed_base-5;
speed_and_time[7][1]=1700;
speed_and_time[7][2]=10;
speed_and_time[7][3]=40;

speed_and_time[8][0]=speed_base;
speed_and_time[8][1]=30000;
speed_and_time[8][2]=7;
speed_and_time[8][3]=30;
				
			}else{
				
speed_and_time[0][0]=speed_base;
speed_and_time[0][1]=100;
speed_and_time[0][2]=15;
speed_and_time[0][3]=40;				
	speed_and_time[1][0]=speed_base+10;
speed_and_time[1][1]=200;	
speed_and_time[1][2]=25;
speed_and_time[1][3]=50;				
	speed_and_time[2][0]=speed_base+20;
speed_and_time[2][1]=300;
speed_and_time[2][2]=35;
speed_and_time[2][3]=65;
				
speed_and_time[3][0]=speed_base+30;
speed_and_time[3][1]=400;
speed_and_time[3][2]=45;
speed_and_time[3][3]=80;
				
speed_and_time[4][0]=speed_base+40;
speed_and_time[4][1]=1400;
speed_and_time[4][2]=55;
speed_and_time[4][3]=95;

speed_and_time[5][0]=speed_base+25;
speed_and_time[5][1]=1500;
speed_and_time[5][2]=35;
speed_and_time[5][3]=70;
				
				
				
speed_and_time[6][0]=speed_base+10;
speed_and_time[6][1]=1600;
speed_and_time[6][2]=15;
speed_and_time[6][3]=50;

speed_and_time[7][0]=speed_base-5;
speed_and_time[7][1]=1700;
speed_and_time[7][2]=10;
speed_and_time[7][3]=40;

speed_and_time[8][0]=speed_base;
speed_and_time[8][1]=30000;
speed_and_time[8][2]=7;
speed_and_time[8][3]=30;		
			}	
//speed_and_time[0][0]=speed_base;
//speed_and_time[0][1]=200;
//speed_and_time[1][0]=speed_base+10;
//speed_and_time[1][1]=300;			
//speed_and_time[2][0]=speed_base+20;
//speed_and_time[2][1]=450;	
//speed_and_time[3][0]=speed_base+30;
//speed_and_time[3][1]=600;
//speed_and_time[4][0]=speed_base+40;
//speed_and_time[4][1]=1400;			
//speed_and_time[5][0]=speed_base+30;
//speed_and_time[5][1]=1500;	
//speed_and_time[6][0]=speed_base+20;
//speed_and_time[6][1]=1600;
//speed_and_time[7][0]=speed_base+10;
//speed_and_time[7][1]=1700;			
//speed_and_time[8][0]=speed_base;
//speed_and_time[8][1]=30000;	
		
		
		
		break;
			
		
		case 3:
			unlock_dist=3000;
					if((road_book[road_pointer+1]&6)==6)
			{
//				fast_speed_tag=1;
speed_and_time[0][0]=speed_base;
speed_and_time[0][1]=100;
speed_and_time[0][2]=15;
speed_and_time[0][3]=40;				
	speed_and_time[1][0]=speed_base+10;
speed_and_time[1][1]=200;	
speed_and_time[1][2]=25;
speed_and_time[1][3]=50;				
	speed_and_time[2][0]=speed_base+20;
speed_and_time[2][1]=300;
speed_and_time[2][2]=35;
speed_and_time[2][3]=65;
				
speed_and_time[3][0]=speed_base+30;
speed_and_time[3][1]=400;
speed_and_time[3][2]=45;
speed_and_time[3][3]=80;
				
speed_and_time[4][0]=speed_base+40;
speed_and_time[4][1]=2600;
speed_and_time[4][2]=55;
speed_and_time[4][3]=95;

speed_and_time[5][0]=speed_base+25;
speed_and_time[5][1]=2900;
speed_and_time[5][2]=35;
speed_and_time[5][3]=70;
				
				
				
speed_and_time[6][0]=speed_base+10;
speed_and_time[6][1]=3100;
speed_and_time[6][2]=20;
speed_and_time[6][3]=50;

speed_and_time[7][0]=speed_base-5;
speed_and_time[7][1]=3300;
speed_and_time[7][2]=10;
speed_and_time[7][3]=40;

speed_and_time[8][0]=speed_base;
speed_and_time[8][1]=30000;
speed_and_time[8][2]=7;
speed_and_time[8][3]=30;		
				
			}else{
				
				speed_and_time[0][0]=speed_base;
speed_and_time[0][1]=100;
speed_and_time[0][2]=15;
speed_and_time[0][3]=40;				
	speed_and_time[1][0]=speed_base+10;
speed_and_time[1][1]=200;	
speed_and_time[1][2]=25;
speed_and_time[1][3]=50;				
	speed_and_time[2][0]=speed_base+20;
speed_and_time[2][1]=300;
speed_and_time[2][2]=35;
speed_and_time[2][3]=65;
				
speed_and_time[3][0]=speed_base+30;
speed_and_time[3][1]=400;
speed_and_time[3][2]=45;
speed_and_time[3][3]=80;
				
speed_and_time[4][0]=speed_base+40;
speed_and_time[4][1]=2600;
speed_and_time[4][2]=55;
speed_and_time[4][3]=95;

speed_and_time[5][0]=speed_base+25;
speed_and_time[5][1]=2900;
speed_and_time[5][2]=35;
speed_and_time[5][3]=70;
				
				
				
speed_and_time[6][0]=speed_base+10;
speed_and_time[6][1]=3100;
speed_and_time[6][2]=20;
speed_and_time[6][3]=50;

speed_and_time[7][0]=speed_base-5;
speed_and_time[7][1]=3300;
speed_and_time[7][2]=10;
speed_and_time[7][3]=40;

speed_and_time[8][0]=speed_base;
speed_and_time[8][1]=30000;
speed_and_time[8][2]=7;
speed_and_time[8][3]=30;		
				
				
				
				
				
				
				
//				
//				
//speed_and_time[0][0]=speed_base;
//speed_and_time[0][1]=100;
//speed_and_time[0][2]=15;
//speed_and_time[0][3]=40;				
//	speed_and_time[1][0]=speed_base+10;
//speed_and_time[1][1]=200;	
//speed_and_time[1][2]=25;
//speed_and_time[1][3]=50;				
//	speed_and_time[2][0]=speed_base+20;
//speed_and_time[2][1]=300;
//speed_and_time[2][2]=35;
//speed_and_time[2][3]=65;
//				
//speed_and_time[3][0]=speed_base+30;
//speed_and_time[3][1]=400;
//speed_and_time[3][2]=45;
//speed_and_time[3][3]=80;
//				
//speed_and_time[4][0]=speed_base+40;
//speed_and_time[4][1]=1500;
//speed_and_time[4][2]=55;
//speed_and_time[4][3]=95;

//speed_and_time[5][0]=speed_base+25;
//speed_and_time[5][1]=1600;
//speed_and_time[5][2]=35;
//speed_and_time[5][3]=70;
//				
//				
//				
//speed_and_time[6][0]=speed_base+10;
//speed_and_time[6][1]=1700;
//speed_and_time[6][2]=20;
//speed_and_time[6][3]=50;

//speed_and_time[7][0]=speed_base-5;
//speed_and_time[7][1]=1800;
//speed_and_time[7][2]=10;
//speed_and_time[7][3]=40;

//speed_and_time[8][0]=speed_base;
//speed_and_time[8][1]=30000;
//speed_and_time[8][2]=7;
//speed_and_time[8][3]=30;	
			}	
		
		
		
		
		break;
		case 4:
			
					if((road_book[road_pointer+1]&6)==6)
			{
//				fast_speed_tag=1;
speed_and_time[0][0]=20;
speed_and_time[0][1]=50;
	speed_and_time[1][0]=50;
speed_and_time[1][1]=200;			
	speed_and_time[0][0]=20;
speed_and_time[0][1]=30000;	
				
			}else{
				
speed_and_time[0][0]=20;
speed_and_time[0][1]=50;
	speed_and_time[1][0]=50;
speed_and_time[1][1]=200;			
	speed_and_time[0][0]=20;
speed_and_time[0][1]=30000;	
				
			}	
		
		
		break;
		
		
		
		
		
		
		
		
	}
	

	
	if(next_fast_tag){
		
		speed_and_time[0][0]=speed_base+20;
speed_and_time[0][1]=100;
speed_and_time[0][2]=10;
speed_and_time[0][3]=40;				

	}
	
	
	
//	forward_spee=1800;
//	kp=7;
//	kd=30;
//	normal_speed
//	fast_time=(((road_book[road_pointer]&248)/8))*18-5;
		//fast_time=(((road_book[road_pointer]&248)/8))*20-5;
	if(road_book[road_pointer]&1)//there is a treasure
	{
		
//		update_speed(1000,1000);
		
			__HAL_TIM_SET_COUNTER(&htim3,0);
	__HAL_TIM_SET_COUNTER(&htim4,0);
		
		if((road_book[road_pointer]&6)==6)
		{
			switch(road_book[road_pointer]>>4){
				
				case 1:fast_distt=0;
								break_temp=1;
								break;
				case 2:fast_distt=1800;
								break;
				
				
				
			}
			
			//fast_distt=((((road_book[road_pointer]&248)/8)-1)*850);
			
		}else{
			
			
			switch(road_book[road_pointer]>>4){
				
				case 1:fast_distt=10;
								break;
				case 2:fast_distt=1250;
								break;
				
				
				
			}
//		fast_time=((((road_book[road_pointer]&248)/8)-1)*35-20);
	//	fast_distt=((((road_book[road_pointer]&248)/8)-1)*850-450);
		}
		
		if(break_temp)
		{
			
			target_diff=45;
	speed_and_time[0][1]=5000;
speed_and_time_pointer=0;
	speed_control_time=0;
//		forward_spee=1100;
//		kp=5;
//	  kd=20;
			for(int j=0;j<20;j++)
		{	
	for(int i=0;i<2;i++){
	__HAL_TIM_SET_COUNTER(&htim6,0);
	HAL_TIM_Base_Start(&htim6);
	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
	HAL_TIM_Base_Stop(&htim6);
		}
		GET_SENSOR();
	}
			
			break_temp=0;
		}else{
		
		
		//old one fast_time=((((road_book[road_pointer]&248)/8)-1)*35-25);
//		LCD_DisNum(200, 270, fast_time, YELLOW);
		last_bias=0;
			__HAL_TIM_SET_COUNTER(&htim3,0);
	__HAL_TIM_SET_COUNTER(&htim4,0);
		
		target_diff=speed_base;
	speed_and_time[0][1]=5000;
speed_and_time_pointer=0;
	speed_control_time=0;
		for(int j=0;j<10000;j++)
		{	
	for(int i=0;i<2;i++){
	__HAL_TIM_SET_COUNTER(&htim6,0);
	HAL_TIM_Base_Start(&htim6);
	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
	HAL_TIM_Base_Stop(&htim6);
		}
		GET_SENSOR();
		if((__HAL_TIM_GET_COUNTER(&htim3)+__HAL_TIM_GET_COUNTER(&htim4))>fast_distt)break;
	}
}
				target_diff=10;
	speed_and_time[0][1]=5000;
speed_and_time_pointer=0;
	speed_control_time=0;
//		forward_spee=1100;
//		kp=5;
//	  kd=20;
			for(int j=0;j<50;j++)
		{	
	for(int i=0;i<2;i++){
	__HAL_TIM_SET_COUNTER(&htim6,0);
	HAL_TIM_Base_Start(&htim6);
	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
	HAL_TIM_Base_Stop(&htim6);
		}
		GET_SENSOR();
	}
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_RESET);
				//take a photo here
	
	update_speed(1000,1000);
		for(int i=0;i<60;i++){
	__HAL_TIM_SET_COUNTER(&htim6,0);
	HAL_TIM_Base_Start(&htim6);
	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
	HAL_TIM_Base_Stop(&htim6);
	}
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_SET);
	for(int i=0;i<5;i++){
	__HAL_TIM_SET_COUNTER(&htim6,0);
	HAL_TIM_Base_Start(&htim6);
	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
	HAL_TIM_Base_Stop(&htim6);
	}

//			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9))cv_count++;
//			total_count++;
//		for(int i=0;i<2;i++){
//	__HAL_TIM_SET_COUNTER(&htim6,0);
//	HAL_TIM_Base_Start(&htim6);
//	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
//	HAL_TIM_Base_Stop(&htim6);
//		}
//	if(total_count>=10)
//	{
//  LCD_DisNum(200, 270, cv_count, YELLOW); 
//	LCD_DisNum(220, 270, total_count, YELLOW);
//		if(cv_count>6){
//			cv1=1;
//		}else{
//			cv_count=0;
//	total_count=0;
//		}
//	}
//	
//	cv_count=0;

//	
//	
//	total_count=0;
	
treasure_result_temp=0;
treasure_result_temp+=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_12);
treasure_result_temp=(treasure_result_temp<<1);
treasure_result_temp+=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_10);
treasure_result_temp=(treasure_result_temp<<1);
treasure_result_temp+=HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9);


//if(treasure_result_temp==0)
//{
//	for(int i=0;i<30;i++){
//	__HAL_TIM_SET_COUNTER(&htim6,0);
//	HAL_TIM_Base_Start(&htim6);
//	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
//	HAL_TIM_Base_Stop(&htim6);
//	}
//	
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_RESET);
//	
//	update_speed(1000,1000);
//		for(int i=0;i<300;i++){
//	__HAL_TIM_SET_COUNTER(&htim6,0);
//	HAL_TIM_Base_Start(&htim6);
//	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
//	HAL_TIM_Base_Stop(&htim6);
//	}
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_SET);
//	for(int i=0;i<5;i++){
//	__HAL_TIM_SET_COUNTER(&htim6,0);
//	HAL_TIM_Base_Start(&htim6);
//	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
//	HAL_TIM_Base_Stop(&htim6);
//	}


//	
//treasure_result_temp=0;
//treasure_result_temp+=HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8);
//treasure_result_temp=(treasure_result_temp<<1);
//treasure_result_temp+=HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9);
//treasure_result_temp=(treasure_result_temp<<1);
//treasure_result_temp+=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4);

//	
//	
//	
//}


if(red_one_blue_two==1)
{
//	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8)&&(!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4))){
		
//		normal_speed
	if(treasure_result_temp==4){
		finded_treasure++;
		temp_ever_pushed_treasure=1;
update_speed(1400,1400);
		for(int j=0;j<50;j++)
		{	
	for(int i=0;i<2;i++){
	__HAL_TIM_SET_COUNTER(&htim6,0);
	HAL_TIM_Base_Start(&htim6);
	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
	HAL_TIM_Base_Stop(&htim6);
		}
//		GET_SENSOR();
	}
update_speed(600,600);
			for(int j=0;j<30;j++)
		{	
	for(int i=0;i<2;i++){
	__HAL_TIM_SET_COUNTER(&htim6,0);
	HAL_TIM_Base_Start(&htim6);
	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
	HAL_TIM_Base_Stop(&htim6);
		}
//		GET_SENSOR();
	}
		update_speed(1000,1000);
	target_diff=0;
	speed_and_time[0][1]=5000;
speed_and_time_pointer=0;
	speed_control_time=0;
//		forward_spee=1100;
//		kp=5;
//	  kd=20;
			for(int j=0;j<20;j++)
		{	
	for(int i=0;i<2;i++){
	__HAL_TIM_SET_COUNTER(&htim6,0);
	HAL_TIM_Base_Start(&htim6);
	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
	HAL_TIM_Base_Stop(&htim6);
		}
		GET_SENSOR();
	}
}

	}else{
		
		
		
		
		if(treasure_result_temp==2){
			finded_treasure++;
update_speed(1600,1600);
			temp_ever_pushed_treasure=1;
		for(int j=0;j<30;j++)
		{	
	for(int i=0;i<2;i++){
	__HAL_TIM_SET_COUNTER(&htim6,0);
	HAL_TIM_Base_Start(&htim6);
	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
	HAL_TIM_Base_Stop(&htim6);
		}
//		GET_SENSOR();
	}
update_speed(600,600);
			for(int j=0;j<30;j++)
		{	
	for(int i=0;i<2;i++){
	__HAL_TIM_SET_COUNTER(&htim6,0);
	HAL_TIM_Base_Start(&htim6);
	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
	HAL_TIM_Base_Stop(&htim6);
		}
//		GET_SENSOR();
	}
		update_speed(1000,1000);
	target_diff=0;
	speed_and_time[0][1]=5000;
speed_and_time_pointer=0;
	speed_control_time=0;
//		forward_spee=1100;
//		kp=5;
//	  kd=20;
			for(int j=0;j<20;j++)
		{	
	for(int i=0;i<2;i++){
	__HAL_TIM_SET_COUNTER(&htim6,0);
	HAL_TIM_Base_Start(&htim6);
	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
	HAL_TIM_Base_Stop(&htim6);
		}
		GET_SENSOR();
	}
}
//		normal_speed

//		for(int j=0;j<50;j++)
//		{	
//	for(int i=0;i<2;i++){
//	__HAL_TIM_SET_COUNTER(&htim6,0);
//	HAL_TIM_Base_Start(&htim6);
//	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
//	HAL_TIM_Base_Stop(&htim6);
//		}
//		GET_SENSOR();
//	}
		
		
	}
	if(((road_book[road_pointer]>>4)==1)&&(!temp_ever_pushed_treasure)){
		
				target_diff=10;
	speed_and_time[0][1]=5000;
speed_and_time_pointer=0;
	speed_control_time=0;
//		forward_spee=1100;
//		kp=5;
//	  kd=20;
			for(int j=0;j<15;j++)
		{	
	for(int i=0;i<2;i++){
	__HAL_TIM_SET_COUNTER(&htim6,0);
	HAL_TIM_Base_Start(&htim6);
	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
	HAL_TIM_Base_Stop(&htim6);
		}
		GET_SENSOR();
	}
		
		
	}
	if(t_turning(0))return;
	target_diff=0;
	speed_and_time[0][1]=5000;
speed_and_time_pointer=0;
	speed_control_time=0;
//		forward_spee=1100;
//		kp=5;
//	  kd=20;
			for(int j=0;j<30;j++)
		{	
	for(int i=0;i<2;i++){
	__HAL_TIM_SET_COUNTER(&htim6,0);
	HAL_TIM_Base_Start(&htim6);
	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
	HAL_TIM_Base_Stop(&htim6);
		}
		GET_SENSOR();
	}
//		forward_spee=1500;
//		kp=5;
//	  kd=20;
//	for(int i=0;i<500;i++){
//	__HAL_TIM_SET_COUNTER(&htim6,0);
//	HAL_TIM_Base_Start(&htim6);
//	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
//	HAL_TIM_Base_Stop(&htim6);
//	}
		normal_speed
		fast_time=0;
		treasure_lock=0;
	treasure_slow_lock=0;

	if(((road_book[road_pointer]>>4)==1)&&(!temp_ever_pushed_treasure)){
	move_dist(-500,-500,100,100);
update_speed(1000,1000);
		target_diff=0;
	speed_and_time[0][1]=5000;
speed_and_time_pointer=0;
	speed_control_time=0;
//		forward_spee=1100;
//		kp=5;
//	  kd=20;
			for(int j=0;j<30;j++)
		{	
	for(int i=0;i<2;i++){
	__HAL_TIM_SET_COUNTER(&htim6,0);
	HAL_TIM_Base_Start(&htim6);
	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
	HAL_TIM_Base_Stop(&htim6);
		}
		GET_SENSOR();
	}
		
}
update_speed(1000,1000);
	//search for the next treasure
	
			map_mem_p=&close[direction[1][0]][direction[1][1]];
		temp_x_cur=direction[1][0]-direction[0][0];
		temp_y_cur=direction[1][1]-direction[0][1];
			if(temp_x_cur==0){
			if(temp_y_cur==1){
				//down is blocked,
				map_mem=14;	
			}else{
				//up is blocked
				map_mem=13;		
			}
		}else{
			if(temp_x_cur==1){
				//right is blocked
				map_mem=11;
			}else{
				//left is blocked
				map_mem=7;
			}
		}
	//	map_mem_p->cur->sur=(map_mem_p->cur->sur)^map_mem;
		map_curr=map_mem_p->cur->sur;
//		map_mem_p->cur->sur=(map_mem_p->cur->sur)|map_mem;
		map_mem_p->cur->sur=map_mem;
		error_log[error_log_point]=treasure_result_temp;
		error_log_point++;
		error_log[error_log_point]=2560;
		error_log_point++;
	if(solving_the_map){
		building_the_map_point=solving_the_map_point;
//if((building_the_map_point==3)||(building_the_map_point==4)){
//	building_the_map_point_neib=2;
//}else{
//	building_the_map_point_neib=3;
//}
		switch(solving_the_map_point){
			case 2:
				building_the_map_point_neib=3;
			break;
			case 3:
				building_the_map_point_neib=2;
			
			break;
			case 4:
				building_the_map_point_neib=5;
			
			break;
			case 5:
				building_the_map_point_neib=4;
			break;
			
			
			
			
			
			
		}

if(red_one_blue_two==1){
if(treasure_result_temp&4){
//	treasure_point[building_the_map_point][2]=treasure_point[building_the_map_point][2]|(1<<6);
	treasure_point[7-building_the_map_point][2]=treasure_point[7-building_the_map_point][2]|(1<<5);
	treasure_point[building_the_map_point_neib][2]=treasure_point[building_the_map_point_neib][2]|(1<<5);
	treasure_point[7-building_the_map_point_neib][2]=treasure_point[7-building_the_map_point_neib][2]|(1<<6);
	
}
else if(treasure_result_temp&2){
//	treasure_point[building_the_map_point][2]=treasure_point[building_the_map_point][2]|(1<<5);
	treasure_point[7-building_the_map_point][2]=treasure_point[7-building_the_map_point][2]|(1<<6);
	treasure_point[building_the_map_point_neib][2]=treasure_point[building_the_map_point_neib][2]|(1<<6);
	treasure_point[7-building_the_map_point_neib][2]=treasure_point[7-building_the_map_point_neib][2]|(1<<5);
	
	
}else{treasure_point[7-building_the_map_point][2]=treasure_point[7-building_the_map_point][2]|(1<<6);
	treasure_point[building_the_map_point_neib][2]=treasure_point[building_the_map_point_neib][2]|(1<<6);
	treasure_point[7-building_the_map_point_neib][2]=treasure_point[7-building_the_map_point_neib][2]|(1<<6);}

}else{
	
	if(treasure_result_temp&4){
//	treasure_point[building_the_map_point][2]=treasure_point[building_the_map_point][2]|(1<<5);
	treasure_point[7-building_the_map_point][2]=treasure_point[7-building_the_map_point][2]|(1<<6);
	treasure_point[building_the_map_point_neib][2]=treasure_point[building_the_map_point_neib][2]|(1<<6);
	treasure_point[7-building_the_map_point_neib][2]=treasure_point[7-building_the_map_point_neib][2]|(1<<5);
	
}
else if(treasure_result_temp&2){
//	treasure_point[building_the_map_point][2]=treasure_point[building_the_map_point][2]|(1<<6);
	treasure_point[7-building_the_map_point][2]=treasure_point[7-building_the_map_point][2]|(1<<5);
	treasure_point[building_the_map_point_neib][2]=treasure_point[building_the_map_point_neib][2]|(1<<5);
	treasure_point[7-building_the_map_point_neib][2]=treasure_point[7-building_the_map_point_neib][2]|(1<<6);
	
	
}else{treasure_point[7-building_the_map_point][2]=treasure_point[7-building_the_map_point][2]|(1<<6);
	treasure_point[building_the_map_point_neib][2]=treasure_point[building_the_map_point_neib][2]|(1<<6);
	treasure_point[7-building_the_map_point_neib][2]=treasure_point[7-building_the_map_point_neib][2]|(1<<6);}
	

}
	target_j=0;
for(int i=0;i<8;i++)
{
	if(treasure_point[i][2]&(1<<6))
	{
		target_j+=(1<<(i+1));
	}
	
	
}
pioneer=solving_the_map_point+1;
	


get_path();
close[treasure_point[dij_result[0]-1][0]][treasure_point[dij_result[0]-1][1]].cur->sur=mem_sur[dij_result[0]-1];
usr_run_astar(direction[1][0],direction[1][1],treasure_point[dij_result[0]-1][0],treasure_point[dij_result[0]-1][1],1);
close[treasure_point[dij_result[0]-1][0]][treasure_point[dij_result[0]-1][1]].cur->sur=15;
road_pointer--;
		solving_the_map=0;
	}else if(building_the_map){
		//usr_rerun_dijkstra(treasure_point[building_the_map_point][0],treasure_point[building_the_map_point][1]);
if(building_the_map_point){
	building_the_map_point_neib=0;
}else{
	building_the_map_point_neib=1;
}

if(red_one_blue_two==1){
if(treasure_result_temp&4){
//	treasure_point[building_the_map_point][2]=treasure_point[building_the_map_point][2]|(1<<6);
	treasure_point[7-building_the_map_point][2]=treasure_point[7-building_the_map_point][2]|(1<<5);
	treasure_point[building_the_map_point_neib][2]=treasure_point[building_the_map_point_neib][2]|(1<<5);
	treasure_point[7-building_the_map_point_neib][2]=treasure_point[7-building_the_map_point_neib][2]|(1<<6);
	
}
else if(treasure_result_temp&2){
//	treasure_point[building_the_map_point][2]=treasure_point[building_the_map_point][2]|(1<<5);
	treasure_point[7-building_the_map_point][2]=treasure_point[7-building_the_map_point][2]|(1<<6);
	treasure_point[building_the_map_point_neib][2]=treasure_point[building_the_map_point_neib][2]|(1<<6);
	treasure_point[7-building_the_map_point_neib][2]=treasure_point[7-building_the_map_point_neib][2]|(1<<5);
	
	
}else{treasure_point[7-building_the_map_point][2]=treasure_point[7-building_the_map_point][2]|(1<<6);
	treasure_point[building_the_map_point_neib][2]=treasure_point[building_the_map_point_neib][2]|(1<<6);
	treasure_point[7-building_the_map_point_neib][2]=treasure_point[7-building_the_map_point_neib][2]|(1<<6);}

}else{
	
	if(treasure_result_temp&4){
//	treasure_point[building_the_map_point][2]=treasure_point[building_the_map_point][2]|(1<<5);
	treasure_point[7-building_the_map_point][2]=treasure_point[7-building_the_map_point][2]|(1<<6);
	treasure_point[building_the_map_point_neib][2]=treasure_point[building_the_map_point_neib][2]|(1<<6);
	treasure_point[7-building_the_map_point_neib][2]=treasure_point[7-building_the_map_point_neib][2]|(1<<5);
	
}
else if(treasure_result_temp&2){
//	treasure_point[building_the_map_point][2]=treasure_point[building_the_map_point][2]|(1<<6);
	treasure_point[7-building_the_map_point][2]=treasure_point[7-building_the_map_point][2]|(1<<5);
	treasure_point[building_the_map_point_neib][2]=treasure_point[building_the_map_point_neib][2]|(1<<5);
	treasure_point[7-building_the_map_point_neib][2]=treasure_point[7-building_the_map_point_neib][2]|(1<<6);
	
	
}else{treasure_point[7-building_the_map_point][2]=treasure_point[7-building_the_map_point][2]|(1<<6);
	treasure_point[building_the_map_point_neib][2]=treasure_point[building_the_map_point_neib][2]|(1<<6);
	treasure_point[7-building_the_map_point_neib][2]=treasure_point[7-building_the_map_point_neib][2]|(1<<6);}
	

}






	dij_cost[0][0]=0;
	
	
	srcX = treasure_point[building_the_map_point][0];    //??X??
srcY = treasure_point[building_the_map_point][1];
	for(int temp_i=3;temp_i<7;temp_i++)
	{
    //??Y??
	dstX=treasure_point[temp_i-1][0];
	dstY=treasure_point[temp_i-1][1];	

road_point_pointer=0;
	if(astar()){
p=&close[dstX][dstY];
do{
road_point[road_point_pointer][0]=p->cur->x;
road_point[road_point_pointer][1]=p->cur->y;

p=p->from;
	road_point_pointer++;
}while((p->cur->x!=srcX)||(p->cur->y!=srcY));
road_point[road_point_pointer][0]=p->cur->x;
road_point[road_point_pointer][1]=p->cur->y;

	if(road_point_pointer<temp_min){
	temp_min=road_point_pointer;
		temp_min_point=temp_i-1;
	}



//dij_cost[0][temp_i+1]=road_point_pointer;
//dij_cost[dij_init_treasure_pointer+1][0]=road_point_pointer;
}else{
	while(1);
}
	
	}
	

//direction[1][0],direction[1][1]
//treasure_point[dij_result[treasure_pointer-1]][0],treasure_point[dij_result[treasure_pointer-1]][1]


solving_the_map_point=temp_min_point;//此值可以直接放到treasure——point里当下标
	close[treasure_point[temp_min_point][0]][treasure_point[temp_min_point][1]].cur->sur=mem_sur[temp_min_point];
	usr_run_astar(direction[1][0],direction[1][1],treasure_point[temp_min_point][0],treasure_point[temp_min_point][1],1);//初始值就是1，从第一个到第二个才进到这个函数
		close[treasure_point[temp_min_point][0]][treasure_point[temp_min_point][1]].cur->sur=15;
//	treasure_pointer++;
	//	road_pointer--;
	road_pointer--;
		solving_the_map=1;
		building_the_map=0;
	
		}else if((treasure_pointer<dij_result_pointer)&&finded_treasure<dynamic_treasure_count){
count=0;
 LCD_Clear(0, 0, 240, 320, BACKGROUND);
for(y=0;y<21;y++)
{			
for(x=0;x<21;x++)
{			
		if(u2_buff[count]==1){
		LCD_DispChar(9*x,9*y,'#',BLUE);
	}else{
		LCD_DispChar(9*x,9*y,'*',WHITE);	
	}
		count++;
}
}
//direction[1][0],direction[1][1]
//treasure_point[dij_result[treasure_pointer-1]][0],treasure_point[dij_result[treasure_pointer-1]][1]
treasure_point[dij_result[treasure_pointer-1]-1][2]=(treasure_point[dij_result[treasure_pointer-1]-1][2]&191);
	close[treasure_point[dij_result[treasure_pointer-1]-1][0]][treasure_point[dij_result[treasure_pointer-1]-1][1]].cur->sur=mem_sur[dij_result[treasure_pointer-1]-1];
	close[treasure_point[dij_result[treasure_pointer]-1][0]][treasure_point[dij_result[treasure_pointer]-1][1]].cur->sur=mem_sur[dij_result[treasure_pointer]-1];
	usr_run_astar(direction[1][0],direction[1][1],treasure_point[dij_result[treasure_pointer]-1][0],treasure_point[dij_result[treasure_pointer]-1][1],1);//初始值就是1，从第一个到第二个才进到这个函数
close[treasure_point[dij_result[treasure_pointer]-1][0]][treasure_point[dij_result[treasure_pointer]-1][1]].cur->sur=15;
close[treasure_point[dij_result[treasure_pointer-1]-1][0]][treasure_point[dij_result[treasure_pointer-1]-1][1]].cur->sur=15;
	treasure_pointer++;
		road_pointer--;
	}else{
//		LCD_Clear(0, 0, 240, 320, BACKGROUND);
	count=0;
//for(y=0;y<21;y++)
//{			
//for(x=0;x<21;x++)
//{			
//		if(u2_buff[count]==1){
//		LCD_DispChar(9*x,9*y,'#',BLUE);
//	}else{
//		LCD_DispChar(9*x,9*y,'*',WHITE);		
//	}
//	count++;
//}
//}
			
	usr_run_astar(direction[1][0],direction[1][1],9,0,0);
		
	treasure_pointer++;
road_pointer--;
//	usr_run_astar(0,9,treasure_point[dij_result[0]][0],treasure_point[dij_result[0]][1],1);
	}
	
map_mem_p->cur->sur=map_curr;
		//update the direction 
	change_temp=direction[0][0];
	direction[0][0]=direction[1][0];
	direction[1][0]=change_temp;
		change_temp=direction[0][1];
	direction[0][1]=direction[1][1];
	direction[1][1]=change_temp;
	
		speed_and_time[0][0]=speed_base-20;
speed_and_time[0][1]=100;
speed_and_time[0][2]=15;
speed_and_time[0][3]=40;
	speed_and_time[0][0]=speed_base-10;
speed_and_time[0][1]=200;
speed_and_time[0][2]=10;
speed_and_time[0][3]=50;
speed_and_time[1][0]=speed_base;
speed_and_time[1][1]=3000;
speed_and_time[1][2]=10;
speed_and_time[1][3]=60;	
	unlock_dist=300;
	__HAL_TIM_SET_COUNTER(&htim3,0);
	__HAL_TIM_SET_COUNTER(&htim4,0);
//	error_log[error_log_point]=114514;
//	error_log_point++;
	
	}

	road_pointer++;
	turning_status=0;
			n_Times=0;
	speed_control_time=0;
target_diff=speed_and_time[0][0];
kp=speed_and_time[0][2];
kd=speed_and_time[0][3];
speed_and_time_pointer=0;
	last_bias=0;
	llast_speed=0;rlast_speed=0;
	score_left1=0,score_left2=0,score_left3=0;
score_right1=0,score_right2=0,score_right3=0;
	__HAL_TIM_SET_COUNTER(&htim3,0);
		__HAL_TIM_SET_COUNTER(&htim4,0);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
	__HAL_TIM_SET_COUNTER(&htim6,0);
	
	




	HAL_TIM_Base_Start_IT(&htim6);
}
int temp_dir[2];
Close *c;
uint8_t next_cross=0;
uint8_t surr;

uint16_t counter_savel,counter_saver;
void move_dist(int16_t left_m,int16_t right_m,uint16_t ld,uint16_t rd){
	
	counter_savel=__HAL_TIM_GET_COUNTER(&htim3);
	counter_saver=__HAL_TIM_GET_COUNTER(&htim4);
	counter_savel=counter_savel+ld;
	counter_saver=counter_saver+rd;
	update_speed(left_m+1000,right_m+1000);
	while((counter_savel>__HAL_TIM_GET_COUNTER(&htim3))||(counter_saver>__HAL_TIM_GET_COUNTER(&htim4)));
	
	update_speed(1000,1000);
}
void update_pos(int way)
{
	next_cross=0;
	c=&close[direction[1][0]][direction[1][1]];
	temp_dir[0]=direction[1][0]-direction[0][0];
	temp_dir[1]=direction[1][1]-direction[0][1];
	do{
		//if the direction[1]is a corss
		surr=c->cur->sur;
		if(((surr>>1)&surr)&&(surr!=6))//it is a straight
		{
	direction[0][0]=direction[1][0];
	direction[0][1]=direction[1][1];
		direction[1][0]+=temp_dir[0];
		direction[1][1]+=temp_dir[1];
c=&close[direction[1][0]][direction[1][1]];			
		}else{
			direction[0][0]=c->cur->x;
			direction[0][1]=c->cur->y;
					if(temp_dir[0]==0){//y direction is not zero
			switch(temp_dir[1]){
				case 1:
					if(way==1){
						//right
						
						direction[1][0]=direction[0][0]-1;
						direction[1][1]=direction[0][1];
					}else if(way==-1){
						//left
						direction[1][0]=direction[0][0]+1;
						direction[1][1]=direction[0][1];
						
					}else{
						direction[1][0]=direction[0][0];
						direction[1][1]=direction[0][1]+1;
						
					}
					break;
				
				case -1:
					if(way==1){
						//right
						direction[1][0]=direction[0][0]+1;
						direction[1][1]=direction[0][1];
						
					}else if(way==-1){
						//left
						direction[1][0]=direction[0][0]-1;
						direction[1][1]=direction[0][1];
						
					}else{
						direction[1][0]=direction[0][0];
						direction[1][1]=direction[0][1]-1;
						
					}
					break;
					
				
				default :while(1);	
			}
			
		}else{
			
			switch(temp_dir[0]){
				case 1:
					if(way==1){
						//right
						direction[1][0]=direction[0][0];
						direction[1][1]=direction[0][1]+1;
						
					}else if(way==-1){
						//left
						direction[1][0]=direction[0][0];
						direction[1][1]=direction[0][1]-1;
						
					}else{
						direction[1][0]=direction[0][0]+1;
						direction[1][1]=direction[0][1];
						
					}
					break;
				
				case -1:
					if(way==1){
						//right
						direction[1][0]=direction[0][0];
						direction[1][1]=direction[0][1]-1;
						
					}else if(way==-1){
						//left
						direction[1][0]=direction[0][0];
						direction[1][1]=direction[0][1]+1;
						
					}else{
						direction[1][0]=direction[0][0]-1;
						direction[1][1]=direction[0][1];
						
					}
					break;
					
				
				default :while(1);	
			}
			
		}
		next_cross=1;
		LCD_DispChar((direction[1][0]+direction[0][0])*9+9,(direction[1][1]+direction[0][1])*9+9,'#',RED);
	}
			
			
			
		
		
		
		
		
	}while(!next_cross);
	
	
	
	
}
	



void rest_action(void){
		HAL_TIM_Base_Stop(&htim6);
				update_speed(1000,1000);
	for(int i=0;i<200;i++){

	__HAL_TIM_SET_COUNTER(&htim6,0);
	HAL_TIM_Base_Start(&htim6);
	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
	HAL_TIM_Base_Stop(&htim6);
		}
	
	__HAL_TIM_SET_COUNTER(&htim6,0);
	HAL_TIM_Base_Start_IT(&htim6);
	
}
	int ok,countee;
int debug1,debug2,debug3=0;
int t_turning(int dir)
{

	debug2=0;
	int i=0;
	ok=1;
	countee=0;
//	for(i=0;i<5;i++){
//	__HAL_TIM_SET_COUNTER(&htim6,0);
//	HAL_TIM_Base_Start(&htim6);
//	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
//	HAL_TIM_Base_Stop(&htim6);
//	}
	
	
	
	
//	update_speed(1000,1000);
//	for(i=0;i<10;i++){
//	__HAL_TIM_SET_COUNTER(&htim6,0);
//	HAL_TIM_Base_Start(&htim6);
//	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
//	HAL_TIM_Base_Stop(&htim6);
//	}
//	
//	update_speed(1000,1000);
//	for(i=0;i<100;i++){
//	__HAL_TIM_SET_COUNTER(&htim6,0);
//	HAL_TIM_Base_Start(&htim6);
//	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
//	HAL_TIM_Base_Stop(&htim6);
//	}
//	
	
	
//	update_speed((int)1000+(int)300*dir,(int)1000-(int)300*dir);
//		for(i=0;i<70;i++){
//	__HAL_TIM_SET_COUNTER(&htim6,0);
//	HAL_TIM_Base_Start(&htim6);
//	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
//	HAL_TIM_Base_Stop(&htim6);
//	}

//	
//	update_speed(1000,1000);
//	for(i=0;i<300;i++){
//	__HAL_TIM_SET_COUNTER(&htim6,0);
//	HAL_TIM_Base_Start(&htim6);
//	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
//	HAL_TIM_Base_Stop(&htim6);
//	}
//		update_speed((int)1000+(int)300*dir,(int)1000-(int)300*dir);
	if(dir==1){
//		do{
//	while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14));
//	while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15)&&(countee<1000))countee++;
//			if(countee<1000){
//				ok=0;
//			}else{
//				countee=0;
//			}
//		}while(ok);
// if(dir==2){
//		do{
//	while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12));
//	while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13)&&(countee<1000))countee++;
//			if(countee<1000){
//				ok=0;
//			}else{
//				countee=0;
//			}
//		}while(ok);
//		
//		
//		
//	}else


move_dist(600,0,350,0);//400
update_speed(1600,1100);
//	__HAL_TIM_SET_COUNTER(&htim6,0);
//		do{
				while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13))
	{
		debug2++;
		if(debug2>2000000)
		{
			update_speed(1000,1000);
			soft_exti=1;

			HAL_GPIO_EXTI_Callback(GPIO_PIN_7);
			return 1;
		}
	}
//	HAL_TIM_Base_Start(&htim6);
	while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14))
	{
		
		debug2++;
		if(debug2>2000000)
		{
			update_speed(1000,1000);
			soft_exti=1;
	//		while(1);
			HAL_GPIO_EXTI_Callback(GPIO_PIN_7);
			return 1;
		}
		
		
		
	}//&&(__HAL_TIM_GET_COUNTER(&htim6)<5000));//countee++;
//			if((__HAL_TIM_GET_COUNTER(&htim6)<5000)){
//				ok=0;
//			}else{

//				__HAL_TIM_SET_COUNTER(&htim6,0);
//				HAL_TIM_Base_Stop(&htim6);

//			}
//		}while(ok);

//		HAL_TIM_Base_Stop(&htim6);
		
	
	
	}else if(dir==-1){
		
		move_dist(0,600,0,350);//400
	//		update_speed(1000,1000);
update_speed(1100,1600);
		
//			move_dist(-300,1000,0,350);//400
//			update_speed(1000,1000);

			
//			update_speed(700,2000);
//			__HAL_TIM_SET_COUNTER(&htim6,0);
//		do{

	while(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_8))
	{
		debug2++;
		if(debug2>3500000)
		{
			update_speed(1000,1000);
			soft_exti=1;
	//		while(1);
			HAL_GPIO_EXTI_Callback(GPIO_PIN_7);
			return 1;
		}
	}
//	HAL_TIM_Base_Start(&htim6);
	while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11)){//&&(__HAL_TIM_GET_COUNTER(&htim6)<5000))countee++;
	
		debug2++;
		if(debug2>3500000)
		{
			update_speed(1000,1000);
			soft_exti=1;
	//		while(1);
			HAL_GPIO_EXTI_Callback(GPIO_PIN_7);
			return 1;
		}
		
		
		
		
		
	}
//				update_speed(1000,1000);
//while(1);
	
//			if((__HAL_TIM_GET_COUNTER(&htim6)<5000)){
//				ok=0;
//			}else{

//				__HAL_TIM_SET_COUNTER(&htim6,0);
//				HAL_TIM_Base_Stop(&htim6);
//	//			countee=0;
//			}
//		}while(ok);
//		//debug3=__HAL_TIM_GET_COUNTER(&htim6);
//		HAL_TIM_Base_Stop(&htim6);
	}else{
		
		
		move_dist(1000,-1000,400,400);
//		update_speed(1000,1000);
		update_speed(1200,800);
	//				__HAL_TIM_SET_COUNTER(&htim6,0);
//		do{

		while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12))
	{
		debug2++;
		if(debug2>3000000)
		{
			update_speed(1000,1000);
			soft_exti=1;
	//		while(1);
			HAL_GPIO_EXTI_Callback(GPIO_PIN_7);
			return 1;
		}
	}	
//	HAL_TIM_Base_Start(&htim6);
	while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13)){//
		debug2++;
		if(debug2>3000000)
		{
			update_speed(1000,1000);
			soft_exti=1;
	//		while(1);
			HAL_GPIO_EXTI_Callback(GPIO_PIN_7);
			return 1;
		}
		
		
	}//&&(__HAL_TIM_GET_COUNTER(&htim7)<5000))countee++;
//			if((__HAL_TIM_GET_COUNTER(&htim6)<5000)){
//				ok=0;
//			}else{

//				__HAL_TIM_SET_COUNTER(&htim6,0);
//				HAL_TIM_Base_Stop(&htim6);
//	//			countee=0;
//			}
//		}while(ok);
//		debug3=__HAL_TIM_GET_COUNTER(&htim7);
//		HAL_TIM_Base_Stop(&htim6);
		
	}
	
	
	
	//debug temp
	
	update_speed(1000,1000);
//	while(1);
//		for(i=0;i<100;i++){
//	__HAL_TIM_SET_COUNTER(&htim6,0);
//	HAL_TIM_Base_Start(&htim6);
//	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
//	HAL_TIM_Base_Stop(&htim6);
//	}
//		
	return 0;
	
	//while(1)SHOW_SENSOR();
}
void SHOW_SENSOR(void)
{
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12))
	{
		LCD_DisNum(0, 300, 1, YELLOW);
	}else{
		LCD_DisNum(0, 300, 0, YELLOW);
	}	
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13))
	{
		LCD_DisNum(30, 300, 1, YELLOW);
	}else{
		LCD_DisNum(30, 300, 0, YELLOW);
	}		
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14))
	{
		LCD_DisNum(60, 300, 1, YELLOW);
	}else{
		LCD_DisNum(60, 300, 0, YELLOW);
	}	
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15))
	{
		LCD_DisNum(90, 300, 1, YELLOW);
	}else{
		LCD_DisNum(90, 300, 0, YELLOW);
	}	
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8))
	{
		LCD_DisNum(120, 300, 1, YELLOW);
	}else{
		LCD_DisNum(120, 300, 0, YELLOW);
	}	
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11))
	{
		LCD_DisNum(150, 300, 1, YELLOW);
	}else{
		LCD_DisNum(150, 300, 0, YELLOW);
	}	
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15))
	{
		LCD_DisNum(180, 300, 1, YELLOW);
	}else{
		LCD_DisNum(180, 300, 0, YELLOW);
	}	
	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_12))
	{
		LCD_DisNum(210, 300, 1, YELLOW);
	}else{
		LCD_DisNum(210, 300, 0, YELLOW);
	}
}

//use it per 10ms 
//before use it you should set last_bias to zero
int left_rep=0;
int right_rep=0;
	uint8_t temp=0;

//last_bias=0;
void GET_SENSOR(void)
{
	int bias=0;
	int sensor_pos=0;
	lspeed_diff=0;
	rspeed_diff=0;
	uint8_t mask=128;
temp=0;
	ldecoder_value=__HAL_TIM_GET_COUNTER(&htim3);
	if(llast_speed){
		
	 lspeed_diff=ldecoder_value-llast_speed;
	}
	llast_speed=ldecoder_value;
	
	rdecoder_value=__HAL_TIM_GET_COUNTER(&htim4);
	if(rlast_speed){
	
	rspeed_diff=rdecoder_value-rlast_speed;
	}
	rlast_speed=rdecoder_value;
	now_diff=rspeed_diff+lspeed_diff;
//		error_log[error_log_point]=__HAL_TIM_GET_COUNTER(&htim3)+__HAL_TIM_GET_COUNTER(&htim4);
//	error_log_point++;
//	error_log[error_log_point]=now_diff;
//	error_log_point++;
//	speed_control_time++;
	
	
//	if(speed_control_time>speed_and_time[speed_and_time_pointer][1])
//	{
//		speed_and_time_pointer++;
//		speed_control_time=0;
//		target_diff=speed_and_time[speed_and_time_pointer][0];
//		
//	}
	if((__HAL_TIM_GET_COUNTER(&htim3)+__HAL_TIM_GET_COUNTER(&htim4))>speed_and_time[speed_and_time_pointer][1])
	{
		
		speed_and_time_pointer++;
		speed_control_time=0;
		target_diff=speed_and_time[speed_and_time_pointer][0];
		kp=speed_and_time[speed_and_time_pointer][2];
		kd=speed_and_time[speed_and_time_pointer][3];
	}
	temp+=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12);
	temp*=2;
	temp+=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13);
	temp*=2;
	temp+=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14);
	temp*=2;
	temp+=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15);
	temp*=2;
	temp+=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8);
	temp*=2;
	temp+=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11);
	temp*=2;
	temp+=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_8);
	temp*=2;
	temp+=HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_12);
	
	score_left3=score_left2;
	score_left2=score_left1;
	score_right3=score_right2;
	score_right2=score_right1;
	score_left1=0;
	score_right1=0;
	if(!(temp&128)){
		score_left1+=20;
//		score_right1+=1;
		
	}
	if(!(temp&64)){
		score_left1+=15;
//		score_right1+=1;
	}
		if(!(temp&32)){
		score_left1+=10;
//		score_right1+=1;
	}
			if(!(temp&16)){
		score_left1+=10;
//		score_right1+=10;
	}
			if(!(temp&8)){
//		score_left1+=10;
		score_right1+=10;
	}
			if(!(temp&4)){
//		score_left1+=10;
		score_right1+=10;
	}
			if(!(temp&2)){
//		score_left1+=10;
		score_right1+=15;
	}
				if(!(temp&1)){
//		score_left1+=10;
		score_right1+=20;
	}
	
	if((score_left1)>35)turning_status+=1;
		if((score_right1)>35)turning_status+=2;
	
//	if(right_rep&&((temp&224)==0)){
//		turning_status+=1;
//		right_rep=0;
//	}else{
//		right_rep=0;
//	}
//	if(left_rep&&((temp&7)==0)){
//	turning_status+=2;
//	left_rep=0;
//	}else{
//	left_rep=0;
//	}
//	if((temp&224)==0)right_rep++;  //turning_status+=1;
//	if((temp&7)==0) left_rep++;    //turning_status+=2;


	center=0;
	counte=0;
	white_num=0;
	while(temp&mask)//search for the first black
	{
			mask/=2;
			sensor_pos+=10;
	}
	//the mask the first black
	while(!(temp&mask)) 
	{
		if(mask>0)
		{
			mask/=2;
			center+=sensor_pos;
			white_num++;
			sensor_pos+=10;
		}else{
			   break;  //all are white
		}
		
	}
	if(white_num<=3&&white_num>0)
	{
		center=center/white_num;
		
	}else{
		center=35;
		
	}
	all_wheel_speed=(((target_diff-now_diff)*speed_k)>0)?((target_diff-now_diff)*speed_k):0;
//	all_wheel_speed=(target_diff-now_diff)*speed_k;
	bias=(int)35-center;
	//calculate with kp
	if(last_bias){
	out= kp*bias+kd*(bias-last_bias);
	}else{
		out= kp*bias;
	}
	last_bias=bias;
//	forward_speed=forward_spee;
	update_speed(1000+all_wheel_speed+out,1000+all_wheel_speed-out);
//	error_log[error_log_point]=temp;
//	error_log_point++;
//	if(error_log_lock){
//	error_log[error_log_point]=temp;
//	error_log_point++;
//		error_log[error_log_point]=bias;
//	error_log_point++;
//		error_log[error_log_point]=out;
//	error_log_point++;
//		error_log[error_log_point]=all_wheel_speed;
//	error_log_point++;
//		error_log[error_log_point]=30000;
//	error_log_point++;
//	}
}




/*telemetry function*/
void USR_UART1_IRQHandler(UART_HandleTypeDef *huart)
{
	 if(RESET != __HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE))   
        {	 
            __HAL_UART_CLEAR_IDLEFLAG(&huart2);                    
            
            
            USAR_UART_IDLECallback(huart);                          
        }
}

int *pointer;
//int temp1=0,temp2=0,temp3=0,temp4=0,temp5=0,temp6=0;;
void USAR_UART_IDLECallback(UART_HandleTypeDef *huart)
{

	int	pointer=0;
	
		while(u2_buff[pointer]!=0){
			pointer++;
		}
		if((pointer!=0)&&(u2_buff[pointer-1]=='0'))
		{		
			//(pointer==0)||
			
		HAL_UART_DMAStop(&huart2);
		//HAL_NVIC_DisableIRQ(USART2_IRQn);
		LCD_DisNum(180,270,pointer, YELLOW);			
			
			pointer-=441;
			count=0;
for(y=0;y<21;y++)
{			
for(x=0;x<21;x++)
{			
		if(u2_buff[count]==1){
		LCD_DispChar(9*x,9*y,'#',BLUE);
	}else{
		LCD_DispChar(9*x,9*y,'*',WHITE);	
	}
//	temp1=x&1;
//	temp2=y&1;
//	if((temp1)&&(temp2))
//	 {
//		 temp3=u2_buff[count];
//		 if(temp3==1){
//			 temp4=y>>1;
//			 temp5=x>>1;
//		graph[y>>1][x>>1].reachable=treasure;
//		LCD_DispChar(9*x,9*y,'#',YELLOW);
//	}else{
//		graph[y>>1][x>>1].reachable=none_treasure;
//		
//	}
//			temp4=y>>1;
//			 temp5=x>>1;
//		 graph[y>>1][x>>1].x=(temp5);
//		graph[y>>1][x>>1].y=(temp4);
////search for the surroundings
//	if(u2_buff[count+1]==1)graph[temp4][temp5].sur+=east;
//	if(u2_buff[count-1]==1)graph[temp4][temp5].sur+=west;
// 	if(u2_buff[count-21]==1)graph[temp4][temp5].sur+=north;
//	if(u2_buff[count+21]==1)graph[temp4][temp5].sur+=south;	 
//	 }
	 if((x&1)&&(y&1))   //backup
	 {
		 if(u2_buff[count]==1){
		graph[x>>1][y>>1].reachable=treasure;
		LCD_DispChar(9*x,9*y,'#',YELLOW);
	}else{
		graph[x>>1][y>>1].reachable=none_treasure;
		
	}

		 graph[x>>1][y>>1].x=(x>>1);
	  graph[x>>1][y>>1].y=(y>>1);
//search for the surroundings
	if(u2_buff[count+1]==1)graph[x>>1][y>>1].sur+=east;
	if(u2_buff[count-1]==1)graph[x>>1][y>>1].sur+=west;
	if(u2_buff[count-21]==1)graph[x>>1][y>>1].sur+=north;
	if(u2_buff[count+21]==1)graph[x>>1][y>>1].sur+=south;	 
	 }

	count++;
}
}
treasure_point[0][0]=u2_buff[count]-1;
treasure_point[0][1]=u2_buff[count+1]-1;
treasure_point[1][0]=u2_buff[count+2]-1;
treasure_point[1][1]=u2_buff[count+3]-1;
treasure_point[2][0]=u2_buff[count+4]-1;
treasure_point[2][1]=u2_buff[count+5]-1;
treasure_point[3][0]=u2_buff[count+6]-1;
treasure_point[3][1]=u2_buff[count+7]-1;
treasure_point[4][0]=u2_buff[count+8]-1;
treasure_point[4][1]=u2_buff[count+9]-1;
treasure_point[5][0]=u2_buff[count+10]-1;
treasure_point[5][1]=u2_buff[count+11]-1;
treasure_point[6][0]=u2_buff[count+12]-1;
treasure_point[6][1]=u2_buff[count+13]-1;
treasure_point[7][0]=u2_buff[count+14]-1;
treasure_point[7][1]=u2_buff[count+15]-1;
if((u2_buff[count+16])==1)
{
	red_one_blue_two=2;
	
}else if((u2_buff[count+16])==16){
	red_one_blue_two=1;
}
//pointer=road_point[0];
//for(int i=0;i<16;i++)
//{
//pointer=u2_buff[count+i];
//pointer++;
//}

is_duichenma=is_duichen();
	
usr_run_astar(0,9,9,0,1);
for(int i=0;i<8;i++)
{
	mem_sur[i]=close[treasure_point[i][0]][treasure_point[i][1]].cur->sur;
	close[treasure_point[i][0]][treasure_point[i][1]].cur->sur=15;
	
}

usr_init_dijkstra();
//for(int i=0;i<pointer-1;i++)// for road book
//{
//	road_book[i]=u2_buff[count+i];
//}
	_tel_ok=0;
update_speed(1300,1300);
//while(1);

	if(dij_cost[0][1]>dij_cost[0][2])//go to point 1
	{
		building_the_map_point =1;
		close[treasure_point[1][0]][treasure_point[1][1]].cur->sur=mem_sur[1];

		usr_run_astar(0,9,treasure_point[1][0],treasure_point[1][1],1);
		close[treasure_point[1][0]][treasure_point[1][1]].cur->sur=15;
		treasure_point[1][2]=treasure_point[1][2]&(1<<7);
	}else{//go tp point 0
		building_the_map_point =0;
			close[treasure_point[0][0]][treasure_point[0][1]].cur->sur=mem_sur[0];

		usr_run_astar(0,9,treasure_point[0][0],treasure_point[0][1],1);
		close[treasure_point[0][0]][treasure_point[0][1]].cur->sur=15;
		treasure_point[0][2]=treasure_point[0][2]&(1<<7);
	}
	//usr_run_astar(0,9,treasure_point[dij_result[0]-1][0],treasure_point[dij_result[0]-1][1],1);

	//init direction
	direction[0][0]=start_x;
	direction[0][1]=start_y;
	for(int i=0;i<4;i++)
	{
	if((graph[start_x][start_y].sur)&(1<<i))
	{}else{
		direction[1][0]=start_x+dir[i][0];
		direction[1][1]=start_y+dir[i][1];
		break;
	}
	}
//	
//	
//	update_speed(1300,1000);
	for(int i=0;i<100;i++){
	__HAL_TIM_SET_COUNTER(&htim6,0);
	HAL_TIM_Base_Start(&htim6);
	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
	HAL_TIM_Base_Stop(&htim6);
	}

update_speed(1000,1000);
	__HAL_UART_DISABLE_IT(&huart2, UART_IT_IDLE);

	
	
	
//	usr_run_astar(0,9,9,0,1);
	
	//请注释掉
	
}else{
	return;
}
	return;
 }


void update_speed(int left,int right)//from 0 to 2000 stay 1000 to stop
{

	if(left>1000)
	{
		LmotorF
		if(left>2000){
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 2000);
		}
		else{
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, left-1000);
		}	
	}else if(left<1000)
	{
		LmotorB
		if(left<0){
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);
		}
		else{
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 1000-left);
		}	
	}else{
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);
	}
	
		if(right>1000)
	{
		RmotorF
		if(right>2000){
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 2000);
		}else{
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, right-1000);
		}
	}else if(right<1000)
	{
		RmotorB
			if(right<0){
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0);
		}else{
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 1000-right);
		}
	}else{
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0);
	}
}
uint16_t take_away_from_the_white_count=0;
uint16_t take_away_from_the_ground_count=0;
//	uint8_t temp_point[8][3];
int is_duichen(){
	uint8_t xiangxianld=0,xiangxianlu=0,xiangxianrd=0,xiangxianru=0;
	uint8_t temp_point[8][2];
	for(int i=0;i<8;i++)
	{
		if((treasure_point[i][0]==255)&&(treasure_point[i][1]==255))
		{
			continue;
		}
		for(int j=i+1;j<8;j++)
		{
			if(((treasure_point[i][0]+treasure_point[j][0])==9)&&((treasure_point[i][1]+treasure_point[j][1])==9))
			{
				if(treasure_point[i][0]<5&&treasure_point[i][1]<5)//
				{
					if(xiangxianlu==0){
						temp_point[5][0]=treasure_point[i][0];
						temp_point[5][1]=treasure_point[i][1];
						temp_point[2][0]=treasure_point[j][0];
						temp_point[2][1]=treasure_point[j][1];
						xiangxianlu++;
						xiangxianrd++;
						
					}else if(xiangxianlu==1){
						temp_point[4][0]=treasure_point[i][0];
						temp_point[4][1]=treasure_point[i][1];
						temp_point[3][0]=treasure_point[j][0];
						temp_point[3][1]=treasure_point[j][1];
						xiangxianlu++;
						xiangxianrd++;
						
						
						
						
					}else{ 
						return 0;						
					
					}
				}else if(treasure_point[i][0]>=5&&treasure_point[i][1]>=5)
				{
										
					if(xiangxianrd==0){
						temp_point[5][0]=treasure_point[j][0];
						temp_point[5][1]=treasure_point[j][1];
						temp_point[2][0]=treasure_point[i][0];
						temp_point[2][1]=treasure_point[i][1];
						xiangxianlu++;
						xiangxianrd++;
						
					}else if(xiangxianrd==1){
						temp_point[4][0]=treasure_point[j][0];
						temp_point[4][1]=treasure_point[j][1];
						temp_point[3][0]=treasure_point[i][0];
						temp_point[3][1]=treasure_point[i][1];
						xiangxianlu++;
						xiangxianrd++;
						
					}else{ 
						return 0;						
					
					}
					
					
					
					
				}else if(treasure_point[i][0]<5&&treasure_point[i][1]>=5){
									
					if(xiangxianld==0){
						temp_point[7][0]=treasure_point[j][0];
						temp_point[7][1]=treasure_point[j][1];
						temp_point[0][0]=treasure_point[i][0];
						temp_point[0][1]=treasure_point[i][1];
						xiangxianld++;
						xiangxianru++;
						
					}else if(xiangxianld==1){
						temp_point[6][0]=treasure_point[j][0];
						temp_point[6][1]=treasure_point[j][1];
						temp_point[1][0]=treasure_point[i][0];
						temp_point[1][1]=treasure_point[i][1];
						xiangxianld++;
						xiangxianru++;
						
					}else{ 
						return 0;						
					
					}
				
				
			}else{
				if(xiangxianru==0){
						temp_point[7][0]=treasure_point[i][0];
						temp_point[7][1]=treasure_point[i][1];
						temp_point[0][0]=treasure_point[j][0];
						temp_point[0][1]=treasure_point[j][1];
						xiangxianld++;
						xiangxianru++;
						
					}else if(xiangxianru==1){
						temp_point[6][0]=treasure_point[i][0];
						temp_point[6][1]=treasure_point[i][1];
						temp_point[1][0]=treasure_point[j][0];
						temp_point[1][1]=treasure_point[j][1];
						xiangxianld++;
						xiangxianru++;
						
					}else{ 
						return 0;						
					
					}
				
				
				
			}
				treasure_point[i][0]=255;
				treasure_point[i][1]=255;
				treasure_point[j][0]=255;
				treasure_point[j][1]=255;
				break;
			}
			
		}
		
	}
	for(int i=0;i<8;i++)
	{
		treasure_point[i][0]=temp_point[i][0];
		treasure_point[i][1]=temp_point[i][1];
		
	}
	return 1;                
}

void USR_TIM_IRQHandler(TIM_HandleTypeDef *htim)
{

	GET_SENSOR();

		if((unlock_dist<(__HAL_TIM_GET_COUNTER(&htim3)+__HAL_TIM_GET_COUNTER(&htim4)))&&turning_status)
	{

		turning_action();	
	}else{
		if(temp==0){
			take_away_from_the_ground_count++;
			if(take_away_from_the_ground_count>10){
				  update_speed(1000,1000);
			soft_exti=1;
	//		while(1);
			HAL_GPIO_EXTI_Callback(GPIO_PIN_7);
				return;
			}
		}else{
			take_away_from_the_ground_count=0;
		}
		if(temp==255){
			take_away_from_the_white_count++;
			if(take_away_from_the_white_count>50){
				  	update_speed(1000,1000);
			soft_exti=1;
	//		while(1);
			HAL_GPIO_EXTI_Callback(GPIO_PIN_7);
				return;
			}
		}else{
			take_away_from_the_white_count=0;
		}
		turning_status=0;
	}
	n_Times++;

//	if(treasure_slow_lock&&(n_Times>10))
//	{
//	fast_speed
//	//	rest_action();
//	treasure_slow_lock=0;
//	//rest_action();
//	}
//	if(treasure_lock&&(n_Times>fast_time))
//	{
////		forward_spee=1500;
////		kp=5;kd=20;
//		normal_speed
////		SHOW_SENSOR();

//		treasure_lock=0;
////		rest_action();

//	}

//	LCD_DisNum(0, 270,__HAL_TIM_GET_COUNTER(&htim6), YELLOW);
}
	
/*math part*/

int abs(int num)
{
    if (num < 0) {
        
        return ~(--num);
    }
    return num;

}
void initClose(Close cls[10][10], int sx, int sy, int dx, int dy)
{    

    int i, j;
    for (i = 0; i < 10; i++)
    {
        for (j = 0; j < 10; j++)
        {
            cls[i][j].cur = &graph[i][j];		       		
            cls[i][j].vis = 0;       
            cls[i][j].from = NULL;                		
            cls[i][j].G = cls[i][j].F = 0;
            cls[i][j].H = 10*abs(dx - i) + 10*abs(dy - j);   
        }
    }
    cls[sx][sy].F = cls[sx][sy].H;          
    cls[sy][sy].G = 0;                       
    //cls[dx][dy].G = Infinity;
}
void initOpen(Open *q)    //???????
{
    q->length = 0;        // ????????0
}
void push(Open *q, Close cls[10][10], uint16_t x, uint16_t y, uint16_t g)			
{    
    Close *t;
    int i, mintag;
	
    cls[x][y].G = g;    //????????
    cls[x][y].F = cls[x][y].G + cls[x][y].H;
	
    q->Array[q->length++] = &(cls[x][y]);
    mintag = q->length - 1;
    for (i = 0; i < q->length - 1; i++)		//???F??????
    {
        if (q->Array[i]->F < q->Array[mintag]->F)
        {
            mintag = i;
        }
    }
    t = q->Array[q->length - 1];
    q->Array[q->length - 1] = q->Array[mintag];
    q->Array[mintag] = t;    				//?F?????????
}
Close* shift(Open *q)
{
    return q->Array[--q->length];
}
    int i, curX, curY, surX, surY;    //??????,??????
    uint16_t surG;
int astar()
{    // A*????

  
    initOpen(&q);
    initClose(close, srcX, srcY, dstX, dstY);  //???? ? ??,?????????
    close[srcX][srcY].vis = 1;            
    push(&q, close, srcX, srcY, 0);       //????Close list,???????
  
    while (q.length)
    {    
        p = shift(&q);
        curX = p->cur->x;
        curY = p->cur->y;
        if (!p->H)
        {	
            return 1;
        }
        for (i = 0; i < 4; i++)
        {
            if ((p->cur->sur & (1 << i)))
            {
                continue;
            }
            surX = curX + dir[i][0];
            surY = curY + dir[i][1];
			
			//surG = p->G + sqrt((curX - surX) * (curX - surX) + (curY - surY) * (curY - surY));
			//surG = p->G + abs(curX - surX) + abs(curY - surY);
			
			    surG = p->G + 10;
			if (!close[surX][surY].vis)             //????????Openlist?
			{						
			    close[surX][surY].vis = 1;          //??
			    close[surX][surY].from = p;
			    push(&q, close, surX, surY, surG);
			}
			else
			{
				if(surG < close[surX][surY].G)   //Openlist??????,??G???,????????
				{
					close[surX][surY].vis = 1;
					close[surX][surY].from = p;    
			        push(&q, close, surX, surY, surG);
			   }
			}
        }
    }
    return 0; //???
}
int dire[2][2]={0};

uint8_t new_road_book[128]={0};
uint8_t new_road_book_pointer;
uint8_t temp_sur;
int one_count;
	int zero_i;
uint8_t distance;
int usr_run_astar(int startx,int starty,int dstx,int dsty,uint8_t tre_ture)
{
//		error_log[error_log_point]=startx;
//	error_log_point++;
//			error_log[error_log_point]=starty;
//	error_log_point++;
//			error_log[error_log_point]=dstx;
//	error_log_point++;
//			error_log[error_log_point]=dsty;
//	error_log_point++;
//	error_log[error_log_point]=10000;
//	error_log_point++;
	uint8_t temp=0;
srcX = startx;    //??X??
srcY = starty;    //??Y??
dstX = dstx;    //??X??
dstY = dsty; 
one_count=0;
zero_i=0;
	new_road_book_pointer=0;
	distance=0;
road_point_pointer=0;
if(astar()){
p=&close[dstx][dsty];
do{
road_point[road_point_pointer][0]=p->cur->x;
road_point[road_point_pointer][1]=p->cur->y;
LCD_DispChar(18*p->cur->x+9,18*p->cur->y+9,'#',YELLOW);
p=p->from;
	road_point_pointer++;
}while((p->cur->x!=srcX)||(p->cur->y!=srcY));
road_point[road_point_pointer][0]=p->cur->x;
road_point[road_point_pointer][1]=p->cur->y;
LCD_DispChar(18*p->cur->x+9,18*p->cur->y+9,'#',YELLOW);

//decode road book
dire[0][0]=road_point[road_point_pointer-1][0]-road_point[road_point_pointer][0];
dire[0][1]=road_point[road_point_pointer-1][1]-road_point[road_point_pointer][1];



for(int i=road_point_pointer-1;i>0;i--)
{
	distance+=2;
	dire[1][0]=road_point[i-1][0]-road_point[i][0];
dire[1][1]=road_point[i-1][1]-road_point[i][1];
	if((dire[1][0]==dire[0][0])&&(dire[1][1]==dire[0][1]))
	{
		
		temp_sur=close[road_point[i][0]][road_point[i][1]].cur->sur;
		one_count=0;
		for(zero_i=0;zero_i<4;zero_i++)
		{
			if(temp_sur&(1<<zero_i))one_count++;
		}
		if((one_count==1)||(one_count==0)){
			new_road_book[new_road_book_pointer]=((distance<<3)+6);
			distance=0;
			new_road_book_pointer++;
		}else{
//			distance+=2;
			
		}
		
	}else{
		// find out it is left turn or right turn
		if(dire[0][0]==0){//y direction is not zero
			switch(dire[0][1]){
				case 1:
					if(dire[1][0]==1){
						//left
						new_road_book[new_road_book_pointer]=((distance<<3)+4);
						distance=0;
						new_road_book_pointer++;
						
					}else{
						//right
						new_road_book[new_road_book_pointer]=((distance<<3)+2);
						distance=0;
						new_road_book_pointer++;
						
					}
					break;
				
				case -1:
					if(dire[1][0]==1){
						//left
						new_road_book[new_road_book_pointer]=((distance<<3)+2);
						distance=0;
						new_road_book_pointer++;
						
					}else{
						//right
						new_road_book[new_road_book_pointer]=((distance<<3)+4);
						distance=0;
						new_road_book_pointer++;
						
					}
					break;
					
				
				default :while(1);	
			}
			
		}else{
			
			switch(dire[0][0]){
				case 1:
					if(dire[1][1]==1){
						//left
						new_road_book[new_road_book_pointer]=((distance<<3)+2);
						distance=0;
						new_road_book_pointer++;
						
					}else{
						//right
						new_road_book[new_road_book_pointer]=((distance<<3)+4);
						distance=0;
						new_road_book_pointer++;
						
					}
					break;
				
				case -1:
					if(dire[1][1]==1){
						//left
						new_road_book[new_road_book_pointer]=((distance<<3)+4);
						distance=0;
						new_road_book_pointer++;
						
					}else{
						//right
						new_road_book[new_road_book_pointer]=((distance<<3)+2);
						distance=0;
						new_road_book_pointer++;
						
					}
					break;
					
				
				default :while(1);	
			}
			
		}
		
	}
	dire[0][0]=dire[1][0];
	dire[0][1]=dire[1][1];
}
distance+=2;
distance=(distance<<3);
new_road_book[new_road_book_pointer-1]+=tre_ture;
//if(!tre_ture)new_road_book[new_road_book_pointer-1]+=32;//debug not stable



//temp=(new_road_book[new_road_book_pointer-1]&248);
//new_road_book[new_road_book_pointer-1]=(new_road_book[new_road_book_pointer-1]&7);
//new_road_book[new_road_book_pointer-1]+=(distance<<3)
//distance=temp;
for(int i=new_road_book_pointer-1;i>=0;i--){
	temp=(new_road_book[i]&248);
	new_road_book[i]=(new_road_book[i]&7);
	new_road_book[i]+=distance;
	distance=temp;
}

//load the road book

for(int i=0;i<new_road_book_pointer;i++)road_book[i]=new_road_book[i];
road_pointer=0;
return 1;
}else
{
	update_speed(900,900);
	while(1);
	//return 0;
}//??Y??
	
}
uint8_t dij_dstx=0,dij_dsty=0,dij_srcx=0,dij_srcy=0;
uint8_t dij_i=0,dij_j=0;
uint8_t dij_init_treasure_pointer;
void usr_init_dijkstra(){
	dij_init_treasure_pointer=0;

	 dij_i=0,dij_j=0;
	road_point_pointer=0;
//	dij_cost[0]
	dij_dstx=0,dij_dsty=0,dij_srcx=0,dij_srcy=0;
	
	dij_cost[0][0]=0;
	
	
	srcX = start_x;    //??X??
srcY = start_y;
	for(dij_init_treasure_pointer=0;dij_init_treasure_pointer<8;dij_init_treasure_pointer++)
	{
    //??Y??
		close[treasure_point[dij_init_treasure_pointer][0]][treasure_point[dij_init_treasure_pointer][1]].cur->sur=mem_sur[dij_init_treasure_pointer];
	dij_dstx=treasure_point[dij_init_treasure_pointer][0];
	dij_dsty=treasure_point[dij_init_treasure_pointer][1];	
dstX = dij_dstx;    //??X??
dstY = dij_dsty; 
road_point_pointer=0;
	if(astar()){
p=&close[dij_dstx][dij_dsty];
do{
road_point[road_point_pointer][0]=p->cur->x;
road_point[road_point_pointer][1]=p->cur->y;

p=p->from;
	road_point_pointer++;
}while((p->cur->x!=srcX)||(p->cur->y!=srcY));
road_point[road_point_pointer][0]=p->cur->x;
road_point[road_point_pointer][1]=p->cur->y;





dij_cost[0][dij_init_treasure_pointer+1]=road_point_pointer;
dij_cost[dij_init_treasure_pointer+1][0]=road_point_pointer;
close[treasure_point[dij_init_treasure_pointer][0]][treasure_point[dij_init_treasure_pointer][1]].cur->sur=15;
}else{
	while(1);
}
	
	}
		dij_dstx=destina_x;
	dij_dsty=destina_y;	
dstX = dij_dstx;    //??X??
dstY = dij_dsty; 
road_point_pointer=0;
	if(astar()){
p=&close[dij_dstx][dij_dsty];
do{
road_point[road_point_pointer][0]=p->cur->x;
road_point[road_point_pointer][1]=p->cur->y;

p=p->from;
	road_point_pointer++;
}while((p->cur->x!=srcX)||(p->cur->y!=srcY));
road_point[road_point_pointer][0]=p->cur->x;
road_point[road_point_pointer][1]=p->cur->y;

dij_cost[0][9]=road_point_pointer;
dij_cost[9][0]=road_point_pointer;

}



	for(dij_i=0;dij_i<8;dij_i++)
{
	
	srcX =treasure_point[dij_i][0];    //??X??
srcY = treasure_point[dij_i][1];
	
	
	close[treasure_point[dij_i][0]][treasure_point[dij_i][1]].cur->sur=mem_sur[dij_i];
	
	
	
	
	
	
	
	
	
	
	for(dij_init_treasure_pointer=dij_i+1;dij_init_treasure_pointer<8;dij_init_treasure_pointer++)
	{
    //??Y??
		close[treasure_point[dij_init_treasure_pointer][0]][treasure_point[dij_init_treasure_pointer][1]].cur->sur=mem_sur[dij_init_treasure_pointer];
		
	dij_dstx=treasure_point[dij_init_treasure_pointer][0];
	dij_dsty=treasure_point[dij_init_treasure_pointer][1];	
dstX = dij_dstx;    //??X??
dstY = dij_dsty; 
road_point_pointer=0;
	if(astar()){
p=&close[dij_dstx][dij_dsty];
do{
road_point[road_point_pointer][0]=p->cur->x;
road_point[road_point_pointer][1]=p->cur->y;

p=p->from;
	road_point_pointer++;
}while((p->cur->x!=srcX)||(p->cur->y!=srcY));
road_point[road_point_pointer][0]=p->cur->x;
road_point[road_point_pointer][1]=p->cur->y;



dij_cost[dij_init_treasure_pointer+1][dij_i+1]=road_point_pointer;
dij_cost[dij_i+1][dij_init_treasure_pointer+1]=road_point_pointer;
close[treasure_point[dij_init_treasure_pointer][0]][treasure_point[dij_init_treasure_pointer][1]].cur->sur=15;
}else{
	while(1);
}
	
	}
		dij_dstx=destina_x;
	dij_dsty=destina_y;	
dstX = dij_dstx;    //??X??
dstY = dij_dsty; 
road_point_pointer=0;
	if(astar()){
p=&close[dij_dstx][dij_dsty];
do{
road_point[road_point_pointer][0]=p->cur->x;
road_point[road_point_pointer][1]=p->cur->y;

p=p->from;
	road_point_pointer++;
}while((p->cur->x!=srcX)||(p->cur->y!=srcY));
road_point[road_point_pointer][0]=p->cur->x;
road_point[road_point_pointer][1]=p->cur->y;

dij_cost[dij_i+1][9]=road_point_pointer;
dij_cost[9][dij_i+1]=road_point_pointer;
	
	
	}
	
	
	
	
	
	
	
	
		close[treasure_point[dij_i][0]][treasure_point[dij_i][1]].cur->sur=15;
	
}
	//destina_x;
	
	
	TSP();
get_path();
	


//start to solve the problem
	
}


void usr_rerun_dijkstra(uint8_t xnow,uint8_t ynow){
	dij_init_treasure_pointer=0;

	 dij_i=0,dij_j=0;
	road_point_pointer=0;
//	dij_cost[0]
	dij_dstx=0,dij_dsty=0,dij_srcx=0,dij_srcy=0;
	
	dij_cost[0][0]=0;
	
	
	srcX = xnow;    //??X??
srcY = ynow;
	for(dij_init_treasure_pointer=0;dij_init_treasure_pointer<8;dij_init_treasure_pointer++)
	{
    //??Y??
	dij_dstx=treasure_point[dij_init_treasure_pointer][0];
	dij_dsty=treasure_point[dij_init_treasure_pointer][1];	
dstX = dij_dstx;    //??X??
dstY = dij_dsty; 
road_point_pointer=0;
	if(astar()){
p=&close[dij_dstx][dij_dsty];
do{
road_point[road_point_pointer][0]=p->cur->x;
road_point[road_point_pointer][1]=p->cur->y;

p=p->from;
	road_point_pointer++;
}while((p->cur->x!=srcX)||(p->cur->y!=srcY));
road_point[road_point_pointer][0]=p->cur->x;
road_point[road_point_pointer][1]=p->cur->y;





dij_cost[0][dij_init_treasure_pointer+1]=road_point_pointer;
dij_cost[dij_init_treasure_pointer+1][0]=road_point_pointer;
}else{
	while(1);
}
	
	}
	
	
			dij_dstx=destina_x;
	dij_dsty=destina_y;	
dstX = dij_dstx;    //??X??
dstY = dij_dsty; 
road_point_pointer=0;
	if(astar()){
p=&close[dij_dstx][dij_dsty];
do{
road_point[road_point_pointer][0]=p->cur->x;
road_point[road_point_pointer][1]=p->cur->y;

p=p->from;
	road_point_pointer++;
}while((p->cur->x!=srcX)||(p->cur->y!=srcY));
road_point[road_point_pointer][0]=p->cur->x;
road_point[road_point_pointer][1]=p->cur->y;

dij_cost[0][9]=road_point_pointer;
dij_cost[9][0]=road_point_pointer;

}

	
	
	
	TSP();
get_path();
	
	
	
	
	
	
	
	
}

uint16_t dp[512][9] = { 0 };

void TSP() {
    //初始化dp[i][0]
    for (int i = 0; i < 9; i++) {
        dp[0][i] = dij_cost[i][9];
    }
    //求解dp[i][j],先跟新列在更新行
    for (int j = 1; j < 512; j++) {
        for (int i = 0; i < N; i++) {
            dp[j][i] = 10000;
            //如果集和j(或状态j)中包含结点i,则不符合条件退出
            if (((j >> (i)) & 1) == 1) {
                continue;
            }
            for (int k = 0; k < N; k++) {
                if (((j >> (k )) & 1) == 0) {
                    continue;
                }//从当前集合中挑一个出来，如果不在集合里就continue
                if (dp[j][i] > dij_cost[i][k] + dp[j ^ (1 << (k))][k]) {//更新表
                    dp[j][i] = dij_cost[i][k] + dp[j ^ (1 << (k))][k];
                }
            }
        }
    }

}
int temp_j;

int target_i = 0;
int dij_min;

int dij_temp_result;

void get_path(void){
    temp_j = target_j;
    dij_min = 10000;
//    pioneer;
    dij_result_pointer = 0;
    while (temp_j)
    {
        for (int i = 0; i < N; i++) {
                        if (((temp_j&(1<<i)))) {//第一项是这一项是否在需要探究的列表里（即是否在集合中）集合会不断缩小，使用temp_j来标记
                            if (dij_min > dij_cost[i][pioneer] + dp[(temp_j ^ (1 << (i)))][i]) {
                                dij_min = dij_cost[i][pioneer] + dp[(temp_j ^ (1 << (i)))][i];
                                dij_temp_result = i;
                            }
                        }
                    }

        pioneer = dij_temp_result;
        dij_result[dij_result_pointer] = pioneer;
        dij_result_pointer++;
        dij_min = 10000;
        temp_j = (temp_j ^ (1 << (dij_temp_result)));
    }

}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FSMC_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
//speed_and_time[0][0]=
	/* motor init*/
	
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0);
	LmotorF
	RmotorF
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);//standby
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
	LCD_Rst();
	Lcd_GramScan(1);
	LCD_REG_Config();
//	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
  LCD_Clear(0, 0, 240, 320, BACKGROUND);	
	LCD_DispStr(40, 100, (uint8_t *)"xxx", WHITE);
	for(int i=0;i<200;i++){
//speed_and_time[0][1]=30000;
	__HAL_TIM_SET_COUNTER(&htim6,0);
	HAL_TIM_Base_Start(&htim6);
	while(__HAL_TIM_GET_COUNTER(&htim6)<=4999);
	HAL_TIM_Base_Stop(&htim6);
		}
	speed_and_time[0][0]=speed_base;
speed_and_time[0][1]=100;
speed_and_time[0][2]=15;
speed_and_time[0][3]=40;				
	speed_and_time[1][0]=speed_base+10;
speed_and_time[1][1]=200;	
speed_and_time[1][2]=25;
speed_and_time[1][3]=50;				
	speed_and_time[2][0]=speed_base+20;
speed_and_time[2][1]=300;
speed_and_time[2][2]=35;
speed_and_time[2][3]=65;
				
speed_and_time[3][0]=speed_base+30;
speed_and_time[3][1]=400;
speed_and_time[3][2]=45;
speed_and_time[3][3]=80;
				
speed_and_time[4][0]=speed_base+40;
speed_and_time[4][1]=1500;
speed_and_time[4][2]=55;
speed_and_time[4][3]=95;

speed_and_time[5][0]=speed_base+25;
speed_and_time[5][1]=1600;
speed_and_time[5][2]=35;
speed_and_time[5][3]=70;
				
				
				
speed_and_time[6][0]=speed_base+10;
speed_and_time[6][1]=1700;
speed_and_time[6][2]=20;
speed_and_time[6][3]=50;

speed_and_time[7][0]=speed_base-5;
speed_and_time[7][1]=1800;
speed_and_time[7][2]=10;
speed_and_time[7][3]=40;

speed_and_time[8][0]=speed_base;
speed_and_time[8][1]=30000;
speed_and_time[8][2]=7;
speed_and_time[8][3]=30;		
//	LCD_DispStr(40, 100, (uint8_t *)"xxx", WHITE);
unlock_dist=1300;

		HAL_UART_Receive_DMA(&huart2,u2_buff,550);
		__HAL_TIM_SET_COUNTER(&htim3,0);
			HAL_TIM_Base_Start(&htim3);
		__HAL_TIM_SET_COUNTER(&htim4,0);
	  	HAL_TIM_Base_Start(&htim4);

		/*pid veriaty init     */
	turning_status=0;
	kp=7;
	kd=50;
	road_pointer=0;

//	forward_spee=1500;
	normal_speed
	treasure_slow_lock=0;
	__HAL_TIM_SET_COUNTER(&htim6,0);
//USAR_UART_IDLECallback(&huart2);
//update_speed(700,700);
//update_speed(1000,1000);
//update_speed(1300,1300);
while(HAL_GPIO_ReadPin(button_port,button_pin))
{ldecoder_value=__HAL_TIM_GET_COUNTER(&htim3);
	rdecoder_value=__HAL_TIM_GET_COUNTER(&htim4);
	SHOW_SENSOR();
}
		__HAL_TIM_SET_COUNTER(&htim4,0);
		__HAL_TIM_SET_COUNTER(&htim3,0);
// &&_tel_ok
	HAL_TIM_Base_Start_IT(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
//		ldecoder_value=__HAL_TIM_GET_COUNTER(&htim3);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 83;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 8300;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
	HAL_NVIC_EnableIRQ(USART2_IRQn);
	HAL_NVIC_SetPriority(USART2_IRQn,4,4);
__HAL_UART_CLEAR_IDLEFLAG(&huart2);  				//??IDLE??
__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_0|GPIO_PIN_1
                          |GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC13 PC14 PC0 PC1
                           PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_0|GPIO_PIN_1
                          |GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC15 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA11 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  hsram1.Init.PageSize = FSMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
