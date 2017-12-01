/****************************************************************************** 
* 
* �ļ���  ��  
* ������  ��yex 
* �������ڣ� 20171122  
* �汾��  �� v1.0 
* �ļ�������  
* ��Ȩ˵���� Copyright (c) 2000-2020   GNU 
* ��    ���� �� 
* �޸���־�� �� 
* 
*******************************************************************************/
/*---------------------------------- Ԥ������ ---------------------------------*/
/************************************ ͷ�ļ� ***********************************/
#include "inno_fan.h"
/************************************ �궨�� ***********************************/


/*----------------------------------- ������ ----------------------------------*/
/********************************** ���������� *********************************/
/* FAN CTRL */
inno_fan_temp_s s_fan_ctrl;
bool auto_fan = true;
int fan_speed = 50;


/********************************** ���������� *********************************/
static int asic_temp_compare(const void *a, const void *b);   /*��ͳ�Ƶ��������¶���һ�αȽϣ�a>b���������a<b���������a=b,���0 */
static void asic_temp_clear(inno_fan_temp_s *fan_temp, int chain_id);     /*���ͳ�Ƶ��������¶�����*/
static void asic_fan_speed_up(inno_fan_temp_s *fan_temp);   /*��������ת��*/
static void asic_fan_speed_down(inno_fan_temp_s *fan_temp);   /*���ͷ���ת��*/

/********************************** ����ʵ���� *********************************/
static const int inno_tsadc_table[] = {    
/* val temp_f */   
652, //-40,
645, //-35,  
638, //-30,  
631, //-25,  
623, //-20,  
616, //-15,   
609, //-10,   
601, // -5,  
594, //  0,  
587, //  5,  
579, // 10,  
572, // 15,    
564, // 20,   
557, // 25,   
550, // 30,  
542, // 35,  
535, // 40, 
527, // 45, 
520, // 50, 
512, // 55, 
505, // 60, 
498, // 65, 
490, // 70, 
483, // 75, 
475, // 80, 
468, // 85, 
460, // 90, 
453, // 95, 
445, //100, 
438, //105, 
430, //110, 
423, //115, 
415, //120, 
408, //125, 
};

/********************************** ����ʵ���� *********************************/
static int asic_temp_compare(const void *a, const void *b)
{
	return *(int *)a - *(int *)b;
}

static float asic_temp_to_float(inno_fan_temp_s *fan_ctrl, int chain_id)
{
  int i = 0;

 for(i=0; i<ASIC_CHAIN_NUM; i++)
 {
   fan_ctrl->temp2float[i][0] = (TEMP_LABEL - fan_ctrl->temp_highest[i]) * 5 / 7.5;
   fan_ctrl->temp2float[i][1] = (TEMP_LABEL - fan_ctrl->temp_arvarge[i]) * 5 / 7.5;
   fan_ctrl->temp2float[i][2] = (TEMP_LABEL - fan_ctrl->temp_lowest[i]) * 5 / 7.5;
 }
}


void asic_temp_sort(inno_fan_temp_s *fan_ctrl, int chain_id)
{

	int i = 0;
   
#if 0
    printf("\nbefore sort:\n ");
    for(i = 0; i < ASIC_CHIP_NUM; i++)
    {
         printf("chain_%d,chip_%d,temp_%d   ",chain_id, i,  fan_ctrl->temp[chain_id][i]);
    }
#endif
    qsort(fan_ctrl->temp[chain_id], ASIC_CHIP_NUM, sizeof(fan_ctrl->temp[chain_id][0]), asic_temp_compare);
#if 0
	printf("\nafter sort: \n");

	for(i = 0; i < ASIC_CHIP_NUM; i++)
    {
       printf("chain_%d,chip_%d,temp_%d   ",chain_id, i, fan_ctrl->temp[chain_id][i]);
    }
   printf("\n\n");
#endif
}



void asic_temp_clear(inno_fan_temp_s *fan_temp, int chain_id)
{    
   int i;
    for(i = 0; i < ASIC_CHIP_NUM; i++)  
	{       
	fan_temp->temp[chain_id][i] = 0; 
	}
}

bool fan_dev_opt(int fd, int speed, int fan_id)
{

	int duty_driver = 0;
	duty_driver = ASIC_INNO_FAN_PWM_FREQ_TARGET / 100 * (100 - speed);

	if(ioctl(fd, IOCTL_SET_FREQ(fan_id), ASIC_INNO_FAN_PWM_FREQ) < 0)
	{
		printf( "set fan0 frequency fail\n");
		close(fd);
		//mutex_unlock(&fan_temp->lock);
		return false;
	}
	if(ioctl(fd, IOCTL_SET_DUTY(fan_id), duty_driver) < 0)
	{
		printf( "set duty fail \n");
		close(fd);
		//mutex_unlock(&fan_temp->lock);
		return false;
	}


 return true;
}

void inno_fan_speed_set(inno_fan_temp_s *fan_temp, int speed)
{	
	int fd = 0;	
    int fan_id;
	bool ret; 
	int duty_driver = 0;
	duty_driver = ASIC_INNO_FAN_PWM_FREQ_TARGET / 100 * (100 - speed);
	pthread_mutex_lock(&fan_temp->lock);		

	/* �������Ƚ�� */		
	fd = open(ASIC_INNO_FAN_PWM0_DEVICE_NAME, O_RDWR);	
	if(fd < 0)		
	{		
	 printf( "open %s fail\n", ASIC_INNO_FAN_PWM0_DEVICE_NAME);
	 pthread_mutex_unlock(&fan_temp->lock);		
	 return;		
	}


  for(fan_id=0; fan_id<FAN_CNT; fan_id++)
  {
	  if(ioctl(fd, IOCTL_SET_FREQ(fan_id), ASIC_INNO_FAN_PWM_FREQ) < 0)
		  {
			  printf( "set fan0 frequency fail\n");
			  close(fd);
			  pthread_mutex_unlock(&fan_temp->lock);
			  return ;
		  }
		  if(ioctl(fd, IOCTL_SET_DUTY(fan_id), duty_driver) < 0)
		  {
			  printf( "set duty fail \n");
			  close(fd);
			  pthread_mutex_unlock(&fan_temp->lock);
			  return ;
		  }

  }
  close(fd);
  pthread_mutex_unlock(&fan_temp->lock);

 return; 
}
	
void inno_fan_temp_init(inno_fan_temp_s *fan_temp)
{
	int chain_id = 0;
	int speed = 80;
    printf("auto_fan %s, fan_speed %d\n",auto_fan == false?"false":"true", fan_speed);
	pthread_mutex_init(&fan_temp->lock,NULL);
	
    fan_temp->auto_ctrl = auto_fan;
	fan_temp->speed = fan_speed;
	
	if(auto_fan)
	{
	 inno_fan_speed_set(fan_temp,speed);	 
	}else
	{
		inno_fan_speed_set(fan_temp,fan_speed);
	}

	for(chain_id = 0; chain_id < ASIC_CHAIN_NUM; chain_id++)
	{
		asic_temp_clear(fan_temp, chain_id);
	}

	printf( "pwm  name:%s.\n", ASIC_INNO_FAN_PWM0_DEVICE_NAME);
	printf( "pwm  step:%d.\n", ASIC_INNO_FAN_PWM_STEP);
	printf( "duty max: %d.\n", ASIC_INNO_FAN_PWM_DUTY_MAX);
	printf( "targ freq:%d.\n", ASIC_INNO_FAN_PWM_FREQ_TARGET);
	printf( "freq rate:%d.\n", ASIC_INNO_FAN_PWM_FREQ);
	printf( "fan speed thrd:%d.\n", ASIC_INNO_FAN_TEMP_MAX_THRESHOLD);
	printf( "fan up thrd:%d.\n", ASIC_INNO_FAN_TEMP_UP_THRESHOLD);
	printf( "fan down thrd:%d.\n", ASIC_INNO_FAN_TEMP_DOWN_THRESHOLD);
	
}
	
	
bool inno_fan_temp_add(inno_fan_temp_s *fan_temp,int chain_id, int chip_id, int temp)
{
  if((temp > ERR_LOW_TEMP) || (temp < ERR_HIGH_TEMP))
  {
	printf("Notice!!! Error temperature %d\n",temp);
   return false;
  }
  
  
  fan_temp->temp[chain_id][chip_id-1] = temp;
  //printf("chain %d, chip %d, temp %d\n",chain_id, chip_id,fan_temp->temp[chain_id][chip_id-1]);
  return true;
}

	
int inno_fan_temp_highest(inno_fan_temp_s *fan_temp, int chain_id, inno_type_e inno_type)
{
	 int i = 0;
	 int high_avg = 0;

  switch(inno_type)
  {
   case INNO_TYPE_A4:
	 printf("Sorry do not have such type named INNO_TYPE_A4\n");
	 break;
   
   case INNO_TYPE_A5:
   case INNO_TYPE_A6:
   case INNO_TYPE_A7:
   case INNO_TYPE_A8:
	for(i=0; i<ACTIVE_STAT; i++)
	{
	 high_avg += fan_temp->temp[chain_id][i];
	}
	  
	fan_temp->temp_highest[chain_id] = (high_avg/ACTIVE_STAT);
   
	 break;
   
   case INNO_TYPE_A9:
	printf("Sorry do not have such type named INNO_TYPE_A9\n");
   default:
	 break;

 }

  return fan_temp->temp_highest[chain_id];
}

	
	int inno_fan_temp_lowest(inno_fan_temp_s *fan_temp, int chain_id, inno_type_e inno_type)
	{
		 int i = 0;
		 int low_avg = 0;
	
	  switch(inno_type)
	  {
	   case INNO_TYPE_A4:
		 printf("Sorry do not have such type named INNO_TYPE_A4\n");
		 break;
	   
	   case INNO_TYPE_A5:
	   case INNO_TYPE_A6:
	   case INNO_TYPE_A7:
	   case INNO_TYPE_A8:
		for(i=0; i<ACTIVE_STAT; i++)
		{
		 low_avg += fan_temp->temp[chain_id][ASIC_CHIP_NUM-i-1];
		}
		  
		fan_temp->temp_lowest[chain_id] = (low_avg/ACTIVE_STAT);
	   
		 break;
	   
	   case INNO_TYPE_A9:
		printf("Sorry do not have such type named INNO_TYPE_A9\n");
	   default:
		 break;
	
	 }
	
	  return fan_temp->temp_lowest[chain_id];
	}
	
	int inno_fan_temp_avg(inno_fan_temp_s *fan_temp, int chain_id, inno_type_e inno_type)
	{ 
	  int i = 0;
	  int temp_avg = 0;
	  switch(inno_type)
	  {
	   case INNO_TYPE_A4:
		 printf("Sorry do not have such type named INNO_TYPE_A4\n");
		 break;
	   
	   case INNO_TYPE_A5:
	   case INNO_TYPE_A6:
	   case INNO_TYPE_A7:
	   case INNO_TYPE_A8:
		for(i=0; i<ASIC_CHIP_NUM; i++)
		{
		 temp_avg += fan_temp->temp[chain_id][i];
		}
		  
		fan_temp->temp_arvarge[chain_id] = (temp_avg/ASIC_CHIP_NUM);
	   
		 break;
	   
	   case INNO_TYPE_A9:
		printf("Sorry do not have such type named INNO_TYPE_A9\n");
	   default:
		 break;
	
	 }
	 return fan_temp->temp_arvarge[chain_id];
	 
	}
	
	void inno_fan_temp_update(inno_fan_temp_s *fan_temp,int chain_id,inno_type_e inno_type)
	{
	   int delta[4][2]={
				//{FAN_FIRST_STAGE + FAN_DELTA,0},
				{FAN_FIRST_STAGE + FAN_DELTA,FAN_FIRST_STAGE - FAN_DELTA},
				{FAN_SECOND_STAGE + FAN_DELTA,FAN_SECOND_STAGE - FAN_DELTA},
				{FAN_THIRD_STAGE + FAN_DELTA,FAN_THIRD_STAGE - FAN_DELTA},
				{FAN_FOUR_STAGE + FAN_DELTA,FAN_FOUR_STAGE - FAN_DELTA}
			};
		
			int fan_speed[4]={0,50,80,100};
			
	   
			//applog(LOG_ERR, "Read:fan_ctrl->last_fan_tem = %d", fan_ctrl->last_fan_temp);

			asic_temp_sort(fan_temp, chain_id);
			inno_fan_temp_highest(fan_temp, chain_id,inno_type);
			inno_fan_temp_avg(fan_temp, chain_id,inno_type);
            inno_fan_temp_lowest(fan_temp, chain_id,inno_type); 
			asic_temp_to_float(fan_temp, chain_id);
			asic_temp_clear(fan_temp, chain_id);
			

		if(fan_temp->auto_ctrl)
		{
		
			if(fan_temp->temp_highest[chain_id] > delta[fan_temp->last_fan_temp][0])
			{
				if(fan_temp->last_fan_temp > 0)
				{
					fan_temp->last_fan_temp -= 1;
				}
				//applog(LOG_ERR, "%s +:arv:%5.2f, lest:%5.2f, hest:%5.2f, speed:%d%%", __func__, arvarge_f, lowest_f, highest_f, 100 - fan_ctrl->duty); 
			}else if (fan_temp->temp_highest[chain_id] < delta[fan_temp->last_fan_temp][1])
			{
				if(fan_temp->last_fan_temp < 3)
				{
					fan_temp->last_fan_temp += 1;
				}
				//applog(LOG_ERR, "%s +:arv:%5.2f, lest:%5.2f, hest:%5.2f, speed:%d%%", __func__, arvarge_f, lowest_f, highest_f, 100 - fan_ctrl->duty);
			}
				fan_temp->speed = fan_speed[fan_temp->last_fan_temp];
				 //printf("temp_highest %d, fan speed %d,last fan id: %d\n",fan_temp->temp_highest[chain_id],fan_speed[fan_temp->last_fan_temp],fan_temp->last_fan_temp);
	   	}
				inno_fan_speed_set(fan_temp,fan_temp->speed);
			  
	}
	
	
#if 0	 
	int main(int argc, char *argv[])
	{
		int i = 0;
		int j = 0;
		inno_fan_temp_s fan_temp;
		int temp[33] = {0};
		int fan_dep_temp = 0;
		
		for(i=0; i<argc-1; i++)
		{
		printf("argv:%s\n",argv[i]);
		  temp[i] = atoi(argv[i+1]);
		}
	   // const char * log_path = "./";
		//int size = 10000;
		int chain_id = 0;
		inno_type_e inno_type = INNO_TYPE_A7;
		
		printf("Hello, World!\n");
	   // im_log_init(log_path, size);
		inno_fan_temp_init(&fan_temp);
	
	for(j=0; j<3; j++)
	{
		for(i=0; i<33; i++)
		{
		 if(!inno_fan_temp_add(&fan_temp,j,i,temp[i]))
		   printf("chain %d, chip %d temperature maybe has some problem\n",j,i);
		 }
	 }	 
	
	 for(chain_id=0; chain_id<3; chain_id++)
	 {
	  inno_fan_temp_highest(&fan_temp, chain_id,inno_type);
	  inno_fan_temp_lowest(&fan_temp, chain_id,inno_type);
	  inno_fan_temp_avg(&fan_temp, chain_id,inno_type);
	  inno_fan_temp_update(&fan_temp,chain_id);
	  //fan_dep_temp += fan_temp->temp_highest[chain_id]
	  printf("now we get h %d/avg %d /l %d for chain %d\n",fan_temp.temp_highest[chain_id],fan_temp.temp_arvarge[chain_id],fan_temp.temp_lowest[chain_id],chain_id);
	 }
	
	
		
		return 0;
	
	}
#endif
