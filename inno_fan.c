/******************************************************************************
 *
 * �ļ���  ��inno_fan.c
 * ������  ��yex(yex@innosilicon.com.cn)
 * �������ڣ� 20171122
 * �汾��  �� v1.0
 * �ļ������� �������¶ȿ������ʵ��
 * ��Ȩ˵���� Copyright (c) 2000-2020   GNU
 * ��    ���� ��
 * �޸���־�� ��
 *
 *******************************************************************************/
/*---------------------------------- Ԥ������ ---------------------------------*/
/************************************ ͷ�ļ� ***********************************/
#include "inno_fan.h"
#include <stdlib.h>
/************************************ �궨�� ***********************************/


/*----------------------------------- ������ ----------------------------------*/
/********************************** ���������� *********************************/
/* FAN CTRL */
//extern inno_fan_temp_s g_fan_ctrl;
int g_auto_fan = 1;  //�����Զ�/�ֶ����ƾ��
int g_fan_speed = 1; //���ȷ�λ���
int fan_speed[8]={40,50,60,70,80,90,100}; //���ȷֵ�
//inno_fan_temp_s g_fan_ctrl;


/********************************** ���������� *********************************/
static int asic_temp_compare(const void *a, const void *b);   /*��ͳ�Ƶ��������¶���һ�αȽϣ�a>b���������a<b���������a=b,���0 */
static void asic_temp_clear(inno_fan_temp_s *fan_temp, int chain_id);     /*���ͳ�Ƶ��������¶�����*/
//static void asic_fan_speed_up(inno_fan_temp_s *fan_temp);   /*��������ת��*/
//static void asic_fan_speed_down(inno_fan_temp_s *fan_temp);   /*���ͷ���ת��*/

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

static void asic_temp_to_float(inno_fan_temp_s *fan_ctrl, int chain_id)
{
    /* int i = 0; */
    //printf("pre_warn: %d\n",fan_ctrl->pre_warn[3]);

   // for(i=0; i<ASIC_CHAIN_NUM; i++)
  // {
        if(fan_ctrl->valid_chain[chain_id])
            return;
        
        if((fan_ctrl->temp_highest[chain_id] > ERR_LOW_TEMP) || (fan_ctrl->temp_highest[chain_id] < ERR_HIGH_TEMP) || \
                (fan_ctrl->temp_arvarge[chain_id] > ERR_LOW_TEMP) || (fan_ctrl->temp_arvarge[chain_id] < ERR_HIGH_TEMP) || \
                (fan_ctrl->temp_lowest[chain_id] > ERR_LOW_TEMP) || (fan_ctrl->temp_lowest[chain_id] < ERR_HIGH_TEMP) )
        {
            inno_log(IM_LOG_ERR,"Notice!!! Error temperature for chain %d,h:%d,a:%d,l:%d\n", chain_id, \
                fan_ctrl->temp_highest[chain_id],fan_ctrl->temp_arvarge[chain_id],fan_ctrl->temp_lowest[chain_id]);
            return ;
        }


        fan_ctrl->temp2float[chain_id][0] = (TEMP_LABEL - fan_ctrl->temp_highest[chain_id]) * 5 / 7.5;
        fan_ctrl->temp2float[chain_id][1] = (TEMP_LABEL - fan_ctrl->temp_arvarge[chain_id]) * 5 / 7.5;
        fan_ctrl->temp2float[chain_id][2] = (TEMP_LABEL - fan_ctrl->temp_lowest[chain_id]) * 5 / 7.5;
    //}

}


void asic_temp_sort(inno_fan_temp_s *fan_ctrl, int chain_id)
{

    //int i = 0;

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
//        fan_temp->valid_temp[chain_id][i] = 0;
    }
}

void inno_fan_speed_set(inno_fan_temp_s *fan_temp, int speed)
{
    int fd = 0;
    int fan_id;
    int duty_driver = 0;
    duty_driver = ASIC_INNO_FAN_PWM_FREQ_TARGET / 100 * (100 - speed);
	inno_log(IM_LOG_ERR, "set fan speed %d\n", speed);
    pthread_mutex_lock(&fan_temp->lock);

    /* �������Ƚ�� */
    fd = open(ASIC_INNO_FAN_PWM0_DEVICE_NAME, O_RDWR);
    if(fd < 0)
    {
        inno_log(IM_LOG_ERR, "open %s fail\n", ASIC_INNO_FAN_PWM0_DEVICE_NAME);
        pthread_mutex_unlock(&fan_temp->lock);
        return;
    }

    for(fan_id=0; fan_id<FAN_CNT; fan_id++)
    {
        if(ioctl(fd, IOCTL_SET_FREQ(fan_id), ASIC_INNO_FAN_PWM_FREQ) < 0)
        {
            inno_log(IM_LOG_ERR,"set fan0 frequency fail\n");
            close(fd);
            pthread_mutex_unlock(&fan_temp->lock);
            return ;
        }
        if(ioctl(fd, IOCTL_SET_DUTY(fan_id), duty_driver) < 0)
        {
            inno_log(IM_LOG_ERR,"set duty fail \n");
            close(fd);
            pthread_mutex_unlock(&fan_temp->lock);
            return ;
        }
    }
    close(fd);
    pthread_mutex_unlock(&fan_temp->lock);

    return;
}

void inno_fan_temp_init(inno_fan_temp_s *fan_temp,int *fan_level)
{
    int chain_id = 0;
    int speed = 80;
    pthread_mutex_init(&fan_temp->lock,NULL);

    fan_temp->auto_ctrl = g_auto_fan;
    fan_temp->speed = speed;

	if(fan_level != NULL){
		   // printf("size %d, %d\n",sizeof(fan_speed),sizeof(fan_speed[0]));
		   memcpy(fan_speed, fan_level,sizeof(fan_speed));
	   }

    if(g_auto_fan)
    {
        inno_fan_speed_set(fan_temp,speed);
    }else
    {
        inno_fan_speed_set(fan_temp,fan_speed[g_fan_speed]);
    }

    for(chain_id = 0; chain_id < ASIC_CHAIN_NUM; chain_id++)
    {
        asic_temp_clear(fan_temp, chain_id);
    }

    inno_log(IM_LOG_DEBUG, "pwm  name:%s.\n", ASIC_INNO_FAN_PWM0_DEVICE_NAME);
    inno_log(IM_LOG_DEBUG, "pwm  step:%d.\n", ASIC_INNO_FAN_PWM_STEP);
    inno_log(IM_LOG_DEBUG, "duty max: %d.\n", ASIC_INNO_FAN_PWM_DUTY_MAX);
    inno_log(IM_LOG_DEBUG, "targ freq:%d.\n", ASIC_INNO_FAN_PWM_FREQ_TARGET);
    inno_log(IM_LOG_DEBUG, "freq rate:%d.\n", ASIC_INNO_FAN_PWM_FREQ);
    inno_log(IM_LOG_DEBUG, "fan speed thrd:%d.\n", ASIC_INNO_FAN_TEMP_MAX_THRESHOLD);
    inno_log(IM_LOG_DEBUG, "fan up thrd:%d.\n", ASIC_INNO_FAN_TEMP_UP_THRESHOLD);
    inno_log(IM_LOG_DEBUG, "fan down thrd:%d.\n", ASIC_INNO_FAN_TEMP_DOWN_THRESHOLD);
    inno_log(IM_LOG_DEBUG, "auto_fan %s, fan_speed %d\n",g_auto_fan == 0?"false":"true", g_fan_speed);
}

bool inno_fan_temp_add(inno_fan_temp_s *fan_temp,int chain_id, int chip_id, int temp)
{
    if((temp > ERR_LOW_TEMP) || (temp < ERR_HIGH_TEMP))
    {
        inno_log(IM_LOG_DEBUG,"Notice!!! Error temperature %d for chain %d, chip %d\n",temp, chain_id, chip_id);
        //printf("Notice!!! Error temperature %d for chain %d, chip %d\n",temp, chain_id, chip_id);
        return false;
    }

    //fan_temp->valid_temp[chain_id][chip_id-1] = 1;
    pthread_mutex_lock(&fan_temp->lock);

    if(temp < PRE_DGR_TEMP)
    {
        fan_temp->pre_warn[0] = chain_id;
        fan_temp->pre_warn[1] = chip_id;
        fan_temp->pre_warn[2] = 0;
        fan_temp->pre_warn[3] = (TEMP_LABEL -   temp) * 5 / 7.5;
        //fan_temp->pre_warn[3] = fan_temp->temp[chain_id][i];
        //printf("There maybe some problem in chain %d, chip %d,The highest temp %d\n",chain_id,chip_id,temp);
        //fan_temp->temp_highest[chain_id] = fan_temp->temp[chain_id][i];
        // break;
    }

    fan_temp->temp[chain_id][chip_id-1] = temp;
    pthread_mutex_unlock(&fan_temp->lock);
   //inno_log(IM_LOG_DEBUG,"chain %d, chip %d, temp %d\n",chain_id, chip_id,fan_temp->temp[chain_id][chip_id-1]);
    return true;
}

int inno_fan_temp_highest(inno_fan_temp_s *fan_temp, int chain_id, inno_type_e inno_type)
{
    int i = 0;
    int high_avg = 0;
    static int stat_hi = 0;
    pthread_mutex_lock(&fan_temp->lock);

    switch(inno_type)
    {
        case INNO_TYPE_A4:
            inno_log(IM_LOG_DEBUG,"Sorry do not have such type named INNO_TYPE_A4\n");
            break;

        case INNO_TYPE_A5:
        case INNO_TYPE_A6:
        case INNO_TYPE_A7:
        case INNO_TYPE_A8:
            for(i=0; i<ASIC_CHIP_NUM; i++)
            {
                if(fan_temp->temp[chain_id][i] != 0)
                {
#if 0
                    if(fan_temp->temp[chain_id][i] < PRE_DGR_TEMP)
                    {
                        fan_temp->pre_warn[0] = chain_id;
                        fan_temp->pre_warn[1] = i;
                        fan_temp->pre_warn[2] = 0;
                        fan_temp->pre_warn[3] = (TEMP_LABEL -   fan_temp->temp[chain_id][i]) * 5 / 7.5;
                        //fan_temp->pre_warn[3] = fan_temp->temp[chain_id][i];
                        printf("There maybe some problem in chain %d, chip %d,The highest temp %d\n",chain_id,i,fan_temp->temp[chain_id][i]);
                        //fan_temp->temp_highest[chain_id] = fan_temp->temp[chain_id][i];
                        // break;
                    }
#endif

                    if(stat_hi < 2)
                    {
                        high_avg += fan_temp->temp[chain_id][i];
                        fan_temp->pre_warn[2] = 0;
                        //printf("There maybe some problem in chain %d, chip %d,The highest temp %d\n",chain_id,i,fan_temp->temp[chain_id][i]);
                        stat_hi++;
                    }
                    else
                    {
                        break;
                    }
                }
            }
            if(stat_hi > 0)
            {
                fan_temp->temp_highest[chain_id] = (high_avg/stat_hi);
            }
            
            if(fan_temp->temp_highest[chain_id] < DANGEROUS_TMP)
            {
                fan_temp->pre_warn[0] = chain_id;
                // fan_temp->pre_warn[1] = i;
                fan_temp->pre_warn[2] = fan_temp->pre_warn[1] - 1;
                // fan_temp->pre_warn[3] = fan_temp->temp[chain_id][i];

                fan_temp->pre_warn[3] = (TEMP_LABEL -   fan_temp->temp_highest[chain_id]) * 5 / 7.5;
                // printf("There maybe some problem in chain %d, chip %d and chip %d,The highest temp %d\n",chain_id,i,i-1,fan_temp->temp_highest[chain_id]);
            }
            stat_hi = 0;

            break;
        case INNO_TYPE_A9:
            inno_log(IM_LOG_DEBUG,"Sorry do not have such type named INNO_TYPE_A9\n");
        default:
            break;

    }
//inno_log(IM_LOG_DEBUG,"chain %d, hi:%d\n",chain_id, fan_temp->temp_highest[chain_id]);
    pthread_mutex_unlock(&fan_temp->lock);
    return fan_temp->temp_highest[chain_id];
}

int inno_fan_temp_lowest(inno_fan_temp_s *fan_temp, int chain_id, inno_type_e inno_type)
{
    int i = 0;
    int low_avg = 0;
    static int stat_lo = 0;
    pthread_mutex_lock(&fan_temp->lock);

    switch(inno_type)
    {
        case INNO_TYPE_A4:
            inno_log(IM_LOG_DEBUG,"Sorry do not have such type named INNO_TYPE_A4\n");
            break;

        case INNO_TYPE_A5:
        case INNO_TYPE_A6:
        case INNO_TYPE_A7:
        case INNO_TYPE_A8:
            for(i=0; i<ASIC_CHIP_NUM; i++)
            {
                if(fan_temp->temp[chain_id][ASIC_CHIP_NUM-i-1] != 0)
                {
                    if(stat_lo < 2)
                    {
                        low_avg += fan_temp->temp[chain_id][ASIC_CHIP_NUM-i-1];
                        stat_lo++;
                    }
                    else
                    {
                       
                        break;
                    }
                }
            }
            if(stat_lo > 0)
            {
             fan_temp->temp_lowest[chain_id] = (low_avg/stat_lo);
            }
             stat_lo = 0;
    
            break;

        case INNO_TYPE_A9:
            inno_log(IM_LOG_DEBUG,"Sorry do not have such type named INNO_TYPE_A9\n");
        default:
            break;
    }
    pthread_mutex_unlock(&fan_temp->lock);
    return fan_temp->temp_lowest[chain_id];
}

int inno_fan_temp_avg(inno_fan_temp_s *fan_temp, int chain_id, inno_type_e inno_type)
{
    int i = 0;
    int temp_avg = 0;
    static int stat_avg = 0;
    pthread_mutex_lock(&fan_temp->lock);

    switch(inno_type)
    {
        case INNO_TYPE_A4:
            inno_log(IM_LOG_DEBUG,"Sorry do not have such type named INNO_TYPE_A4\n");
            break;

        case INNO_TYPE_A5:
        case INNO_TYPE_A6:
        case INNO_TYPE_A7:
        case INNO_TYPE_A8:
            for(i=0; i<ASIC_CHIP_NUM; i++)
            {
                if(fan_temp->temp[chain_id][i] != 0)
                    temp_avg += fan_temp->temp[chain_id][i];
                else
                    stat_avg++;
            }

            if(stat_avg != ASIC_CHIP_NUM)
                fan_temp->temp_arvarge[chain_id] = (temp_avg/(ASIC_CHIP_NUM - stat_avg));

            stat_avg = 0;
            break;

        case INNO_TYPE_A9:
            inno_log(IM_LOG_DEBUG,"Sorry do not have such type named INNO_TYPE_A9\n");
        default:
            break;

    }
    pthread_mutex_unlock(&fan_temp->lock);
    return fan_temp->temp_arvarge[chain_id];
}

void chain_temp_update(inno_fan_temp_s *fan_temp,int chain_id,inno_type_e inno_type)
{
    //applog(LOG_ERR, "Read:fan_ctrl->last_fan_tem = %d", fan_ctrl->last_fan_temp);
    asic_temp_sort(fan_temp, chain_id);
    inno_fan_temp_highest(fan_temp, chain_id,inno_type);
    inno_fan_temp_avg(fan_temp, chain_id,inno_type);
    inno_fan_temp_lowest(fan_temp, chain_id,inno_type);
    asic_temp_to_float(fan_temp, chain_id);
    asic_temp_clear(fan_temp, chain_id);

  return; 
}

void inno_fan_speed_update(inno_fan_temp_s *fan_temp)
{
    int i = 0;

	int temp_hi = DEFAULT_HI_TEMP; //fan_temp->temp_highest[0];
    int delta[8][2]={

        //{FAN_FIRST_STAGE + FAN_DELTA,0},
        {FAN_FIRST_STAGE + FAN_DELTA1, FAN_FIRST_STAGE - FAN_DELTA1},
        {FAN_SECOND_STAGE + FAN_DELTA1,FAN_SECOND_STAGE - FAN_DELTA1},
        {FAN_THIRD_STAGE + FAN_DELTA1, FAN_THIRD_STAGE - FAN_DELTA2},
        {FAN_FOUR_STAGE + FAN_DELTA2,  FAN_FOUR_STAGE - FAN_DELTA2},
        {FAN_FIVE_STAGE + FAN_DELTA2,  FAN_FIVE_STAGE - FAN_DELTA2,},
        {FAN_SIX_STAGE + FAN_DELTA2,   FAN_SIX_STAGE - FAN_DELTA2, },
        {FAN_SEVEN_STAGE + FAN_DELTA2, FAN_SEVEN_STAGE - FAN_DELTA2, },
        {FAN_EIGHT_STAGE + FAN_DELTA2, FAN_EIGHT_STAGE - FAN_DELTA2, },
    };

    //printf("fan_speed %d, %d, %d, %d\n",fan_level[0],fan_level[1],fan_level[2],fan_level[3]);

    //printf("level_speed %d, %d, %d, %d\n",fan_level[0],fan_level[1],fan_level[2],fan_level[3]);
    //printf("after fan_speed %d, %d, %d, %d\n",fan_speed[0],fan_speed[1],fan_speed[2],fan_speed[3]);
    fan_temp->auto_ctrl = g_auto_fan;
    
   for(i=0; i<ASIC_CHAIN_NUM; i++)
   {
   
     //inno_log(IM_LOG_DEBUG,"hi:%d lo:%d av:%d,valid %d\n",fan_temp->temp_highest[i],fan_temp->temp_lowest[i],fan_temp->temp_arvarge[i],fan_temp->valid_chain[i]);
     if((fan_temp->temp_highest[i] > ERR_LOW_TEMP) || (fan_temp->temp_highest[i] < ERR_HIGH_TEMP) || fan_temp->valid_chain[i])
        continue;

     if(temp_hi > fan_temp->temp_highest[i])
        temp_hi = fan_temp->temp_highest[i];
   }
    

    if(fan_temp->auto_ctrl)
    {
        if(temp_hi > delta[fan_temp->last_fan_temp][0])
        {
            if(fan_temp->last_fan_temp > 0)
            {
                fan_temp->last_fan_temp -= 1;
            }
            //applog(LOG_ERR, "%s +:arv:%5.2f, lest:%5.2f, hest:%5.2f, speed:%d%%", __func__, arvarge_f, lowest_f, highest_f, 100 - fan_ctrl->duty);
        }else if (temp_hi < delta[fan_temp->last_fan_temp][1])
        {
            if(fan_temp->last_fan_temp < 7)
            {
                fan_temp->last_fan_temp += 1;
            }
            //applog(LOG_ERR, "%s +:arv:%5.2f, lest:%5.2f, hest:%5.2f, speed:%d%%", __func__, arvarge_f, lowest_f, highest_f, 100 - fan_ctrl->duty);
        }
        fan_temp->speed = fan_speed[fan_temp->last_fan_temp];
        //printf("temp_highest %d, fan speed %d,last fan id: %d\n",fan_temp->temp_highest[chain_id],fan_speed[fan_temp->last_fan_temp],fan_temp->last_fan_temp);
    }else{
        fan_temp->speed = fan_speed[g_fan_speed];

    }
    if(fan_temp->speed != fan_temp->last_fan_speed)
    {
        fan_temp->last_fan_speed = fan_temp->speed;
        inno_fan_speed_set(fan_temp,fan_temp->speed);
    }

  // inno_log(IM_LOG_DEBUG,"hi %d,spd %d,lid: %d,md %d,list: %d, %d, %d, %d\n",temp_hi,fan_speed[fan_temp->last_fan_temp],fan_temp->last_fan_temp,fan_temp->auto_ctrl,fan_level[0],fan_level[1],fan_level[2],fan_level[3]);
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
    // inno_log_init(log_path, size);
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
