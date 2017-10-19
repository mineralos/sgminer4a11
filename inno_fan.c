#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/ioctl.h>

#include "inno_fan.h"
#include "asic_inno_cmd.h"

#define MAGIC_NUM  100 
#define IOCTL_SET_FREQ_0 _IOR(MAGIC_NUM, 0, char *)
#define IOCTL_SET_DUTY_0 _IOR(MAGIC_NUM, 1, char *)
#define IOCTL_SET_FREQ_1 _IOR(MAGIC_NUM, 2, char *)
#define IOCTL_SET_DUTY_1 _IOR(MAGIC_NUM, 3, char *)

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


static int inno_fan_temp_compare(const void *a, const void *b);
static void inno_fan_speed_max(INNO_FAN_CTRL_T *fan_ctrl);
static void inno_fan_pwm_set(INNO_FAN_CTRL_T *fan_ctrl, int duty);

void inno_fan_init(INNO_FAN_CTRL_T *fan_ctrl)
{
    int chain_id = 0;

    mutex_init(&fan_ctrl->lock);

    inno_fan_pwm_set(fan_ctrl, 10); /* 90% */
    sleep(1);
    inno_fan_pwm_set(fan_ctrl, 20); /* 80% */

    for(chain_id = 0; chain_id < ASIC_CHAIN_NUM; chain_id++)
    {
        inno_fan_temp_clear(fan_ctrl, chain_id);
    }

	fan_ctrl->temp_nums = sizeof(inno_tsadc_table) / sizeof(inno_tsadc_table[0]);
    fan_ctrl->temp_v_min = inno_tsadc_table[fan_ctrl->temp_nums - 1];
    fan_ctrl->temp_v_max = inno_tsadc_table[0];
    fan_ctrl->temp_f_step = 5.0f;
    fan_ctrl->temp_f_min = -40.0f;
    fan_ctrl->temp_f_max = fan_ctrl->temp_f_min + fan_ctrl->temp_f_step * (fan_ctrl->temp_nums - 1);

	applog(LOG_ERR, "chip nums:%d.", ASIC_CHIP_A_BUCKET);
	applog(LOG_ERR, "pwm  name:%s.", ASIC_INNO_FAN_PWM0_DEVICE_NAME);
	applog(LOG_ERR, "pwm  step:%d.", ASIC_INNO_FAN_PWM_STEP);
	applog(LOG_ERR, "duty max: %d.", ASIC_INNO_FAN_PWM_DUTY_MAX);
	applog(LOG_ERR, "targ freq:%d.", ASIC_INNO_FAN_PWM_FREQ_TARGET);
	applog(LOG_ERR, "freq rate:%d.", ASIC_INNO_FAN_PWM_FREQ);
	applog(LOG_ERR, "max  thrd:%5.2f.", ASIC_INNO_FAN_TEMP_MAX_THRESHOLD);
	applog(LOG_ERR, "up   thrd:%5.2f.", ASIC_INNO_FAN_TEMP_UP_THRESHOLD);
	applog(LOG_ERR, "down thrd:%5.2f.", ASIC_INNO_FAN_TEMP_DOWN_THRESHOLD);
	applog(LOG_ERR, "temp nums:%d.", fan_ctrl->temp_nums);
	applog(LOG_ERR, "temp vmin:%d.", fan_ctrl->temp_v_min);
	applog(LOG_ERR, "temp vmax:%d.", fan_ctrl->temp_v_max);
	applog(LOG_ERR, "temp fstp:%5.2f.", fan_ctrl->temp_f_step);
	applog(LOG_ERR, "temp fmin:%5.2f.", fan_ctrl->temp_f_min);
	applog(LOG_ERR, "temp fmax:%5.2f.", fan_ctrl->temp_f_max);
}

void inno_fan_temp_add(INNO_FAN_CTRL_T *fan_ctrl, int chain_id, int temp, bool warn_on)
{
    float temp_f = 0.0f;
    int index = 0;

    index = fan_ctrl->index[chain_id];

    applog(LOG_INFO, "inno_fan_temp_add:chain_%d,chip_%d,temp:%7.4f(%d)", chain_id, index, inno_fan_temp_to_float(fan_ctrl, temp), temp);
    fan_ctrl->temp[chain_id][index] = temp;
    index++;
    fan_ctrl->index[chain_id] = index; 

    if(!warn_on)
    {
        return;
    }

    /* applog(LOG_ERR, "inno_fan_temp_add: temp warn_on(init):%d\n", warn_on); */
    temp_f = inno_fan_temp_to_float(fan_ctrl, temp);
    if(temp_f > ASIC_INNO_FAN_TEMP_MAX_THRESHOLD)
    { 
        applog(LOG_DEBUG, "inno_fan_temp_add:chain_%d,chip_%d,temp:%7.4f(%d) is too high!\n", chain_id, index, temp_f, temp);
    }
}

static void inno_fan_temp_sort(INNO_FAN_CTRL_T *fan_ctrl, int chain_id)
{
    int i = 0;
    int temp_nums = 0;

    temp_nums = fan_ctrl->index[chain_id];

    applog(LOG_DEBUG, "not sort:");
    for(i = 0; i < temp_nums; i++)
    {
        applog(LOG_DEBUG, "chip_%d:%08x(%d)", i, fan_ctrl->temp[chain_id][i], fan_ctrl->temp[chain_id][i]);
    }
    applog(LOG_DEBUG, "sorted:");
    qsort(fan_ctrl->temp[chain_id], temp_nums, sizeof(fan_ctrl->temp[chain_id][0]), inno_fan_temp_compare);
    for(i = 0; i < temp_nums; i++)
    {
        applog(LOG_DEBUG, "chip_%d:%08x(%d)", i, fan_ctrl->temp[chain_id][i], fan_ctrl->temp[chain_id][i]);
    }
    applog(LOG_DEBUG, "sort end.");
}

static int inno_fan_temp_get_arvarge(INNO_FAN_CTRL_T *fan_ctrl, int chain_id)
{
    int   i = 0;
    int   temp_nums = 0;
    int   head_index = 0;
    int   tail_index = 0;
    float arvarge_temp = 0.0f; 

    temp_nums = fan_ctrl->index[chain_id];
    /* step1: delete temp (0, ASIC_INNO_FAN_TEMP_MARGIN_NUM) & (max - ASIC_INNO_FAN_TEMP_MARGIN_NUM, max) */
    head_index = temp_nums * ASIC_INNO_FAN_TEMP_MARGIN_RATE;
    tail_index = temp_nums - head_index;
    /* ��ֹԽ�� */
    if(head_index < 0)
    {
        head_index = 0;
    }
    if(tail_index < 0)
    {
        tail_index = head_index;
    }

    /* step2: arvarge */
    for(i = head_index; i < tail_index; i++)
    {
        arvarge_temp += fan_ctrl->temp[chain_id][i];
    }
    arvarge_temp /= (tail_index - head_index);

    float temp_f = 0.0f;
    temp_f = inno_fan_temp_to_float(fan_ctrl, (int)arvarge_temp);
	applog(LOG_DEBUG, "inno_fan_temp_get_arvarge, chain_id:%d, temp nums:%d, valid index[%d,%d], reseult:%7.4f(%d).\n",
            chain_id, temp_nums, head_index, tail_index, inno_fan_temp_to_float(fan_ctrl, (int)arvarge_temp), (int)arvarge_temp); 

    return (int)arvarge_temp;
}

static int inno_fan_temp_get_highest(INNO_FAN_CTRL_T *fan_ctrl, int chain_id)
{
    return fan_ctrl->temp[chain_id][0];
}

static int inno_fan_temp_get_lowest(INNO_FAN_CTRL_T *fan_ctrl, int chain_id)
{
    int temp_nums = 0;
    int index = 0;

    temp_nums = fan_ctrl->index[chain_id];
    index = temp_nums - 1;

    /* ����Խ�� */
    if(index < 0)
    {
        index = 0;
    }

    return fan_ctrl->temp[chain_id][index];
}

void inno_fan_temp_clear(INNO_FAN_CTRL_T *fan_ctrl, int chain_id)
{
    int i = 0;

    fan_ctrl->index[chain_id] = 0;
    for(i = 0; i < ASIC_CHIP_NUM; i++)
    {
        fan_ctrl->temp[chain_id][i] = 0;
    }
}

void inno_fan_temp_init(INNO_FAN_CTRL_T *fan_ctrl, int chain_id)
{
    int temp = 0;

    inno_fan_temp_sort(fan_ctrl, chain_id);

    temp = inno_fan_temp_get_arvarge(fan_ctrl, chain_id);
    fan_ctrl->temp_init[chain_id] = temp;
    fan_ctrl->temp_arvarge[chain_id] = temp;

    temp = inno_fan_temp_get_highest(fan_ctrl, chain_id);
    fan_ctrl->temp_highest[chain_id] = temp;

    temp = inno_fan_temp_get_lowest(fan_ctrl, chain_id);
    fan_ctrl->temp_lowest[chain_id] = temp;

    inno_fan_temp_clear(fan_ctrl, chain_id);
}

void inno_fan_pwm_set(INNO_FAN_CTRL_T *fan_ctrl, int duty)
{
    int fd = 0;
    int duty_driver = 0;

    duty_driver = ASIC_INNO_FAN_PWM_FREQ_TARGET / 100 * duty;

	mutex_lock(&fan_ctrl->lock);

    /* �������Ƚ�� */
    fd = open(ASIC_INNO_FAN_PWM0_DEVICE_NAME, O_RDWR);
    if(fd < 0)
    {
        applog(LOG_ERR, "open %s fail", ASIC_INNO_FAN_PWM0_DEVICE_NAME);
        mutex_unlock(&fan_ctrl->lock);
        return;
    }
    if(ioctl(fd, IOCTL_SET_FREQ_0, ASIC_INNO_FAN_PWM_FREQ) < 0)
    {
        applog(LOG_ERR, "set fan0 frequency fail");
        mutex_unlock(&fan_ctrl->lock);
        return;
    }
    if(ioctl(fd, IOCTL_SET_DUTY_0, duty_driver) < 0)
    {
        applog(LOG_ERR, "set duty fail \n");
        mutex_unlock(&fan_ctrl->lock);
        return;
    }
    close(fd);

    fan_ctrl->duty = duty;

    mutex_unlock(&fan_ctrl->lock);
}

void inno_fan_speed_up(INNO_FAN_CTRL_T *fan_ctrl)
{
    int duty = 0;
    
    /* �Ѿ��������ֵ,���� */
    if(0 == fan_ctrl->duty)
    {
        return;
    }

    duty = fan_ctrl->duty;
    duty -= ASIC_INNO_FAN_PWM_STEP;
    if(duty < 0)
    {
        duty = 0;
    } 
    applog(LOG_DEBUG, "speed+(%02d%% to %02d%%)" , 100 - fan_ctrl->duty, 100 - duty);

    inno_fan_pwm_set(fan_ctrl, duty);
}

void inno_fan_speed_down(INNO_FAN_CTRL_T *fan_ctrl)
{
    int duty = 0;

    /* �Ѿ�������Сֵ,���� */
    if(ASIC_INNO_FAN_PWM_DUTY_MAX == fan_ctrl->duty)
    {
        return;
    }

    duty = fan_ctrl->duty;
    duty += ASIC_INNO_FAN_PWM_STEP;
    if(duty > ASIC_INNO_FAN_PWM_DUTY_MAX)
    {
        duty = ASIC_INNO_FAN_PWM_DUTY_MAX;
    }
    applog(LOG_DEBUG, "speed-(%02d%% to %02d%%)" , 100 - fan_ctrl->duty, 100 - duty);

    inno_fan_pwm_set(fan_ctrl, duty);
}

void inno_fan_speed_update(INNO_FAN_CTRL_T *fan_ctrl, int chain_id, struct cgpu_info *cgpu)
{
	struct A1_chain *a1 = cgpu->device_data;
    int arvarge = 0;       
    int highest = 0;       
    int lowest  = 0;       

    float arvarge_f = 0.0f;
    float highest_f = 0.0f;
    float lowest_f  = 0.0f;

    inno_fan_temp_sort(fan_ctrl, chain_id);
    arvarge = inno_fan_temp_get_arvarge(fan_ctrl, chain_id);
    highest = inno_fan_temp_get_highest(fan_ctrl, chain_id);
    lowest  = inno_fan_temp_get_lowest(fan_ctrl, chain_id);
    inno_fan_temp_clear(fan_ctrl, chain_id);

    //applog(LOG_DEBUG, "chain_%d, init:%7.4f,now:%7.4f,delta:%7.4f",
    /* 
    applog(LOG_DEBUG, "chain_%d, init:%7.4f(%7.4f),now:%7.4f(%7.4f),delta:%7.4f(%7.4f)",
            chain_id, 
            inno_fan_temp_to_float(fan_ctrl, fan_ctrl->temp_init[chain_id]), fan_ctrl->temp_init[chain_id], 
            inno_fan_temp_to_float(fan_ctrl, fan_ctrl->temp_now[chain_id]), fan_ctrl->temp_now[chain_id],
            fan_ctrl->temp_delta[chain_id]);
            */
 
    arvarge_f = inno_fan_temp_to_float(fan_ctrl, (int)arvarge);
    lowest_f = inno_fan_temp_to_float(fan_ctrl, (int)lowest);
    highest_f = inno_fan_temp_to_float(fan_ctrl, (int)highest);

    /* ����������Power Down */
    if(highest_f > ASIC_INNO_FAN_TEMP_MAX_THRESHOLD)
    {
        //applog(LOG_ERR, "%s z:arv:%5.2f, lest:%5.2f, hest:%5.2f, power down", __func__, arvarge_f, lowest_f, highest_f);
    }

    /* �¶ȹ��� */
    if(highest_f > ASIC_INNO_FAN_TEMP_UP_THRESHOLD)
    {
        if(0 != fan_ctrl->duty)
        {
            inno_fan_pwm_set(fan_ctrl, 0);
            applog(LOG_ERR, "%s +:arv:%5.2f, lest:%5.2f, hest:%5.2f, speed:%d%%", __func__, arvarge_f, lowest_f, highest_f, 100 - fan_ctrl->duty);
        } 
    }

    /* �¶��Ѿ��ָ����� */
    if(highest_f < ASIC_INNO_FAN_TEMP_DOWN_THRESHOLD)
    {
        if(40 != fan_ctrl->duty) 
        {
            inno_fan_pwm_set(fan_ctrl, 40);
            applog(LOG_ERR, "%s -:arv:%5.2f, lest:%5.2f, hest:%5.2f, speed:%d%%", __func__, arvarge_f, lowest_f, highest_f, 100 - fan_ctrl->duty);
        }
    } 

    cgpu->temp = arvarge_f;
    cgpu->temp_max = highest_f;
    cgpu->temp_min = lowest_f;
    cgpu->fan_duty = 100 - fan_ctrl->duty;
            
    cgpu->chip_num = a1->num_active_chips;
    cgpu->core_num = a1->num_cores; 

    applog(LOG_INFO, "%s n:arv:%5.2f, lest:%5.2f, hest:%5.2f", __func__, arvarge_f, lowest_f, highest_f);
}

static void inno_fan_speed_max(INNO_FAN_CTRL_T *fan_ctrl)
{
    inno_fan_pwm_set(fan_ctrl, 0);
}

static int inno_fan_temp_compare(const void *a, const void *b)
{
    return *(int *)a - *(int *)b;
}

float inno_fan_temp_to_float(INNO_FAN_CTRL_T *fan_ctrl, int temp)
{
    int i = 0;
    int i_max = 0;
    float temp_f_min = 0.0f;
    float temp_f_step = 0.0f;

    float temp_f_start = 0.0f;
    float temp_f_end = 0.0f;
    int temp_v_start = 0;
    int temp_v_end = 0;
    float temp_f = 0.0f;

    if(temp < fan_ctrl->temp_v_min)
    {
        return 9999.0f;
    }
    if(temp > fan_ctrl->temp_v_max)
    {
        return -9999.0f;
    } 
    
    i_max = fan_ctrl->temp_nums;
    for(i = 1; i < i_max - 1; i++)
    {
        if(temp > inno_tsadc_table[i])
        {
            break;
        }
    }

    temp_f_min = fan_ctrl->temp_f_min;
    temp_f_step = fan_ctrl->temp_f_step;

     /*
     * (x - temp_f_start) / temp_f_step = (temp - temp_v_start) / (temp_v_end - temp_v_start)
     *
     * x = temp_f_start + temp_f_step * (temp - temp_v_start) / (temp_v_end - temp_v_start)
     *
     * */
    temp_f_end = temp_f_min + i * temp_f_step;
    temp_f_start = temp_f_end - temp_f_step;
    temp_v_start = inno_tsadc_table[i - 1];
    temp_v_end = inno_tsadc_table[i]; 

    temp_f = temp_f_start + temp_f_step * (temp - temp_v_start) / (temp_v_end - temp_v_start);

    applog(LOG_DEBUG, "inno_fan_temp_to_float: temp:%d,%d,%d, %7.4f,%7.4f" , temp,
            temp_v_start, temp_v_end, temp_f_start, temp_f_end);
    applog(LOG_DEBUG, "inno_fan_temp_to_float: :%7.4f,%7.4f,%d,%d",
            temp_f_start, temp_f_step,
            temp - temp_v_start, temp_v_end - temp_v_start);

    return temp_f;
}

void inno_temp_contrl(INNO_FAN_CTRL_T *fan_ctrl, struct A1_chain *a1, int chain_id)
{
	int i;
	int arvarge = 0;
    float arvarge_f = 0.0f; 
	uint8_t reg[REG_LENGTH];

	arvarge_f = inno_fan_temp_to_float(fan_ctrl, fan_ctrl->temp_arvarge[chain_id]);
	applog(LOG_ERR,"---Read Temp:%.2f\n",arvarge_f);
	if(arvarge_f >= 25.0){
		return;
	}

	while(arvarge_f < 25.0){
		for (i = a1->num_active_chips; i > 0; i--){	
			if (!inno_cmd_read_reg(a1, i, reg)){
				applog(LOG_ERR, "%d: Failed to read temperature sensor register for chip %d ", a1->chain_id, i);
				continue;
			}
			/* update temp database */
            uint32_t temp = 0;
            float    temp_f = 0.0f;

            temp = 0x000003ff & ((reg[7] << 8) | reg[8]);
            inno_fan_temp_add(fan_ctrl, a1->chain_id, temp, false);
		} 

		inno_fan_temp_init(fan_ctrl, a1->chain_id);
		arvarge_f = inno_fan_temp_to_float(fan_ctrl, fan_ctrl->temp_arvarge[a1->chain_id]);
		applog(LOG_WARNING, "%s +:arv:%7.4f. \t \n", __func__, arvarge_f);
		inno_fan_pwm_set(fan_ctrl, 100);
		sleep(1);
	}
	
	inno_fan_pwm_set(fan_ctrl, 10);
}
