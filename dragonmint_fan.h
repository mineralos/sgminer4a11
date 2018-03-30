/******************************************************************************
 *
 * �ļ���  ��inno_fan.h
 * ������  �� yex(yex@innosilicon.com.cn)
 * �������ڣ� 20171122
 * �汾��  �� V1.0
 * �ļ������� �������¶ȿ��ƽӿ�
 * ��Ȩ˵���� Copyright (c) 2000-2020   GNU
 * ��    ���� ��
 * �޸���־�� ��
 *
 *******************************************************************************/

/*---------------------------------- Ԥ������ ---------------------------------*/
#ifndef _INNO_FAN_H_
#define _INNO_FAN_H_

/************************************ ͷ�ļ� ***********************************/
#include <stdio.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdbool.h>
#include <sys/ioctl.h>
#include <string.h>
#include <unistd.h>

#include "config.h"
#include "dragonmint_a11.h"
#include "inno_log.h"

/************************************ �궨�� ***********************************/

#define ASIC_INNO_FAN_PWM0_DEVICE_NAME  ("/dev/pwmgen0.0")

#define ASIC_INNO_FAN_PWM_STEP            (5)
#define ASIC_INNO_FAN_PWM_DUTY_MAX        (100)
#define ASIC_INNO_FAN_PWM_FREQ_TARGET     (20000)
#define ASIC_INNO_FAN_PWM_FREQ            (20000)
#define FAN_CNT                           (2)
#define ASIC_INNO_FAN_TEMP_MAX_THRESHOLD  (100)
#define ASIC_INNO_FAN_TEMP_UP_THRESHOLD   (55)
#define ASIC_INNO_FAN_TEMP_DOWN_THRESHOLD (35)
#define ERR_HIGH_TEMP                     (400)
#define ERR_LOW_TEMP                      (655)
#define FAN_FIRST_STAGE                   (535)//40
#define FAN_SECOND_STAGE                  (520)//50
#define FAN_THIRD_STAGE                   (505)//60
#define FAN_FOUR_STAGE                    (498)//65
#define FAN_FIVE_STAGE                    (490)//70
#define FAN_SIX_STAGE                     (483)//75
#define FAN_SEVEN_STAGE                   (475)//80
#define FAN_EIGHT_STAGE                   (468)//85

#define FAN_DELTA1                        (10)//6
#define FAN_DELTA2                        (5) //3
#define TEMP_LABEL                        (594)
#define ACTIVE_STAT                       (6)
#define START_FAN_TH                      (550)//30
#define PREHEAT_SPEED                     (0)
//#define DANGEROUS_TMP                     (460)//90
#define DANGEROUS_TMP                     (445)//100
#define PRE_DGR_TEMP                      (456)//92.x
#define DEFAULT_HI_TEMP                   (652) //-40
#define MAGIC_NUM                         (100)

#define IOCTL_SET_FREQ(X) _IOR(MAGIC_NUM, (2*X), char *)
#define IOCTL_SET_DUTY(X) _IOR(MAGIC_NUM, (2*X+1), char *)

/*********************************** ���Ͷ��� **********************************/

/*--------------------------------- �ӿ������� --------------------------------*/

/*********************************** ȫ�ֱ��� **********************************/
typedef struct {
    int temp[ASIC_CHAIN_NUM][ASIC_CHIP_NUM];    /* ���ڴ���������ϵ�оƬ�¶�*/
    int valid_chain[ASIC_CHAIN_NUM];           //�����жϸ�chain�Ƿ���Ч
    int index[ASIC_CHAIN_NUM];                  /*��Ӧ���ϵ�chip_id */

    int speed;                              /* 0 - 100�������÷���ת��(����32��) */
    int last_fan_speed;
    int auto_ctrl;
    int pre_warn[4];

    int temp_arvarge[ASIC_CHAIN_NUM];          /*��Ӧ���ϵ�ƽ���¶�*/
    int temp_highest[ASIC_CHAIN_NUM];            /*��Ӧ���ϵ�����¶�*/
    int temp_lowest[ASIC_CHAIN_NUM];             /*��Ӧ���ϵ�����¶�*/
    float temp2float[ASIC_CHAIN_NUM][3];         /*[][0]->highest,[][1]->avg, [][2]->lowest*/
    int last_fan_temp;
    pthread_mutex_t lock;                       /* lock */

}inno_fan_temp_s;

/*********************************** �ӿں��� **********************************/
void inno_fan_temp_init(inno_fan_temp_s *fan_temp,int *fan_level);   /*��Ҫ����Axϵ�г�ʼ�����ȿ������¶���ʾ*/
bool inno_fan_temp_add(inno_fan_temp_s *fan_temp,int chain_id, int chip_id, int temp); /*����ʵʱ����ͳ�Ƶ��ĵ�ǰ�¶�ֵ���쳣��¼*/
void asic_temp_sort(inno_fan_temp_s *fan_temp, int chain_id);   /*�Ե�����ͳ�Ƶ��������¶���һ����������*/
int inno_fan_temp_highest(inno_fan_temp_s *fan_temp, int chain_id, inno_type_e inno_type); /*�ṩ��ǰ��ͳ�Ƶ���ʵʱ����¶�(chip_type)*/
int inno_fan_temp_lowest(inno_fan_temp_s *fan_temp, int chain_id, inno_type_e inno_type); /*�ṩ��ǰ��ͳ�Ƶ���ʵʱ����£�chip_type��*/
int inno_fan_temp_avg(inno_fan_temp_s *fan_temp, int chain_id, inno_type_e inno_type); /*�ṩ��ǰ������оƬͳ�Ƶ�ʵʱƽ���¶�*/
//void inno_fan_temp_update(inno_fan_temp_s *fan_temp,int chain_id, inno_type_e inno_type, int *fan_level);  /*���ڸ��·���ת�����¶���ʾ����*/
void inno_fan_speed_update(inno_fan_temp_s *fan_temp);
void chain_temp_update(inno_fan_temp_s *fan_temp,int chain_id,inno_type_e inno_type);
void inno_fan_speed_set(inno_fan_temp_s *fan_temp, int speed);  /*���÷���ת�� */

#endif // #ifndef _INNO_FAN_TEMP_H_
